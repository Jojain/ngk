use nalgebra::{Point2, Point4, UnitVector3, Vector3, Vector4};
use serde::{Deserialize, Serialize};

use super::curve::{InterpolationSystem, KNOT_TOLERANCE, NurbsCurve};
use super::degree::Degree;
use super::knots::KnotVector;
use super::points::{ControlNet, ControlPolygon, HPoint};
use crate::geometry::counters::{count_global_surface_projection, count_hinted_surface_projection};
use crate::geometry::nurbs::basis::{basis_function_derivatives, basis_functions};
use crate::geometry::nurbs::error::{NurbsError, SkinningIncompatibility};
use crate::geometry::{BBox, Interval, LINEAR_TOLERANCE, Point3};

/// Fewest grid samples per direction a closest-point search starts from.
const MIN_CLOSEST_SAMPLES: usize = 12;
/// Most grid samples per direction a closest-point search starts from.
const MAX_CLOSEST_SAMPLES: usize = 512;
/// How many of the nearest grid samples a closest-point search refines.
const CLOSEST_POINT_STARTS: usize = 4;

/// One exact rational Bézier patch extracted from a parent NURBS surface.
#[derive(Debug, Clone)]
pub struct BezierSurface {
    surface: NurbsSurface,
}

impl BezierSurface {
    /// Returns the patch domain in the parent surface's u parameter.
    pub fn domain_u(&self) -> Interval {
        self.surface.domain_u()
    }

    /// Returns the patch domain in the parent surface's v parameter.
    pub fn domain_v(&self) -> Interval {
        self.surface.domain_v()
    }

    /// Returns the patch's rational control net.
    pub fn control_points(&self) -> &ControlNet {
        self.surface.control_points()
    }

    /// Evaluates the patch in parent-surface parameters.
    pub fn point_at(&self, u: f64, v: f64) -> Point3 {
        self.surface.point_at(u, v)
    }

    /// Returns the patch as a NURBS surface over its own parameter box.
    pub fn surface(&self) -> &NurbsSurface {
        &self.surface
    }

    /// Returns the patch normal in parent-surface parameters.
    pub fn normal_at(&self, u: f64, v: f64) -> UnitVector3<f64> {
        self.surface.normal_at(u, v)
    }

    /// Returns a conservative control-hull bound for positive weights.
    pub fn bbox(&self) -> BBox {
        BBox::from_points(
            self.control_points()
                .as_slice()
                .iter()
                .map(|point| point.to_cartesian()),
        )
    }

    /// Returns whether every control weight is finite and positive.
    ///
    /// The control hull bounds the patch only under this condition, which every
    /// embedding-based rejection test relies on.
    pub fn has_positive_weights(&self) -> bool {
        self.control_points()
            .as_slice()
            .iter()
            .all(|point| point.weight().is_finite() && point.weight() > 0.0)
    }

    /// Splits the patch at `u`, keeping both halves in parent-surface parameters.
    pub fn subdivide_u(&self, u: f64) -> Result<(Self, Self), NurbsError> {
        let domain = self.domain_u();
        if u <= domain.start.value() || u >= domain.end.value() {
            return Err(NurbsError::DegenerateInterval {
                start: domain.start.value(),
                end: u,
            });
        }
        let degree = self.surface.degree_u;
        let p = degree.get();
        let mut refined = self.surface.clone();
        for _ in 0..p {
            refined.insert_knot_u(u)?;
        }
        let nv = refined.control_points.nv();
        let mut left = Vec::with_capacity((p + 1) * nv);
        let mut right = Vec::with_capacity((p + 1) * nv);
        for v in 0..nv {
            for i in 0..=p {
                left.push(refined.control_points.get(i, v));
                right.push(refined.control_points.get(p + i, v));
            }
        }
        Ok((
            self.rebuilt_u(left, Interval::new(domain.start.value(), u))?,
            self.rebuilt_u(right, Interval::new(u, domain.end.value()))?,
        ))
    }

    /// Splits the patch at `v`, keeping both halves in parent-surface parameters.
    pub fn subdivide_v(&self, v: f64) -> Result<(Self, Self), NurbsError> {
        let domain = self.domain_v();
        if v <= domain.start.value() || v >= domain.end.value() {
            return Err(NurbsError::DegenerateInterval {
                start: domain.start.value(),
                end: v,
            });
        }
        let degree = self.surface.degree_v;
        let q = degree.get();
        let mut refined = self.surface.clone();
        for _ in 0..q {
            refined.insert_knot_v(v)?;
        }
        let nu = refined.control_points.nu();
        let mut lower = Vec::with_capacity(nu * (q + 1));
        let mut upper = Vec::with_capacity(nu * (q + 1));
        for j in 0..=q {
            for u in 0..nu {
                lower.push(refined.control_points.get(u, j));
                upper.push(refined.control_points.get(u, q + j));
            }
        }
        Ok((
            self.rebuilt_v(lower, Interval::new(domain.start.value(), v))?,
            self.rebuilt_v(upper, Interval::new(v, domain.end.value()))?,
        ))
    }

    fn rebuilt_u(&self, points: Vec<HPoint>, domain_u: Interval) -> Result<Self, NurbsError> {
        let degree_u = self.surface.degree_u;
        Ok(Self {
            surface: NurbsSurface::new(
                degree_u,
                self.surface.degree_v,
                ControlNet::new(points, degree_u.get() + 1, self.surface.control_points.nv())?,
                bezier_knots(degree_u, domain_u.start.value(), domain_u.end.value())?,
                self.surface.knots_v.clone(),
            )?,
        })
    }

    fn rebuilt_v(&self, points: Vec<HPoint>, domain_v: Interval) -> Result<Self, NurbsError> {
        let degree_v = self.surface.degree_v;
        Ok(Self {
            surface: NurbsSurface::new(
                self.surface.degree_u,
                degree_v,
                ControlNet::new(points, self.surface.control_points.nu(), degree_v.get() + 1)?,
                self.surface.knots_u.clone(),
                bezier_knots(degree_v, domain_v.start.value(), domain_v.end.value())?,
            )?,
        })
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NurbsSurface {
    degree_u: Degree,
    degree_v: Degree,
    control_points: ControlNet,
    knots_u: KnotVector,
    knots_v: KnotVector,
}

impl NurbsSurface {
    pub fn new(
        degree_u: Degree,
        degree_v: Degree,
        control_points: ControlNet,
        knots_u: KnotVector,
        knots_v: KnotVector,
    ) -> Result<Self, NurbsError> {
        let expected_u = control_points.nu() + degree_u.get() + 1;
        if knots_u.len() != expected_u {
            return Err(NurbsError::KnotCountMismatch {
                expected: expected_u,
                got: knots_u.len(),
            });
        }
        let expected_v = control_points.nv() + degree_v.get() + 1;
        if knots_v.len() != expected_v {
            return Err(NurbsError::KnotCountMismatch {
                expected: expected_v,
                got: knots_v.len(),
            });
        }
        Ok(Self {
            degree_u,
            degree_v,
            control_points,
            knots_u,
            knots_v,
        })
    }

    /// This surface with every control point moved by `f`.
    ///
    /// Degrees, weights and both knot vectors are kept, so the result is the
    /// image of this surface under `f` with the same parameterization whenever
    /// `f` is affine. Infallible because the count invariants [`Self::new`]
    /// enforces are untouched by a point-wise map.
    pub(crate) fn map_control_points(&self, f: impl Fn(Point3) -> Point3) -> Self {
        Self {
            degree_u: self.degree_u,
            degree_v: self.degree_v,
            control_points: self.control_points.map_points(f),
            knots_u: self.knots_u.clone(),
            knots_v: self.knots_v.clone(),
        }
    }

    pub fn with_uniform_knots(
        degree_u: Degree,
        degree_v: Degree,
        control_points: ControlNet,
    ) -> Result<Self, NurbsError> {
        let knots_u = KnotVector::uniform_clamped(control_points.nu(), degree_u);
        let knots_v = KnotVector::uniform_clamped(control_points.nv(), degree_v);
        Self::new(degree_u, degree_v, control_points, knots_u, knots_v)
    }

    pub fn degree_u(&self) -> Degree {
        self.degree_u
    }
    pub fn degree_v(&self) -> Degree {
        self.degree_v
    }
    pub fn control_points(&self) -> &ControlNet {
        &self.control_points
    }
    pub fn knots_u(&self) -> &KnotVector {
        &self.knots_u
    }
    pub fn knots_v(&self) -> &KnotVector {
        &self.knots_v
    }

    pub fn domain_u(&self) -> Interval {
        self.knots_u.domain(self.degree_u)
    }
    pub fn domain_v(&self) -> Interval {
        self.knots_v.domain(self.degree_v)
    }

    /// Whether any control point is weighted differently from the others.
    ///
    /// A net of equal weights describes the same surface as a net of ones, so
    /// what this answers is whether the weights carry shape — which is the
    /// question a writer asks before deciding whether they have to be written
    /// at all. Mirrors [`NurbsCurve::is_rational`].
    pub fn is_rational(&self) -> bool {
        let points = self.control_points.as_slice();
        let Some(first) = points.first().map(|point| point.weight()) else {
            return false;
        };
        points
            .iter()
            .any(|point| (point.weight() - first).abs() > LINEAR_TOLERANCE)
    }

    pub fn point_at(&self, u: f64, v: f64) -> Point3 {
        let domain_u = self.domain_u();
        let domain_v = self.domain_v();
        let u = u.clamp(domain_u.start.value(), domain_u.end.value());
        let v = v.clamp(domain_v.start.value(), domain_v.end.value());

        let p = self.degree_u.get();
        let q = self.degree_v.get();
        let n = self.control_points.nu() - 1;
        let m = self.control_points.nv() - 1;

        let span_u = self.knots_u.find_span(n, self.degree_u, u);
        let span_v = self.knots_v.find_span(m, self.degree_v, v);
        let nu = basis_functions(span_u, u, self.degree_u, &self.knots_u);
        let nv = basis_functions(span_v, v, self.degree_v, &self.knots_v);

        let mut acc = Point4::origin();
        for j in 0..=q {
            let mut temp = Point4::origin();
            for i in 0..=p {
                let hp = self.control_points.get(span_u - p + i, span_v - q + j);
                let contrib: Vector4<f64> = hp.0.coords * nu[i];
                temp.coords += contrib;
            }
            acc.coords += temp.coords * nv[j];
        }
        HPoint(acc).to_cartesian()
    }

    /// Unit surface normal at `(u, v)` using first partial derivatives in
    /// homogeneous space, then the quotient rule for the w-divide.
    pub fn normal_at(&self, u: f64, v: f64) -> UnitVector3<f64> {
        let (du, dv) = self.derivatives_uv(u, v);
        UnitVector3::new_normalize(du.cross(&dv))
    }

    /// Returns the parameters of the point on the surface nearest `point`.
    ///
    /// Newton converges only from a start already in the right basin, and a
    /// surface that folds back near itself -- a thread flank winding round
    /// many turns -- has one basin per fold. The starts therefore come from a
    /// grid as fine as the control net, which is what bounds how often the
    /// surface can turn between two samples, and the few closest are all
    /// refined, keeping whichever lands nearest.
    pub fn closest_parameter(&self, point: Point3) -> Point2<f64> {
        count_global_surface_projection();
        self.closest_sample_parameters(point)
            .into_iter()
            .map(|(u, v)| self.refine_closest_parameter(point, u, v))
            .min_by(|a, b| {
                let distance = |uv: &Point2<f64>| (self.point_at(uv.x, uv.y) - point).norm();
                distance(a).total_cmp(&distance(b))
            })
            .expect("the sample grid is never empty")
    }

    /// Returns the parameters of `point` refined from `hint` alone, or `None`
    /// when that start does not land within `tolerance` of it.
    ///
    /// [`Self::closest_parameter`] searches the whole grid because it knows
    /// nothing of where `point` lies. A caller walking a curve on the surface
    /// does: the previous point's parameters are a start inside the right
    /// basin. Newton from a start on another fold still converges, to a foot
    /// on that fold a fold's spacing away, so the answer is only kept when
    /// the foot is the point itself. A point lies on one fold only, so a
    /// foot within `tolerance` is its own.
    pub fn closest_parameter_near(
        &self,
        point: Point3,
        hint: Point2<f64>,
        tolerance: f64,
    ) -> Option<Point2<f64>> {
        let uv = self.refine_closest_parameter(point, hint.x, hint.y);
        ((self.point_at(uv.x, uv.y) - point).norm() <= tolerance).then(|| {
            count_hinted_surface_projection();
            uv
        })
    }

    /// Gauss-Newton from `(u, v)`, clamped to the domain.
    fn refine_closest_parameter(&self, point: Point3, mut u: f64, mut v: f64) -> Point2<f64> {
        let domain_u = self.domain_u();
        let domain_v = self.domain_v();

        for _ in 0..10 {
            let surface_point = self.point_at(u, v);
            let residual = surface_point - point;
            let (du, dv) = self.derivatives_uv(u, v);
            let uu = du.dot(&du);
            let uv = du.dot(&dv);
            let vv = dv.dot(&dv);
            let ru = du.dot(&residual);
            let rv = dv.dot(&residual);
            let determinant = uu * vv - uv * uv;

            if determinant.abs() <= 1.0e-14 {
                break;
            }

            let delta_u = (vv * ru - uv * rv) / determinant;
            let delta_v = (uu * rv - uv * ru) / determinant;
            u = (u - delta_u).clamp(domain_u.start.value(), domain_u.end.value());
            v = (v - delta_v).clamp(domain_v.start.value(), domain_v.end.value());

            if delta_u.hypot(delta_v) <= 1.0e-10 {
                break;
            }
        }

        Point2::new(u, v)
    }

    /// The grid samples nearest `point`, closest first.
    ///
    /// Each direction takes two samples per control point, and never fewer
    /// than [`MIN_CLOSEST_SAMPLES`], so a small patch is searched exactly as
    /// finely as before while a long one is not searched more coarsely.
    fn closest_sample_parameters(&self, point: Point3) -> Vec<(f64, f64)> {
        let samples = |count: usize| (2 * count).clamp(MIN_CLOSEST_SAMPLES, MAX_CLOSEST_SAMPLES);
        let samples_u = samples(self.control_points.nu());
        let samples_v = samples(self.control_points.nv());
        let domain_u = self.domain_u();
        let domain_v = self.domain_v();
        let mut candidates = Vec::with_capacity((samples_u + 1) * (samples_v + 1));

        for i in 0..=samples_u {
            let u = domain_u.start.value() + domain_u.length() * (i as f64 / samples_u as f64);
            for j in 0..=samples_v {
                let v = domain_v.start.value() + domain_v.length() * (j as f64 / samples_v as f64);
                candidates.push(((self.point_at(u, v) - point).norm_squared(), (u, v)));
            }
        }

        candidates.sort_by(|a, b| a.0.total_cmp(&b.0));
        candidates
            .into_iter()
            .take(CLOSEST_POINT_STARTS)
            .map(|(_, uv)| uv)
            .collect()
    }

    /// Returns `(dS/du, dS/dv)` in cartesian space.
    pub fn derivatives_uv(&self, u: f64, v: f64) -> (Vector3<f64>, Vector3<f64>) {
        let domain_u = self.domain_u();
        let domain_v = self.domain_v();
        let u = u.clamp(domain_u.start.value(), domain_u.end.value());
        let v = v.clamp(domain_v.start.value(), domain_v.end.value());

        let p = self.degree_u.get();
        let q = self.degree_v.get();
        let n = self.control_points.nu() - 1;
        let m = self.control_points.nv() - 1;

        let span_u = self.knots_u.find_span(n, self.degree_u, u);
        let span_v = self.knots_v.find_span(m, self.degree_v, v);
        let du_basis = basis_function_derivatives(span_u, u, self.degree_u, &self.knots_u, 1);
        let dv_basis = basis_function_derivatives(span_v, v, self.degree_v, &self.knots_v, 1);

        let mut s = Point4::origin();
        let mut s_u = Vector4::zeros();
        let mut s_v = Vector4::zeros();

        for j in 0..=q {
            let mut row = Point4::origin();
            let mut row_u = Vector4::zeros();
            for i in 0..=p {
                let hp = self.control_points.get(span_u - p + i, span_v - q + j).0;
                row.coords += hp.coords * du_basis[0][i];
                row_u += hp.coords * du_basis[1][i];
            }
            s.coords += row.coords * dv_basis[0][j];
            s_u += row_u * dv_basis[0][j];
            s_v += row.coords * dv_basis[1][j];
        }

        let w = s.w;
        let s_xyz = Vector3::new(s.x, s.y, s.z);
        let ds_u_xyz = Vector3::new(s_u.x, s_u.y, s_u.z);
        let ds_v_xyz = Vector3::new(s_v.x, s_v.y, s_v.z);
        let du = (ds_u_xyz - s_xyz * (s_u.w / w)) / w;
        let dv = (ds_v_xyz - s_xyz * (s_v.w / w)) / w;
        (du, dv)
    }

    /// Decomposes this surface exactly into rational Bézier knot spans.
    pub fn bezier_spans(&self) -> Result<Vec<BezierSurface>, NurbsError> {
        let mut refined = self.clone();
        let domain_u = refined.domain_u();
        let domain_v = refined.domain_v();
        for knot in distinct_interior_knots(refined.knots_u.as_slice(), domain_u) {
            while refined.knots_u.multiplicity(knot) < refined.degree_u.get() {
                refined.insert_knot_u(knot)?;
            }
        }
        for knot in distinct_interior_knots(refined.knots_v.as_slice(), domain_v) {
            while refined.knots_v.multiplicity(knot) < refined.degree_v.get() {
                refined.insert_knot_v(knot)?;
            }
        }

        let u_breaks = distinct_domain_knots(refined.knots_u.as_slice(), domain_u);
        let v_breaks = distinct_domain_knots(refined.knots_v.as_slice(), domain_v);
        let u_offsets = span_offsets(&refined.knots_u, refined.degree_u, &u_breaks);
        let v_offsets = span_offsets(&refined.knots_v, refined.degree_v, &v_breaks);
        let mut spans = Vec::new();
        for (v_index, v_window) in v_breaks.windows(2).enumerate() {
            for (u_index, u_window) in u_breaks.windows(2).enumerate() {
                let mut points =
                    Vec::with_capacity((refined.degree_u.get() + 1) * (refined.degree_v.get() + 1));
                for local_v in 0..=refined.degree_v.get() {
                    for local_u in 0..=refined.degree_u.get() {
                        points.push(
                            refined
                                .control_points
                                .get(u_offsets[u_index] + local_u, v_offsets[v_index] + local_v),
                        );
                    }
                }
                let surface = NurbsSurface::new(
                    refined.degree_u,
                    refined.degree_v,
                    ControlNet::new(
                        points,
                        refined.degree_u.get() + 1,
                        refined.degree_v.get() + 1,
                    )?,
                    bezier_knots(refined.degree_u, u_window[0], u_window[1])?,
                    bezier_knots(refined.degree_v, v_window[0], v_window[1])?,
                )?;
                spans.push(BezierSurface { surface });
            }
        }
        Ok(spans)
    }

    /// The surface that passes through every one of `sections`, of degree
    /// `degree_v` across them.
    ///
    /// Skinning (Piegl & Tiller §10.3) reads the sections as the rows of one
    /// control grid and interpolates down each column of it, so
    /// `S(u, v_k) = C_k(u)` for every `k` — the surface reproduces every
    /// section, not only the two ends, and is `C^(degree_v - 1)` across the
    /// intermediate ones. `degree_v = 1` gives a surface linear in `v` between
    /// consecutive sections, which is the multi-section ruled surface; nothing
    /// else changes, so the two are one construction.
    ///
    /// The sections must already be compatible — one degree, one domain, one
    /// knot vector — which [`make_compatible`] establishes. A grid is
    /// rectangular or it is not a grid, so a mismatch is named rather than
    /// papered over.
    ///
    /// The interpolation runs on the sections' **homogeneous** control points,
    /// so a rational section is reproduced exactly rather than approximated by
    /// a surface through its cartesian control points.
    pub fn skinned(sections: &[NurbsCurve], degree_v: Degree) -> Result<Self, NurbsError> {
        if sections.len() < 2 {
            return Err(NurbsError::InsufficientSkinningSections {
                minimum: 2,
                got: sections.len(),
            });
        }
        if degree_v.get() >= sections.len() {
            return Err(NurbsError::SkinningDegreeTooHigh {
                degree: degree_v.get(),
                sections: sections.len(),
            });
        }

        let first = &sections[0];
        let nu = first.control_points().len();
        for (index, section) in sections.iter().enumerate().skip(1) {
            let reason = if section.degree() != first.degree() {
                Some(SkinningIncompatibility::Degree)
            } else if section.control_points().len() != nu {
                Some(SkinningIncompatibility::ControlPointCount)
            } else if !knot_vectors_agree(section.knots(), first.knots()) {
                Some(SkinningIncompatibility::KnotVector)
            } else {
                None
            };
            if let Some(reason) = reason {
                return Err(NurbsError::IncompatibleSkinningSection { index, reason });
            }
        }

        let parameters = Self::skinning_parameters(sections)?;
        let knots_v = KnotVector::averaged(&parameters, degree_v)?;
        let system = InterpolationSystem::new(&parameters, degree_v, &knots_v)?;

        let rows = (0..nu)
            .map(|index| {
                let column = sections
                    .iter()
                    .map(|section| *section.control_points().get(index).expect("index < nu"))
                    .collect::<Vec<_>>();
                system.solve(&column)
            })
            .collect::<Result<Vec<_>, _>>()?;

        let mut points = Vec::with_capacity(nu * sections.len());
        for j in 0..sections.len() {
            for row in &rows {
                points.push(*row.get(j).expect("one control point per section"));
            }
        }

        Self::new(
            first.degree(),
            degree_v,
            ControlNet::new(points, nu, sections.len())?,
            first.knots().clone(),
            knots_v,
        )
    }

    /// The `v` at which a skin through `sections` reproduces each of them.
    ///
    /// Piegl & Tiller eq. 10.8: chord length along each row of the control
    /// grid, normalized per row and then averaged over all rows. Averaging is
    /// what stops one wild row skewing the parameterization of the whole
    /// surface; a row of zero extent — every section sharing that control
    /// point — has no chord length to contribute and is left out of the mean
    /// rather than counted as zero.
    ///
    /// [`Self::skinned`] takes these as its interpolation parameters, so
    /// `S(u, parameters[k])` is section `k`. The sections must already be
    /// compatible, which is what makes one control point index name one row.
    pub fn skinning_parameters(sections: &[NurbsCurve]) -> Result<Vec<f64>, NurbsError> {
        let count = sections.len();
        if count < 2 {
            return Err(NurbsError::InsufficientSkinningSections {
                minimum: 2,
                got: count,
            });
        }
        let nu = sections[0].control_points().len();
        let mut totals = vec![0.0; nu];
        let mut cumulative = vec![vec![0.0; count]; nu];
        for (index, row) in cumulative.iter_mut().enumerate() {
            let mut accumulated = 0.0;
            for k in 1..count {
                let at = |section: &NurbsCurve| {
                    section
                        .control_points()
                        .get(index)
                        .map(|point| point.to_cartesian())
                };
                let (Some(here), Some(before)) = (at(&sections[k]), at(&sections[k - 1])) else {
                    return Err(NurbsError::IncompatibleSkinningSection {
                        index: k,
                        reason: SkinningIncompatibility::ControlPointCount,
                    });
                };
                accumulated += (here - before).norm();
                row[k] = accumulated;
            }
            totals[index] = accumulated;
        }

        let contributing = (0..nu)
            .filter(|&index| totals[index] > LINEAR_TOLERANCE)
            .collect::<Vec<_>>();
        if contributing.is_empty() {
            return Err(NurbsError::DegenerateInterpolationSamples);
        }

        let mut parameters = vec![0.0; count];
        for (k, parameter) in parameters.iter_mut().enumerate().take(count - 1).skip(1) {
            *parameter = contributing
                .iter()
                .map(|&index| cumulative[index][k] / totals[index])
                .sum::<f64>()
                / contributing.len() as f64;
        }
        parameters[count - 1] = 1.0;
        if parameters.windows(2).any(|pair| pair[1] <= pair[0]) {
            return Err(NurbsError::InvalidInterpolationParameters);
        }
        Ok(parameters)
    }

    /// The curve this surface traces at constant `u`, parameterized by `v`.
    ///
    /// Exact, not sampled: the isocurve's control points are the `u` basis
    /// functions applied to each column of the net, so the curve lies on the
    /// surface everywhere rather than at the points it was fitted through.
    /// That is what a rail of a skinned face needs — a rail derived any other
    /// way would not lie on both the faces that share it.
    pub fn isocurve_u(&self, u: f64) -> Result<NurbsCurve, NurbsError> {
        let domain = self.domain_u();
        let u = u.clamp(domain.start.value(), domain.end.value());
        let p = self.degree_u.get();
        let span = self
            .knots_u
            .find_span(self.control_points.nu() - 1, self.degree_u, u);
        let basis = basis_functions(span, u, self.degree_u, &self.knots_u);

        let points = (0..self.control_points.nv())
            .map(|j| {
                let mut accumulated = Vector4::zeros();
                for (i, weight) in basis.iter().copied().enumerate().take(p + 1) {
                    accumulated += self.control_points.get(span - p + i, j).0.coords * weight;
                }
                HPoint(Point4::from(accumulated))
            })
            .collect();

        NurbsCurve::new(
            self.degree_v,
            ControlPolygon::new(points)?,
            self.knots_v.clone(),
        )
    }

    /// The curve this surface traces at constant `v`, parameterized by `u`.
    ///
    /// The `v` twin of [`Self::isocurve_u`], and exact for the same reason.
    pub fn isocurve_v(&self, v: f64) -> Result<NurbsCurve, NurbsError> {
        let domain = self.domain_v();
        let v = v.clamp(domain.start.value(), domain.end.value());
        let q = self.degree_v.get();
        let span = self
            .knots_v
            .find_span(self.control_points.nv() - 1, self.degree_v, v);
        let basis = basis_functions(span, v, self.degree_v, &self.knots_v);

        let points = (0..self.control_points.nu())
            .map(|i| {
                let mut accumulated = Vector4::zeros();
                for (j, weight) in basis.iter().copied().enumerate().take(q + 1) {
                    accumulated += self.control_points.get(i, span - q + j).0.coords * weight;
                }
                HPoint(Point4::from(accumulated))
            })
            .collect();

        NurbsCurve::new(
            self.degree_u,
            ControlPolygon::new(points)?,
            self.knots_u.clone(),
        )
    }

    fn insert_knot_u(&mut self, knot: f64) -> Result<(), NurbsError> {
        let old_nu = self.control_points.nu();
        let nv = self.control_points.nv();
        let mut rows = Vec::with_capacity(nv);
        let mut knots = None;
        for v in 0..nv {
            let mut curve = NurbsCurve::new(
                self.degree_u,
                ControlPolygon::new((0..old_nu).map(|u| self.control_points.get(u, v)).collect())?,
                self.knots_u.clone(),
            )?;
            curve.insert_knot(knot);
            knots = Some(curve.knots().clone());
            rows.push(curve.control_points().as_slice().to_vec());
        }
        self.control_points =
            ControlNet::new(rows.into_iter().flatten().collect(), old_nu + 1, nv)?;
        self.knots_u = knots.expect("a valid control net has at least one row");
        Ok(())
    }

    /// Returns the same surface over the same domain, clamped in both
    /// directions.
    ///
    /// The knot vectors are clamped one direction at a time, for the reason
    /// [`NurbsCurve::clamped`] gives: an unclamped end leaves control points
    /// that influence only the surface outside its own domain, and anything
    /// bounding the surface by its control net then answers about a larger one.
    pub fn clamped(&self) -> Result<Self, NurbsError> {
        let mut clamped = self.clone();
        clamped.clamp_u()?;
        clamped.clamp_v()?;
        Ok(clamped)
    }

    fn clamp_u(&mut self) -> Result<(), NurbsError> {
        if self.knots_u.is_clamped(self.degree_u) {
            return Ok(());
        }
        let p = self.degree_u.get();
        let domain = self.domain_u();
        for end in [domain.start.value(), domain.end.value()] {
            while self.knots_u.multiplicity(end) < p + 1 {
                self.insert_knot_u(end)?;
            }
        }

        let (first, last) = clamped_span(self.knots_u.as_slice(), domain)?;
        let nv = self.control_points.nv();
        let kept = first..=last - p - 1;
        let points = (0..nv)
            .flat_map(|v| kept.clone().map(move |u| (u, v)))
            .map(|(u, v)| self.control_points.get(u, v))
            .collect();
        self.control_points = ControlNet::new(points, kept.count(), nv)?;
        self.knots_u = KnotVector::new(self.knots_u.as_slice()[first..=last].to_vec())?;
        Ok(())
    }

    fn clamp_v(&mut self) -> Result<(), NurbsError> {
        if self.knots_v.is_clamped(self.degree_v) {
            return Ok(());
        }
        let q = self.degree_v.get();
        let domain = self.domain_v();
        for end in [domain.start.value(), domain.end.value()] {
            while self.knots_v.multiplicity(end) < q + 1 {
                self.insert_knot_v(end)?;
            }
        }

        let (first, last) = clamped_span(self.knots_v.as_slice(), domain)?;
        let nu = self.control_points.nu();
        let kept = first..=last - q - 1;
        let points = kept
            .clone()
            .flat_map(|v| (0..nu).map(move |u| (u, v)))
            .map(|(u, v)| self.control_points.get(u, v))
            .collect();
        self.control_points = ControlNet::new(points, nu, kept.count())?;
        self.knots_v = KnotVector::new(self.knots_v.as_slice()[first..=last].to_vec())?;
        Ok(())
    }

    fn insert_knot_v(&mut self, knot: f64) -> Result<(), NurbsError> {
        let nu = self.control_points.nu();
        let old_nv = self.control_points.nv();
        let mut columns = Vec::with_capacity(nu);
        let mut knots = None;
        for u in 0..nu {
            let mut curve = NurbsCurve::new(
                self.degree_v,
                ControlPolygon::new((0..old_nv).map(|v| self.control_points.get(u, v)).collect())?,
                self.knots_v.clone(),
            )?;
            curve.insert_knot(knot);
            knots = Some(curve.knots().clone());
            columns.push(curve.control_points().as_slice().to_vec());
        }
        let mut points = Vec::with_capacity(nu * (old_nv + 1));
        for v in 0..=old_nv {
            for column in &columns {
                points.push(column[v]);
            }
        }
        self.control_points = ControlNet::new(points, nu, old_nv + 1)?;
        self.knots_v = knots.expect("a valid control net has at least one column");
        Ok(())
    }
}

fn bezier_knots(degree: Degree, start: f64, end: f64) -> Result<KnotVector, NurbsError> {
    let mut knots = vec![start; degree.get() + 1];
    knots.extend(std::iter::repeat_n(end, degree.get() + 1));
    KnotVector::new(knots)
}

fn distinct_interior_knots(knots: &[f64], domain: Interval) -> Vec<f64> {
    distinct_domain_knots(knots, domain)
        .into_iter()
        .filter(|knot| *knot > domain.start.value() && *knot < domain.end.value())
        .collect()
}

fn distinct_domain_knots(knots: &[f64], domain: Interval) -> Vec<f64> {
    let mut distinct = Vec::new();
    for &knot in knots {
        if knot < domain.start.value() || knot > domain.end.value() {
            continue;
        }
        if distinct.last().is_none_or(|last| *last != knot) {
            distinct.push(knot);
        }
    }
    distinct
}

fn span_offsets(knots: &KnotVector, degree: Degree, breaks: &[f64]) -> Vec<usize> {
    let mut offsets = Vec::with_capacity(breaks.len().saturating_sub(1));
    let mut offset = 0;
    for end in breaks.iter().copied().skip(1) {
        offsets.push(offset);
        offset += knots.multiplicity(end).min(degree.get() + 1);
    }
    offsets
}

/// The knot indices a clamped vector keeps, once both ends repeat enough.
///
/// Everything before the domain's first knot and after its last influences only
/// the surface outside its own domain, so the clamped vector is the run between
/// them.
fn clamped_span(knots: &[f64], domain: Interval) -> Result<(usize, usize), NurbsError> {
    let (Some(first), Some(last)) = (
        knots.iter().position(|&knot| knot == domain.start.value()),
        knots.iter().rposition(|&knot| knot == domain.end.value()),
    ) else {
        return Err(NurbsError::UnsortedKnots);
    };
    Ok((first, last))
}

/// Whether two knot vectors are the same vector, within [`KNOT_TOLERANCE`].
fn knot_vectors_agree(first: &KnotVector, second: &KnotVector) -> bool {
    first.len() == second.len()
        && first
            .as_slice()
            .iter()
            .zip(second.as_slice())
            .all(|(a, b)| (a - b).abs() <= KNOT_TOLERANCE)
}
