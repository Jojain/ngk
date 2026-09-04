use nalgebra::{Point2, Point4, UnitVector3, Vector3, Vector4};
use serde::{Deserialize, Serialize};

use super::curve::NurbsCurve;
use super::degree::Degree;
use super::knots::KnotVector;
use super::points::{ControlNet, ControlPolygon, HPoint};
use crate::geometry::nurbs::basis::{basis_function_derivatives, basis_functions};
use crate::geometry::nurbs::error::NurbsError;
use crate::geometry::{BBox, Interval, Point3};

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

    /// Returns a conservative control-hull bound for positive weights.
    pub fn bbox(&self) -> BBox {
        BBox::from_points(
            self.control_points()
                .as_slice()
                .iter()
                .map(|point| point.to_cartesian()),
        )
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
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

    pub fn point_at(&self, u: f64, v: f64) -> Point3 {
        let domain_u = self.domain_u();
        let domain_v = self.domain_v();
        let u = u.clamp(domain_u.start, domain_u.end);
        let v = v.clamp(domain_v.start, domain_v.end);

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

    pub fn closest_parameter(&self, point: Point3) -> Point2<f64> {
        let (mut u, mut v) = self.closest_sample_parameter(point, 12);
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
            u = (u - delta_u).clamp(domain_u.start, domain_u.end);
            v = (v - delta_v).clamp(domain_v.start, domain_v.end);

            if delta_u.hypot(delta_v) <= 1.0e-10 {
                break;
            }
        }

        Point2::new(u, v)
    }

    fn closest_sample_parameter(&self, point: Point3, sample_count: usize) -> (f64, f64) {
        let sample_count = sample_count.max(1);
        let domain_u = self.domain_u();
        let domain_v = self.domain_v();
        let mut closest = (domain_u.start, domain_v.start);
        let mut closest_distance = f64::INFINITY;

        for i in 0..=sample_count {
            let u = domain_u.start + domain_u.length() * (i as f64 / sample_count as f64);
            for j in 0..=sample_count {
                let v = domain_v.start + domain_v.length() * (j as f64 / sample_count as f64);
                let distance = (self.point_at(u, v) - point).norm_squared();
                if distance < closest_distance {
                    closest = (u, v);
                    closest_distance = distance;
                }
            }
        }

        closest
    }

    /// Returns `(dS/du, dS/dv)` in cartesian space.
    pub fn derivatives_uv(&self, u: f64, v: f64) -> (Vector3<f64>, Vector3<f64>) {
        let domain_u = self.domain_u();
        let domain_v = self.domain_v();
        let u = u.clamp(domain_u.start, domain_u.end);
        let v = v.clamp(domain_v.start, domain_v.end);

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
        .filter(|knot| *knot > domain.start && *knot < domain.end)
        .collect()
}

fn distinct_domain_knots(knots: &[f64], domain: Interval) -> Vec<f64> {
    let mut distinct = Vec::new();
    for &knot in knots {
        if knot < domain.start || knot > domain.end {
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
