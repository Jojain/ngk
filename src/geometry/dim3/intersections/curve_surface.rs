//! Curve/surface intersection by exact Bézier decomposition and bounded subdivision.
//!
//! Both operands are decomposed once into rational Bézier spans and patches
//! whose control hulls bound them. Candidate pairs are rejected by those hulls,
//! surviving pairs are subdivided until isolated, and only then is a Newton
//! correction run against the original NURBS equations. No sampled polyline or
//! triangulated surface grid takes part in finding candidates.

use nalgebra::{Matrix3, Vector3};

use super::error::IntersectionError;
use super::options::IntersectionOptions;
use super::{
    CurveSurfaceIntersection, CurveSurfaceIntersections, IntersectionCoverage,
    IntersectionIncompleteReason,
};
use crate::geometry::{
    Bezier, BezierSurface, Curve, Interval, NurbsCurve, NurbsSurface, Point3, PointCoincidence,
    Surface,
};

/// Combined curve/surface depth at which an isolated pair is corrected directly
/// instead of split further.
const EARLY_REFINEMENT_DEPTH: usize = 10;

/// Node visits allowed per query before the search reports itself incomplete.
///
/// Transverse candidates isolate within a few dozen nodes; only contacts this
/// solver cannot resolve approach the cap.
const SEARCH_NODE_BUDGET: usize = 4_096;

/// Samples used to decide whether a Bézier span rests on a surface.
///
/// The span and the surface are both polynomial, so their separation is smooth;
/// a span that agrees at this many parameters and nowhere departs between them
/// is treated as resting on the surface.
const OVERLAP_SAMPLE_COUNT: usize = 16;

/// A curve decomposed once for repeated intersection against many surfaces.
///
/// Building this is the expensive part of a curve/surface query, so callers
/// intersecting one curve against several surfaces should build it once.
#[derive(Debug, Clone)]
pub struct PreparedCurve {
    nurbs: NurbsCurve,
    spans: Vec<Bezier>,
}

impl PreparedCurve {
    /// Decomposes `curve` into its exact rational Bézier spans.
    pub fn new(curve: &Curve) -> Result<Self, IntersectionError> {
        let nurbs = curve.to_nurbs()?;
        let spans = nurbs.bezier_spans()?;
        Ok(Self { nurbs, spans })
    }

    /// Returns the parameter domain of the underlying curve.
    pub fn domain(&self) -> Interval {
        self.nurbs.domain()
    }

    fn has_positive_weights(&self) -> bool {
        self.nurbs
            .control_points()
            .as_slice()
            .iter()
            .all(|point| point.weight().is_finite() && point.weight() > 0.0)
    }
}

/// A surface decomposed once for repeated intersection against many curves.
#[derive(Clone)]
pub struct PreparedSurface {
    source: Surface,
    nurbs: NurbsSurface,
    patches: Vec<BezierSurface>,
}

impl PreparedSurface {
    /// Decomposes `surface` into its exact rational Bézier patches.
    pub fn new(surface: &Surface) -> Result<Self, IntersectionError> {
        Self::from_nurbs(surface.clone(), surface.to_nurbs()?)
    }

    /// Decomposes `surface` realized over the requested parameter box.
    ///
    /// Unbounded analytic surfaces are otherwise converted over an arbitrary
    /// unit patch, which silently drops every intersection outside it. Callers
    /// holding a trim domain should pass it here.
    pub fn over(
        surface: &Surface,
        domain_u: Interval,
        domain_v: Interval,
    ) -> Result<Self, IntersectionError> {
        Self::from_nurbs(surface.clone(), surface.to_nurbs_over(domain_u, domain_v)?)
    }

    fn from_nurbs(source: Surface, nurbs: NurbsSurface) -> Result<Self, IntersectionError> {
        let patches = nurbs.bezier_spans()?;
        Ok(Self {
            source,
            nurbs,
            patches,
        })
    }

    /// Returns the u parameter domain of the underlying surface.
    pub fn domain_u(&self) -> Interval {
        self.nurbs.domain_u()
    }

    /// Returns the v parameter domain of the underlying surface.
    pub fn domain_v(&self) -> Interval {
        self.nurbs.domain_v()
    }

    /// Returns the underlying NURBS surface.
    pub fn nurbs(&self) -> &NurbsSurface {
        &self.nurbs
    }

    /// Returns the source surface whose parameter space public results use.
    pub fn source(&self) -> &Surface {
        &self.source
    }
}

pub fn intersect_curve_surface(
    curve: &Curve,
    surface: &Surface,
) -> Result<CurveSurfaceIntersections, IntersectionError> {
    intersect_curve_surface_with_options(curve, surface, IntersectionOptions::default())
}

pub fn intersect_curve_surface_with_options(
    curve: &Curve,
    surface: &Surface,
    options: IntersectionOptions,
) -> Result<CurveSurfaceIntersections, IntersectionError> {
    if !options.validate() {
        return Err(IntersectionError::InvalidOptions);
    }
    let curve = PreparedCurve::new(curve)?;
    let surface = PreparedSurface::new(surface)?;
    intersect_prepared_curve_surface(&curve, &surface, options)
}

/// Intersects operands that were decomposed ahead of time.
pub fn intersect_prepared_curve_surface(
    curve: &PreparedCurve,
    surface: &PreparedSurface,
    options: IntersectionOptions,
) -> Result<CurveSurfaceIntersections, IntersectionError> {
    if !options.validate() {
        return Err(IntersectionError::InvalidOptions);
    }

    // Control hulls bound their geometry only under positive weights, and every
    // rejection below is a hull test. Without that the search cannot claim to
    // have visited the whole domain.
    if !curve.has_positive_weights()
        || !surface
            .patches
            .iter()
            .all(BezierSurface::has_positive_weights)
    {
        return Ok(CurveSurfaceIntersections::new(
            Vec::new(),
            IntersectionCoverage::Incomplete(vec![
                IntersectionIncompleteReason::UnsupportedControlPointWeights,
            ]),
        ));
    }

    let mut search = Search {
        curve: &curve.nurbs,
        surface: &surface.nurbs,
        options,
        points: Vec::new(),
        overlaps: Vec::new(),
        reasons: Vec::new(),
        budget: SEARCH_NODE_BUDGET,
    };
    for span in &curve.spans {
        for patch in &surface.patches {
            search.visit(
                CurvePiece::new(span.clone()),
                SurfacePiece::new(patch.clone()),
            );
        }
    }
    Ok(search.finish())
}

#[derive(Clone)]
struct CurvePiece {
    bezier: Bezier,
    depth: usize,
}

impl CurvePiece {
    fn new(bezier: Bezier) -> Self {
        Self { bezier, depth: 0 }
    }

    fn diagonal_length(&self) -> f64 {
        self.bezier.bbox().diagonal_length()
    }

    fn split(&self) -> Option<(Self, Self)> {
        let domain = self.bezier.domain();
        let midpoint = 0.5 * (domain.start + domain.end);
        let (left, right) = self.bezier.subdivide(midpoint).ok()?;
        Some((
            Self {
                bezier: left,
                depth: self.depth + 1,
            },
            Self {
                bezier: right,
                depth: self.depth + 1,
            },
        ))
    }
}

#[derive(Clone)]
struct SurfacePiece {
    patch: BezierSurface,
    depth: usize,
}

impl SurfacePiece {
    fn new(patch: BezierSurface) -> Self {
        Self { patch, depth: 0 }
    }

    fn diagonal_length(&self) -> f64 {
        self.patch.bbox().diagonal_length()
    }

    /// Splits along whichever parameter direction spans more model space.
    fn split(&self) -> Option<(Self, Self)> {
        let domain_u = self.patch.domain_u();
        let domain_v = self.patch.domain_v();
        let extent_u = (self.patch.point_at(domain_u.end, domain_v.start)
            - self.patch.point_at(domain_u.start, domain_v.start))
        .norm();
        let extent_v = (self.patch.point_at(domain_u.start, domain_v.end)
            - self.patch.point_at(domain_u.start, domain_v.start))
        .norm();
        let (left, right) = if extent_u >= extent_v {
            self.patch
                .subdivide_u(0.5 * (domain_u.start + domain_u.end))
                .ok()?
        } else {
            self.patch
                .subdivide_v(0.5 * (domain_v.start + domain_v.end))
                .ok()?
        };
        Some((
            Self {
                patch: left,
                depth: self.depth + 1,
            },
            Self {
                patch: right,
                depth: self.depth + 1,
            },
        ))
    }
}

struct Search<'a> {
    curve: &'a NurbsCurve,
    surface: &'a NurbsSurface,
    options: IntersectionOptions,
    points: Vec<CurveSurfaceIntersection>,
    overlaps: Vec<Interval>,
    reasons: Vec<IntersectionIncompleteReason>,
    /// Remaining node visits for this query.
    ///
    /// Transverse candidates isolate in a handful of splits, but a tangential
    /// contact keeps both hulls overlapping however far they are split, so an
    /// unbounded search does not terminate in useful time. Running out is
    /// reported rather than passed off as an empty result.
    budget: usize,
}

impl Search<'_> {
    fn visit(&mut self, curve: CurvePiece, surface: SurfacePiece) {
        let options = self.options;
        let Some(remaining) = self.budget.checked_sub(1) else {
            self.push_reason(IntersectionIncompleteReason::SubdivisionBudgetExhausted);
            return;
        };
        self.budget = remaining;

        let curve_bbox = curve.bezier.bbox();
        let surface_bbox = surface.patch.bbox();
        if !curve_bbox
            .expanded(options.bbox_tolerance)
            .intersects(&surface_bbox, options.bbox_tolerance)
        {
            return;
        }

        if let Some(plane) = patch_plane(&surface.patch, options.linear_tolerance) {
            // A rational Bézier point is a convex combination of its control
            // points and distance to a plane is affine, so a span whose hull
            // lies in the plane lies in it entirely. This certifies the overlap
            // without sampling; the caller still trims it to the patch.
            if span_lies_in_plane(&curve.bezier, plane, options.linear_tolerance) {
                self.overlaps.push(curve.bezier.domain());
                return;
            }
        }

        // A curve resting on a curved surface keeps both hulls overlapping
        // however far the pair is split, so subdivision alone never settles it.
        // The untouched span is tested against the surface directly instead.
        if curve.depth == 0 && self.span_lies_on_surface(&curve.bezier) {
            self.overlaps.push(curve.bezier.domain());
            return;
        }

        let curve_leaf = curve.diagonal_length() <= options.leaf_diagonal_tolerance;
        let surface_leaf = surface.diagonal_length() <= options.leaf_diagonal_tolerance;
        // The cap is on total splits: bounding each side separately admits the
        // product of both budgets, which no tangential contact ever finishes.
        let exhausted = curve.depth + surface.depth >= options.max_subdivision_depth;

        if (curve_leaf && surface_leaf) || exhausted {
            // Hulls that still overlap without padding, around a candidate that
            // will not correct to tolerance, mean a tangential or singular
            // contact this solver does not resolve.
            if !self.refine(&curve, &surface) && curve_bbox.intersects(&surface_bbox, 0.0) {
                self.report_unresolved(&curve);
            }
            return;
        }

        // Once the candidate is small enough to hold a single root, correcting
        // is both cheaper and more accurate than subdividing to tolerance.
        if curve.depth + surface.depth >= EARLY_REFINEMENT_DEPTH && self.refine(&curve, &surface) {
            return;
        }

        let split_curve =
            !curve_leaf && (surface_leaf || curve.diagonal_length() >= surface.diagonal_length());
        if split_curve && let Some((left, right)) = curve.split() {
            self.visit(left, surface.clone());
            self.visit(right, surface);
            return;
        }

        if !surface_leaf && let Some((left, right)) = surface.split() {
            self.visit(curve.clone(), left);
            self.visit(curve, right);
        } else if !self.refine(&curve, &surface) && curve_bbox.intersects(&surface_bbox, 0.0) {
            self.report_unresolved(&curve);
        }
    }

    /// Records an abandoned candidate as an overlap when the span rests on the
    /// surface, and as an unresolved tangency otherwise.
    ///
    /// A span that only *partly* rests on the surface is abandoned in pieces,
    /// and `finish` merges those pieces back into one interval.
    fn report_unresolved(&mut self, curve: &CurvePiece) {
        if self.span_lies_on_surface(&curve.bezier) {
            self.overlaps.push(curve.bezier.domain());
            return;
        }
        self.push_reason(IntersectionIncompleteReason::TangentOrSingularContact);
    }

    /// Whether every sample of `span` projects onto the surface within tolerance.
    ///
    /// The projection is clamped to the surface's own parameter domain, so a
    /// span running alongside the surface but past its trim is not an overlap.
    fn span_lies_on_surface(&self, span: &Bezier) -> bool {
        let domain = span.domain();
        (0..=OVERLAP_SAMPLE_COUNT).all(|index| {
            let fraction = index as f64 / OVERLAP_SAMPLE_COUNT as f64;
            let point = span.point_at(domain.start + (domain.end - domain.start) * fraction);
            let uv = self.surface.closest_parameter(point);
            (self.surface.point_at(uv.x, uv.y) - point).norm() <= self.options.residual_tolerance
        })
    }

    /// Corrects one isolated candidate against the original NURBS equations.
    fn refine(&mut self, curve: &CurvePiece, surface: &SurfacePiece) -> bool {
        let options = self.options;
        let curve_domain = curve.bezier.domain();
        let surface_domain_u = surface.patch.domain_u();
        let surface_domain_v = surface.patch.domain_v();
        let mut curve_u = 0.5 * (curve_domain.start + curve_domain.end);
        let mut surface_u = 0.5 * (surface_domain_u.start + surface_domain_u.end);
        let mut surface_v = 0.5 * (surface_domain_v.start + surface_domain_v.end);

        for _ in 0..options.newton_max_iterations {
            let curve_point = self.curve.point_at(curve_u);
            let surface_point = self.surface.point_at(surface_u, surface_v);
            let residual = curve_point - surface_point;
            let curve_derivative = self.curve.derivative_at(curve_u, 1);
            let (surface_du, surface_dv) = self.surface.derivatives_uv(surface_u, surface_v);
            let jacobian = Matrix3::from_columns(&[curve_derivative, -surface_du, -surface_dv]);
            let Some(delta) = jacobian.lu().solve(&(-residual)) else {
                break;
            };

            curve_u = clamp_interval(curve_u + delta.x, curve_domain);
            surface_u = clamp_interval(surface_u + delta.y, surface_domain_u);
            surface_v = clamp_interval(surface_v + delta.z, surface_domain_v);
            if delta.norm() <= options.parameter_tolerance {
                break;
            }
        }

        let curve_point = self.curve.point_at(curve_u);
        let surface_point = self.surface.point_at(surface_u, surface_v);
        if (curve_point - surface_point).norm_squared() > options.linear_tolerance_squared() {
            return false;
        }

        let point = Point3::from((curve_point.coords + surface_point.coords) * 0.5);
        if self.points.iter().any(|existing| {
            matches!(
                existing,
                CurveSurfaceIntersection::Point { point: existing, .. }
                    if existing.coincides(point, point_merge_tolerance(options))
            )
        }) {
            return true;
        }
        self.points.push(CurveSurfaceIntersection::Point {
            point,
            curve_u,
            surface_u,
            surface_v,
        });
        true
    }

    fn push_reason(&mut self, reason: IntersectionIncompleteReason) {
        if !self.reasons.contains(&reason) {
            self.reasons.push(reason);
        }
    }

    fn finish(mut self) -> CurveSurfaceIntersections {
        let options = self.options;
        let overlaps = merge_intervals(&mut self.overlaps, options);
        let mut intersections = Vec::with_capacity(self.points.len() + overlaps.len());
        // A point sitting inside a certified overlap is that overlap, not a
        // separate transverse contact.
        intersections.extend(self.points.into_iter().filter(|point| {
            let CurveSurfaceIntersection::Point { curve_u, .. } = point else {
                return true;
            };
            !overlaps
                .iter()
                .any(|interval| interval.contains(*curve_u, options.parameter_tolerance))
        }));
        intersections.extend(
            overlaps
                .into_iter()
                .map(|curve_interval| CurveSurfaceIntersection::Overlap { curve_interval }),
        );

        let coverage = if self.reasons.is_empty() {
            IntersectionCoverage::Complete
        } else {
            IntersectionCoverage::Incomplete(self.reasons)
        };
        CurveSurfaceIntersections::new(intersections, coverage)
    }
}

/// A plane through a patch's control hull, when every control point lies on it.
#[derive(Clone, Copy)]
struct PatchPlane {
    origin: Point3,
    normal: Vector3<f64>,
}

/// Returns the patch's plane when its control net is planar within `tolerance`.
fn patch_plane(patch: &BezierSurface, tolerance: f64) -> Option<PatchPlane> {
    let points = patch.control_points().as_slice();
    let origin = points.first()?.to_cartesian();
    let mut normal = None;
    for i in 1..points.len() {
        for j in (i + 1)..points.len() {
            let candidate =
                (points[i].to_cartesian() - origin).cross(&(points[j].to_cartesian() - origin));
            if candidate.norm() > tolerance {
                normal = Some(candidate.normalize());
                break;
            }
        }
        if normal.is_some() {
            break;
        }
    }
    let normal = normal?;
    points
        .iter()
        .all(|point| (point.to_cartesian() - origin).dot(&normal).abs() <= tolerance)
        .then_some(PatchPlane { origin, normal })
}

/// Returns whether a whole Bézier span lies in `plane`, by its control hull.
fn span_lies_in_plane(span: &Bezier, plane: PatchPlane, tolerance: f64) -> bool {
    span.control_points().iter().all(|point| {
        (point.to_cartesian() - plane.origin)
            .dot(&plane.normal)
            .abs()
            <= tolerance
    })
}

/// Merges touching or overlapping curve intervals into maximal runs.
fn merge_intervals(intervals: &mut [Interval], options: IntersectionOptions) -> Vec<Interval> {
    if intervals.is_empty() {
        return Vec::new();
    }
    intervals.sort_by(|a, b| a.ordered().start.total_cmp(&b.ordered().start));
    let mut merged: Vec<Interval> = Vec::new();
    for interval in intervals.iter().map(|interval| interval.ordered()) {
        match merged.last_mut() {
            Some(last) if interval.start <= last.end + options.parameter_tolerance => {
                last.end = last.end.max(interval.end);
            }
            _ => merged.push(interval),
        }
    }
    merged
}

fn point_merge_tolerance(options: IntersectionOptions) -> f64 {
    (options.linear_tolerance.sqrt() * 10.0).max(options.linear_tolerance)
}

fn clamp_interval(value: f64, interval: Interval) -> f64 {
    value.clamp(interval.start, interval.end)
}
