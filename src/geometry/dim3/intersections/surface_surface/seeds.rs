use nalgebra::{Vector3, Vector4};

use super::super::curve_surface::{
    PreparedCurve, PreparedSurface, intersect_prepared_curve_surface,
};
use super::super::{
    CurveSurfaceIntersection, IntersectionCoverage, IntersectionError,
    IntersectionIncompleteReason, IntersectionOptions,
};
use super::normals::NormalCone;
use super::tracer::TraceState;
use crate::geometry::{
    BBox, BezierSurface, ControlNet, ControlPolygon, Curve, Interval, NurbsCurve, NurbsSurface,
    Point2, Point3, Surface,
};

const PLANAR_SEARCH_NODE_BUDGET: usize = 4_096;
const PAIR_SEARCH_NODE_BUDGET: usize = 4_096;

#[derive(Clone, Copy)]
enum Boundary {
    UMin,
    UMax,
    VMin,
    VMax,
}

pub(super) struct SeedSearch {
    pub seeds: Vec<TraceState>,
    pub overlap_boundary_found: bool,
    /// Reasons the shared curve/surface solver could not certify a boundary
    /// search. Surface/surface coverage cannot exceed the coverage of the
    /// boundary searches it is built from.
    pub incomplete_reasons: Vec<IntersectionIncompleteReason>,
}

#[derive(Clone, Copy)]
struct PlaneEquation {
    origin: Point3,
    normal: Vector3<f64>,
}

/// The parameter direction in which a patch's distance numerator is monotone,
/// so the contour cannot close into a loop inside that patch.
#[derive(Clone, Copy, PartialEq)]
enum MonotoneDirection {
    U,
    V,
}

/// One boundary edge of a Bézier patch, named by the parameter it holds fixed.
#[derive(Clone, Copy)]
struct PatchEdge {
    /// Whether the edge varies in u and holds v fixed.
    varying_u: bool,
    /// Whether the fixed parameter is the end of its domain.
    at_end: bool,
}

impl PatchEdge {
    const ALL: [Self; 4] = [
        Self {
            varying_u: true,
            at_end: false,
        },
        Self {
            varying_u: true,
            at_end: true,
        },
        Self {
            varying_u: false,
            at_end: false,
        },
        Self {
            varying_u: false,
            at_end: true,
        },
    ];

    /// Returns the value of the parameter this edge holds fixed.
    fn fixed(self, patch: &BezierSurface) -> f64 {
        let domain = if self.varying_u {
            patch.domain_v()
        } else {
            patch.domain_u()
        };
        if self.at_end {
            domain.end
        } else {
            domain.start
        }
    }

    /// Returns the domain of the parameter this edge varies.
    fn varying(self, patch: &BezierSurface) -> Interval {
        if self.varying_u {
            patch.domain_u()
        } else {
            patch.domain_v()
        }
    }

    /// Builds the patch parameters of a point on this edge.
    fn parameters(self, patch: &BezierSurface, varying: f64) -> Point2 {
        let fixed = self.fixed(patch);
        if self.varying_u {
            Point2::new(varying, fixed)
        } else {
            Point2::new(fixed, varying)
        }
    }

    /// Returns whether a control index lies on this edge.
    fn holds_index(self, net: &ControlNet, u: usize, v: usize) -> bool {
        if self.varying_u {
            v == if self.at_end { net.nv() - 1 } else { 0 }
        } else {
            u == if self.at_end { net.nu() - 1 } else { 0 }
        }
    }

    /// Returns this edge's control coefficients in index order.
    fn coefficients(self, patch: &BezierSurface, values: &[f64]) -> Vec<f64> {
        let net = patch.control_points();
        indexed_coefficients(net, values)
            .filter(|(u, v, _)| self.holds_index(net, *u, *v))
            .map(|(_, _, value)| value)
            .collect()
    }

    /// Returns the control coefficients that are not on this edge.
    fn complement(self, patch: &BezierSurface, values: &[f64]) -> Vec<f64> {
        let net = patch.control_points();
        indexed_coefficients(net, values)
            .filter(|(u, v, _)| !self.holds_index(net, *u, *v))
            .map(|(_, _, value)| value)
            .collect()
    }

    /// Returns the two patch corners bounding this edge.
    fn corners(self, patch: &BezierSurface) -> [Point2; 2] {
        let varying = self.varying(patch);
        [varying.start, varying.end].map(|value| self.parameters(patch, value))
    }

    /// Returns whether this edge is a level set of the monotone direction.
    ///
    /// Monotonicity already bounds an edge running along that direction to one
    /// crossing, so only the level-set edges still need checking.
    fn is_level_set_of(self, direction: MonotoneDirection) -> bool {
        match direction {
            MonotoneDirection::U => !self.varying_u,
            MonotoneDirection::V => self.varying_u,
        }
    }
}

/// Finds and certifies every regular contour when either operand is planar.
///
/// Signed distances from a positive-weight rational Bézier patch to a plane are
/// bounded by the signed distances of its control points, so a sign-definite
/// patch is discarded outright. A surviving patch is accounted for by exactly
/// one regular arc when either Bernstein argument holds:
///
/// - the distance numerators vanish on one patch edge and are strictly sign
///   definite on every other control point, so no Bernstein combination can
///   vanish off that edge and the contour *is* the edge; or
/// - the numerators are monotone in one parameter direction, so the zero set is
///   connected along that direction for every value of the other one and cannot
///   close into an interior loop, and every edge that is a level set of that
///   direction is itself sign definite or monotone, so it contributes at most
///   one crossing.
///
/// A patch that touches the plane without crossing it carries a tangency rather
/// than a regular arc; that is reported instead of subdivided indefinitely.
pub(super) fn planar_seeds(
    a: &NurbsSurface,
    b: &NurbsSurface,
    options: IntersectionOptions,
) -> Result<Option<SeedSearch>, IntersectionError> {
    if let Some(plane) = plane_equation(a, options.linear_tolerance) {
        return PlanarSeedSearch::new(a, b, true, plane, options)
            .run()
            .map(Some);
    }
    if let Some(plane) = plane_equation(b, options.linear_tolerance) {
        return PlanarSeedSearch::new(b, a, false, plane, options)
            .run()
            .map(Some);
    }
    Ok(None)
}

struct PlanarSeedSearch<'a> {
    planar: &'a NurbsSurface,
    other: &'a NurbsSurface,
    planar_is_a: bool,
    plane: PlaneEquation,
    planar_bbox: BBox,
    options: IntersectionOptions,
    search: SeedSearch,
    budget: usize,
}

impl<'a> PlanarSeedSearch<'a> {
    fn new(
        planar: &'a NurbsSurface,
        other: &'a NurbsSurface,
        planar_is_a: bool,
        plane: PlaneEquation,
        options: IntersectionOptions,
    ) -> Self {
        Self {
            planar,
            other,
            planar_is_a,
            plane,
            planar_bbox: control_bbox(planar),
            options,
            search: SeedSearch {
                seeds: Vec::new(),
                overlap_boundary_found: false,
                incomplete_reasons: Vec::new(),
            },
            budget: PLANAR_SEARCH_NODE_BUDGET,
        }
    }

    fn run(mut self) -> Result<SeedSearch, IntersectionError> {
        for patch in self.other.bezier_spans()? {
            self.visit(patch, 0)?;
        }
        dedup_seeds(&mut self.search.seeds, self.options);
        Ok(self.search)
    }

    fn visit(&mut self, patch: BezierSurface, depth: usize) -> Result<(), IntersectionError> {
        let Some(remaining) = self.budget.checked_sub(1) else {
            self.report(IntersectionIncompleteReason::SubdivisionBudgetExhausted);
            return Ok(());
        };
        self.budget = remaining;

        if !patch
            .bbox()
            .expanded(self.options.bbox_tolerance)
            .intersects(&self.planar_bbox, self.options.bbox_tolerance)
        {
            return Ok(());
        }

        let tolerance = self.options.linear_tolerance;
        let distances = signed_control_distances(&patch, self.plane);
        let minimum = distances.iter().copied().fold(f64::INFINITY, f64::min);
        let maximum = distances.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        if minimum > tolerance || maximum < -tolerance {
            return Ok(());
        }
        if minimum >= -tolerance && maximum <= tolerance {
            self.report(IntersectionIncompleteReason::CoincidentRegionResolutionNotImplemented);
            return Ok(());
        }

        let numerators = signed_control_numerators(&patch, &distances);
        if let Some(seeds) = self
            .edge_contour_seeds(&patch, &numerators)
            .or_else(|| self.monotone_contour_seeds(&patch, &numerators))
        {
            self.search.seeds.extend(seeds);
            return Ok(());
        }

        // The control hull already proves the patch stays on one side of the
        // plane, so a contact here cannot be a regular crossing.
        if (minimum >= -tolerance || maximum <= tolerance)
            && self.touches_tangentially(&patch, &distances)
        {
            self.report(IntersectionIncompleteReason::TangentOrSingularContact);
            return Ok(());
        }

        if depth >= self.options.max_subdivision_depth {
            self.report(IntersectionIncompleteReason::SubdivisionBudgetExhausted);
            return Ok(());
        }

        let u = patch.domain_u();
        let v = patch.domain_v();
        let (left, right) = patch.subdivide_u(0.5 * (u.start + u.end))?;
        let (lower_left, upper_left) = left.subdivide_v(0.5 * (v.start + v.end))?;
        let (lower_right, upper_right) = right.subdivide_v(0.5 * (v.start + v.end))?;
        for child in [lower_left, upper_left, lower_right, upper_right] {
            self.visit(child, depth + 1)?;
        }
        Ok(())
    }

    /// Seeds a patch whose contour is exactly one of its own edges.
    fn edge_contour_seeds(
        &self,
        patch: &BezierSurface,
        numerators: &[f64],
    ) -> Option<Vec<TraceState>> {
        let tolerance = self.options.linear_tolerance;
        for edge in PatchEdge::ALL {
            if !edge
                .coefficients(patch, numerators)
                .iter()
                .all(|value| value.abs() <= tolerance)
            {
                continue;
            }
            if !strictly_same_sign(edge.complement(patch, numerators).into_iter(), tolerance) {
                continue;
            }
            let seeds = self.seeds_at(patch, edge.corners(patch));
            if seeds.len() == 2 {
                return Some(seeds);
            }
        }
        None
    }

    /// Seeds a patch crossed by a single monotone contour arc.
    fn monotone_contour_seeds(
        &self,
        patch: &BezierSurface,
        numerators: &[f64],
    ) -> Option<Vec<TraceState>> {
        let tolerance = self.options.linear_tolerance;
        let direction = monotone_direction(patch, numerators, tolerance)?;
        let mut roots = Vec::new();
        for edge in PatchEdge::ALL {
            if edge.is_level_set_of(direction)
                && !edge_has_simple_root(patch, numerators, edge, tolerance)
            {
                return None;
            }
            roots.extend(isolate_edge_root(patch, self.plane, edge, self.options));
        }
        let seeds = self.seeds_at(patch, roots);
        (seeds.len() == 2).then_some(seeds)
    }

    /// Confirms that a one-sided patch really meets the plane, and tangentially.
    ///
    /// Only control points already within tolerance of the plane are examined,
    /// and the patch normal at the matching Greville parameters must be
    /// parallel to the plane normal. A patch that merely *could* touch by its
    /// control hull is subdivided instead.
    fn touches_tangentially(&self, patch: &BezierSurface, distances: &[f64]) -> bool {
        let net = patch.control_points();
        indexed_coefficients(net, distances)
            .filter(|(_, _, distance)| distance.abs() <= self.options.linear_tolerance)
            .any(|(u, v, _)| {
                let uv = Point2::new(
                    greville(patch.domain_u(), u, net.nu()),
                    greville(patch.domain_v(), v, net.nv()),
                );
                patch.normal_at(uv.x, uv.y).cross(&self.plane.normal).norm()
                    <= self.options.angular_tolerance
            })
    }

    /// Realizes patch parameters as trace seeds shared by both surfaces.
    fn seeds_at(
        &self,
        patch: &BezierSurface,
        parameters: impl IntoIterator<Item = Point2>,
    ) -> Vec<TraceState> {
        let mut seeds: Vec<TraceState> = Vec::new();
        for other_uv in parameters {
            let point = patch.point_at(other_uv.x, other_uv.y);
            let plane_uv = self.planar.closest_parameter(point);
            if !self
                .planar
                .domain_u()
                .contains(plane_uv.x, self.options.parameter_tolerance)
                || !self
                    .planar
                    .domain_v()
                    .contains(plane_uv.y, self.options.parameter_tolerance)
                || (self.planar.point_at(plane_uv.x, plane_uv.y) - point).norm()
                    > self.options.residual_tolerance
            {
                continue;
            }
            let parameters = if self.planar_is_a {
                Vector4::new(plane_uv.x, plane_uv.y, other_uv.x, other_uv.y)
            } else {
                Vector4::new(other_uv.x, other_uv.y, plane_uv.x, plane_uv.y)
            };
            let state = if self.planar_is_a {
                TraceState::new(self.planar, self.other, parameters)
            } else {
                TraceState::new(self.other, self.planar, parameters)
            };
            if !seeds.iter().any(|existing| {
                (existing.point - state.point).norm() <= self.options.linear_tolerance
            }) {
                seeds.push(state);
            }
        }
        seeds
    }

    fn report(&mut self, reason: IntersectionIncompleteReason) {
        push_reason(&mut self.search.incomplete_reasons, reason);
    }
}

fn control_bbox(surface: &NurbsSurface) -> BBox {
    BBox::from_points(
        surface
            .control_points()
            .as_slice()
            .iter()
            .map(|point| point.to_cartesian()),
    )
}

fn plane_equation(surface: &NurbsSurface, tolerance: f64) -> Option<PlaneEquation> {
    let points = surface.control_points().as_slice();
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
        .then_some(PlaneEquation { origin, normal })
}

/// Finds regular branch seeds for a pair of surfaces with no planar operand.
///
/// Subdivision isolates parameter boxes whose two normal cones are disjoint. By
/// the Sederberg-Meyers loop criterion such a box cannot contain a closed
/// intersection loop, so every branch crossing it also crosses its boundary and
/// seeding from the patch boundary curves accounts for all of them. A box that
/// exhausts the depth or node budget without that certificate is reported, so
/// the caller never mistakes a missed interior loop for an empty one.
pub(super) fn pair_seeds(
    a: &NurbsSurface,
    b: &NurbsSurface,
    options: IntersectionOptions,
) -> Result<SeedSearch, IntersectionError> {
    let mut search = PairSeedSearch {
        options,
        search: SeedSearch {
            seeds: Vec::new(),
            overlap_boundary_found: false,
            incomplete_reasons: Vec::new(),
        },
        budget: PAIR_SEARCH_NODE_BUDGET,
    };
    for first in a.bezier_spans()? {
        for second in b.bezier_spans()? {
            search.visit(&first, &second, 0)?;
        }
    }
    dedup_seeds(&mut search.search.seeds, options);
    Ok(search.search)
}

struct PairSeedSearch {
    options: IntersectionOptions,
    search: SeedSearch,
    budget: usize,
}

impl PairSeedSearch {
    fn visit(
        &mut self,
        a: &BezierSurface,
        b: &BezierSurface,
        depth: usize,
    ) -> Result<(), IntersectionError> {
        let Some(remaining) = self.budget.checked_sub(1) else {
            self.report(IntersectionIncompleteReason::SubdivisionBudgetExhausted);
            return Ok(());
        };
        self.budget = remaining;

        let tolerance = self.options.bbox_tolerance;
        if !a
            .bbox()
            .expanded(tolerance)
            .intersects(&b.bbox(), tolerance)
        {
            return Ok(());
        }

        let cones = (
            NormalCone::from_patch(a, self.options.linear_tolerance),
            NormalCone::from_patch(b, self.options.linear_tolerance),
        );
        if let (Some(first), Some(second)) = cones
            && first.is_disjoint_from(second, self.options.angular_tolerance)
        {
            collect_surface_boundaries(
                a.surface(),
                b.surface(),
                true,
                self.options,
                &mut self.search,
            )?;
            collect_surface_boundaries(
                b.surface(),
                a.surface(),
                false,
                self.options,
                &mut self.search,
            )?;
            return Ok(());
        }

        if depth >= self.options.max_subdivision_depth {
            self.report(IntersectionIncompleteReason::LoopFreedomNotCertified);
            return Ok(());
        }

        if split_first(a, b, cones) {
            for child in quarters(a)? {
                self.visit(&child, b, depth + 1)?;
            }
        } else {
            for child in quarters(b)? {
                self.visit(a, &child, depth + 1)?;
            }
        }
        Ok(())
    }

    fn report(&mut self, reason: IntersectionIncompleteReason) {
        push_reason(&mut self.search.incomplete_reasons, reason);
    }
}

/// Chooses the patch whose subdivision is most likely to separate the cones.
///
/// A patch with no cone at all is the blocker; otherwise the wider cone is.
fn split_first(
    a: &BezierSurface,
    b: &BezierSurface,
    cones: (Option<NormalCone>, Option<NormalCone>),
) -> bool {
    match cones {
        (None, Some(_)) => true,
        (Some(_), None) => false,
        (Some(first), Some(second)) => first.width() >= second.width(),
        (None, None) => a.bbox().diagonal_length() >= b.bbox().diagonal_length(),
    }
}

/// Splits a patch at the midpoint of both parameter directions.
fn quarters(patch: &BezierSurface) -> Result<[BezierSurface; 4], IntersectionError> {
    let u = patch.domain_u();
    let v = patch.domain_v();
    let (left, right) = patch.subdivide_u(0.5 * (u.start + u.end))?;
    let (lower_left, upper_left) = left.subdivide_v(0.5 * (v.start + v.end))?;
    let (lower_right, upper_right) = right.subdivide_v(0.5 * (v.start + v.end))?;
    Ok([lower_left, upper_left, lower_right, upper_right])
}

/// Iterates control coefficients together with their net indices.
fn indexed_coefficients<'a>(
    net: &'a ControlNet,
    values: &'a [f64],
) -> impl Iterator<Item = (usize, usize, f64)> + 'a {
    (0..net.nv()).flat_map(move |v| (0..net.nu()).map(move |u| (u, v, values[v * net.nu() + u])))
}

/// Returns the Greville parameter of a Bézier control index.
fn greville(domain: Interval, index: usize, count: usize) -> f64 {
    if count < 2 {
        return domain.start;
    }
    domain.start + (domain.end - domain.start) * index as f64 / (count - 1) as f64
}

fn signed_control_distances(patch: &BezierSurface, plane: PlaneEquation) -> Vec<f64> {
    patch
        .control_points()
        .as_slice()
        .iter()
        .map(|point| (point.to_cartesian() - plane.origin).dot(&plane.normal))
        .collect()
}

/// Returns the Bernstein coefficients of the rational distance numerator.
///
/// Weights are positive, so these keep the signs of the control distances while
/// being the coefficients that actually govern the zero set.
fn signed_control_numerators(patch: &BezierSurface, distances: &[f64]) -> Vec<f64> {
    patch
        .control_points()
        .as_slice()
        .iter()
        .zip(distances)
        .map(|(point, distance)| distance * point.weight())
        .collect()
}

fn monotone_direction(
    patch: &BezierSurface,
    numerators: &[f64],
    tolerance: f64,
) -> Option<MonotoneDirection> {
    let net = patch.control_points();
    let (nu, nv) = (net.nu(), net.nv());
    let along_u = (0..nv).flat_map(|v| {
        (0..nu.saturating_sub(1)).map(move |u| numerators[v * nu + u + 1] - numerators[v * nu + u])
    });
    let along_v = (0..nu).flat_map(|u| {
        (0..nv.saturating_sub(1))
            .map(move |v| numerators[(v + 1) * nu + u] - numerators[v * nu + u])
    });
    if weakly_same_sign(along_u, tolerance) {
        Some(MonotoneDirection::U)
    } else if weakly_same_sign(along_v, tolerance) {
        Some(MonotoneDirection::V)
    } else {
        None
    }
}

/// Returns whether every value clears `tolerance` with the same sign.
fn strictly_same_sign(values: impl Iterator<Item = f64>, tolerance: f64) -> bool {
    let values = values.collect::<Vec<_>>();
    !values.is_empty()
        && (values.iter().all(|value| *value > tolerance)
            || values.iter().all(|value| *value < -tolerance))
}

/// Returns whether no value opposes the sign of those clearing `tolerance`.
///
/// Applied to consecutive coefficient differences this is weak monotonicity,
/// which the Bernstein derivative inherits: the zero set stays connected along
/// that direction even where the derivative vanishes.
fn weakly_same_sign(values: impl Iterator<Item = f64>, tolerance: f64) -> bool {
    let values = values.collect::<Vec<_>>();
    (values.iter().all(|value| *value >= 0.0) && values.iter().any(|value| *value > tolerance))
        || (values.iter().all(|value| *value <= 0.0)
            && values.iter().any(|value| *value < -tolerance))
}

/// Returns whether an edge numerator crosses the plane at most once.
///
/// A sign-definite edge never crosses and a monotone one crosses once; anything
/// else may hide several arcs and must be subdivided.
fn edge_has_simple_root(
    patch: &BezierSurface,
    numerators: &[f64],
    edge: PatchEdge,
    tolerance: f64,
) -> bool {
    let coefficients = edge.coefficients(patch, numerators);
    strictly_same_sign(coefficients.iter().copied(), tolerance)
        || weakly_same_sign(
            coefficients.windows(2).map(|pair| pair[1] - pair[0]),
            tolerance,
        )
}

/// Bisects one patch edge for the single parameter where it meets the plane.
fn isolate_edge_root(
    patch: &BezierSurface,
    plane: PlaneEquation,
    edge: PatchEdge,
    options: IntersectionOptions,
) -> Option<Point2> {
    let domain = edge.varying(patch);
    let evaluate = |parameter: f64| {
        let uv = edge.parameters(patch, parameter);
        (patch.point_at(uv.x, uv.y) - plane.origin).dot(&plane.normal)
    };
    let mut lower = domain.start;
    let mut upper = domain.end;
    let mut lower_value = evaluate(lower);
    let upper_value = evaluate(upper);
    if lower_value.abs() <= options.residual_tolerance {
        return Some(edge.parameters(patch, lower));
    }
    if upper_value.abs() <= options.residual_tolerance {
        return Some(edge.parameters(patch, upper));
    }
    if lower_value.signum() == upper_value.signum() {
        return None;
    }
    for _ in 0..options.newton_max_iterations * 4 {
        let midpoint = 0.5 * (lower + upper);
        let value = evaluate(midpoint);
        if value.abs() <= options.residual_tolerance || upper - lower <= options.parameter_tolerance
        {
            return Some(edge.parameters(patch, midpoint));
        }
        if value.signum() == lower_value.signum() {
            lower = midpoint;
            lower_value = value;
        } else {
            upper = midpoint;
        }
    }
    None
}

fn push_reason(
    reasons: &mut Vec<IntersectionIncompleteReason>,
    reason: IntersectionIncompleteReason,
) {
    if !reasons.contains(&reason) {
        reasons.push(reason);
    }
}

fn collect_surface_boundaries(
    boundary_surface: &NurbsSurface,
    other_surface: &NurbsSurface,
    boundary_belongs_to_a: bool,
    options: IntersectionOptions,
    search: &mut SeedSearch,
) -> Result<(), IntersectionError> {
    // The other surface is decomposed once and reused across all four
    // boundaries rather than re-decomposed per boundary curve.
    let other_prepared = PreparedSurface::new(&Surface::Nurbs(other_surface.clone()))?;
    for boundary in [
        Boundary::UMin,
        Boundary::UMax,
        Boundary::VMin,
        Boundary::VMax,
    ] {
        let curve = boundary_curve(boundary_surface, boundary)?;
        let results = intersect_prepared_curve_surface(
            &PreparedCurve::new(&Curve::Nurbs(curve))?,
            &other_prepared,
            options,
        )?;
        if let IntersectionCoverage::Incomplete(reasons) = results.coverage() {
            for reason in reasons {
                if !search.incomplete_reasons.contains(reason) {
                    search.incomplete_reasons.push(*reason);
                }
            }
        }
        for result in results {
            match result {
                CurveSurfaceIntersection::Point {
                    curve_u,
                    surface_u,
                    surface_v,
                    ..
                } => {
                    let boundary_uv = boundary_parameters(boundary_surface, boundary, curve_u);
                    let parameters = if boundary_belongs_to_a {
                        Vector4::new(boundary_uv.0, boundary_uv.1, surface_u, surface_v)
                    } else {
                        Vector4::new(surface_u, surface_v, boundary_uv.0, boundary_uv.1)
                    };
                    search.seeds.push(TraceState::new(
                        if boundary_belongs_to_a {
                            boundary_surface
                        } else {
                            other_surface
                        },
                        if boundary_belongs_to_a {
                            other_surface
                        } else {
                            boundary_surface
                        },
                        parameters,
                    ));
                }
                CurveSurfaceIntersection::Overlap { .. } => {
                    search.overlap_boundary_found = true;
                }
            }
        }
    }
    Ok(())
}

fn boundary_curve(
    surface: &NurbsSurface,
    boundary: Boundary,
) -> Result<NurbsCurve, IntersectionError> {
    let control_net = surface.control_points();
    let (degree, knots, points) = match boundary {
        Boundary::UMin => (
            surface.degree_v(),
            surface.knots_v().clone(),
            (0..control_net.nv())
                .map(|v| control_net.get(0, v))
                .collect(),
        ),
        Boundary::UMax => (
            surface.degree_v(),
            surface.knots_v().clone(),
            (0..control_net.nv())
                .map(|v| control_net.get(control_net.nu() - 1, v))
                .collect(),
        ),
        Boundary::VMin => (
            surface.degree_u(),
            surface.knots_u().clone(),
            (0..control_net.nu())
                .map(|u| control_net.get(u, 0))
                .collect(),
        ),
        Boundary::VMax => (
            surface.degree_u(),
            surface.knots_u().clone(),
            (0..control_net.nu())
                .map(|u| control_net.get(u, control_net.nv() - 1))
                .collect(),
        ),
    };
    Ok(NurbsCurve::new(
        degree,
        ControlPolygon::new(points)?,
        knots,
    )?)
}

fn boundary_parameters(
    surface: &NurbsSurface,
    boundary: Boundary,
    curve_parameter: f64,
) -> (f64, f64) {
    match boundary {
        Boundary::UMin => (surface.domain_u().start, curve_parameter),
        Boundary::UMax => (surface.domain_u().end, curve_parameter),
        Boundary::VMin => (curve_parameter, surface.domain_v().start),
        Boundary::VMax => (curve_parameter, surface.domain_v().end),
    }
}

fn dedup_seeds(seeds: &mut Vec<TraceState>, options: IntersectionOptions) {
    let mut unique = Vec::new();
    for seed in seeds.drain(..) {
        if unique.iter().any(|existing: &TraceState| {
            (existing.parameters - seed.parameters).norm() <= options.parameter_tolerance * 10.0
                || (existing.point - seed.point).norm() <= options.linear_tolerance
        }) {
            continue;
        }
        unique.push(seed);
    }
    *seeds = unique;
}
