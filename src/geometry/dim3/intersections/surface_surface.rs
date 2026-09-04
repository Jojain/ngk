mod fitting;
mod seeds;
mod simplification;
mod tracer;

use fitting::fit_branch;
use seeds::boundary_seeds;
use tracer::{TraceState, trace_from_seed};

use super::error::IntersectionError;
use super::options::IntersectionOptions;
use super::{
    IntersectionCoverage, SurfaceIntersectionIncompleteReason, SurfaceIntersectionPointKind,
    SurfaceOverlapCandidate, SurfaceSurfaceIntersection, SurfaceSurfaceIntersections,
};
use crate::geometry::{BBox, NurbsSurface, Surface};

/// Intersects two surfaces with the default operation-scoped tolerances.
pub fn intersect_surfaces(
    a: &Surface,
    b: &Surface,
) -> Result<SurfaceSurfaceIntersections, IntersectionError> {
    intersect_surfaces_with_options(a, b, IntersectionOptions::default())
}

/// Intersects two surfaces and returns ordered observations with explicit coverage.
pub fn intersect_surfaces_with_options(
    a: &Surface,
    b: &Surface,
    options: IntersectionOptions,
) -> Result<SurfaceSurfaceIntersections, IntersectionError> {
    if !options.validate() {
        return Err(IntersectionError::InvalidOptions);
    }

    let a = a.to_nurbs()?;
    let b = b.to_nurbs()?;
    if !has_supported_weights(&a) || !has_supported_weights(&b) {
        return Ok(SurfaceSurfaceIntersections::new(
            Vec::new(),
            IntersectionCoverage::Incomplete(vec![
                SurfaceIntersectionIncompleteReason::UnsupportedControlPointWeights,
            ]),
        ));
    }
    if control_hulls_are_disjoint(&a, &b, options)
        || bezier_span_hulls_are_disjoint(&a, &b, options)?
    {
        return Ok(SurfaceSurfaceIntersections::new(
            Vec::new(),
            IntersectionCoverage::Complete,
        ));
    }
    if same_nurbs_surface(&a, &b, options) {
        return Ok(SurfaceSurfaceIntersections::new(
            vec![SurfaceSurfaceIntersection::OverlapCandidate(
                SurfaceOverlapCandidate {
                    domain_a_u: a.domain_u(),
                    domain_a_v: a.domain_v(),
                    domain_b_u: b.domain_u(),
                    domain_b_v: b.domain_v(),
                },
            )],
            IntersectionCoverage::Incomplete(vec![
                SurfaceIntersectionIncompleteReason::CoincidentRegionResolutionNotImplemented,
            ]),
        ));
    }

    let seed_search = boundary_seeds(&a, &b, options)?;
    let mut intersections = Vec::new();
    let mut reasons = vec![SurfaceIntersectionIncompleteReason::InteriorLoopSearchNotImplemented];
    if seed_search.overlap_boundary_found {
        push_reason(
            &mut reasons,
            SurfaceIntersectionIncompleteReason::TangentOrSingularContact,
        );
    }

    for seed in seed_search.seeds {
        let Some(outcome) = trace_from_seed(&a, &b, seed, options) else {
            push_singular_point(&mut intersections, seed, options);
            push_reason(
                &mut reasons,
                SurfaceIntersectionIncompleteReason::TangentOrSingularContact,
            );
            continue;
        };
        for reason in outcome.incomplete_reasons {
            push_reason(&mut reasons, reason);
        }
        if outcome.states.len() < 2 {
            push_singular_point(&mut intersections, seed, options);
            continue;
        }
        let branch = fit_branch(&a, &b, outcome.states, outcome.closed, options)?;
        if !branch.quality.certified {
            push_reason(
                &mut reasons,
                SurfaceIntersectionIncompleteReason::SynchronizedFitToleranceExceeded,
            );
        }
        if !contains_equivalent_branch(&intersections, &branch, options) {
            intersections.push(SurfaceSurfaceIntersection::Branch(branch));
        }
    }

    Ok(SurfaceSurfaceIntersections::new(
        intersections,
        IntersectionCoverage::Incomplete(reasons),
    ))
}

fn has_supported_weights(surface: &NurbsSurface) -> bool {
    surface
        .control_points()
        .as_slice()
        .iter()
        .all(|point| point.weight().is_finite() && point.weight() > 0.0)
}

fn control_hulls_are_disjoint(
    a: &NurbsSurface,
    b: &NurbsSurface,
    options: IntersectionOptions,
) -> bool {
    let bbox_a = BBox::from_points(
        a.control_points()
            .as_slice()
            .iter()
            .map(|point| point.to_cartesian()),
    );
    let bbox_b = BBox::from_points(
        b.control_points()
            .as_slice()
            .iter()
            .map(|point| point.to_cartesian()),
    );
    !bbox_a.intersects(&bbox_b, options.bbox_tolerance)
}

/// Uses exact Bézier decomposition so non-overlapping local control hulls are rejected conservatively.
fn bezier_span_hulls_are_disjoint(
    a: &NurbsSurface,
    b: &NurbsSurface,
    options: IntersectionOptions,
) -> Result<bool, IntersectionError> {
    let spans_a = a.bezier_spans()?;
    let spans_b = b.bezier_spans()?;
    Ok(!spans_a.iter().any(|span_a| {
        let bbox_a = span_a.bbox();
        spans_b
            .iter()
            .any(|span_b| bbox_a.intersects(&span_b.bbox(), options.bbox_tolerance))
    }))
}

fn same_nurbs_surface(a: &NurbsSurface, b: &NurbsSurface, options: IntersectionOptions) -> bool {
    a.degree_u() == b.degree_u()
        && a.degree_v() == b.degree_v()
        && a.knots_u().as_slice() == b.knots_u().as_slice()
        && a.knots_v().as_slice() == b.knots_v().as_slice()
        && a.control_points().nu() == b.control_points().nu()
        && a.control_points().nv() == b.control_points().nv()
        && a.control_points()
            .as_slice()
            .iter()
            .zip(b.control_points().as_slice())
            .all(|(a, b)| {
                (a.0.coords - b.0.coords).norm()
                    <= options.linear_tolerance * a.weight().abs().max(b.weight().abs()).max(1.0)
            })
}

fn push_singular_point(
    intersections: &mut Vec<SurfaceSurfaceIntersection>,
    seed: TraceState,
    options: IntersectionOptions,
) {
    if intersections.iter().any(|intersection| match intersection {
        SurfaceSurfaceIntersection::Point(point) => {
            (point.point - seed.point).norm() <= options.linear_tolerance
        }
        SurfaceSurfaceIntersection::Branch(_) | SurfaceSurfaceIntersection::OverlapCandidate(_) => {
            false
        }
    }) {
        return;
    }
    intersections.push(SurfaceSurfaceIntersection::Point(
        seed.sample(SurfaceIntersectionPointKind::Singular),
    ));
}

fn contains_equivalent_branch(
    intersections: &[SurfaceSurfaceIntersection],
    candidate: &super::SurfaceIntersectionBranch,
    options: IntersectionOptions,
) -> bool {
    let Some((candidate_start, candidate_end)) =
        candidate.samples.first().zip(candidate.samples.last())
    else {
        return false;
    };
    intersections.iter().any(|intersection| {
        let SurfaceSurfaceIntersection::Branch(existing) = intersection else {
            return false;
        };
        let Some((existing_start, existing_end)) =
            existing.samples.first().zip(existing.samples.last())
        else {
            return false;
        };
        let same_direction = (candidate_start.point - existing_start.point).norm()
            <= options.linear_tolerance * 10.0
            && (candidate_end.point - existing_end.point).norm() <= options.linear_tolerance * 10.0;
        let reverse_direction = (candidate_start.point - existing_end.point).norm()
            <= options.linear_tolerance * 10.0
            && (candidate_end.point - existing_start.point).norm()
                <= options.linear_tolerance * 10.0;
        same_direction || reverse_direction
    })
}

fn push_reason(
    reasons: &mut Vec<SurfaceIntersectionIncompleteReason>,
    reason: SurfaceIntersectionIncompleteReason,
) {
    if !reasons.contains(&reason) {
        reasons.push(reason);
    }
}
