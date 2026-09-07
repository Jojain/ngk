mod fitting;
mod normals;
mod seeds;
mod simplification;
mod tracer;

use fitting::fit_branch;
use seeds::{pair_seeds, planar_seeds};
use tracer::{TraceState, trace_from_seed};

use super::PreparedSurface;
use super::analytic::{AnalyticSection, AnalyticSurfaceIntersection, intersect_analytic_surfaces};
use super::error::IntersectionError;
use super::options::IntersectionOptions;
use super::{
    IntersectionCoverage, IntersectionIncompleteReason, IntersectionQuality,
    SurfaceIntersectionBranch, SurfaceIntersectionBranchKind, SurfaceIntersectionPoint,
    SurfaceIntersectionPointKind, SurfaceOverlapCandidate, SurfaceSurfaceIntersection,
    SurfaceSurfaceIntersections,
};
use crate::geometry::counters::count_surface_surface_call;
use crate::geometry::{BBox, Curve, NurbsSurface, Point2, Surface};

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

    // Ask the closed-form table before converting anything: the conversion and
    // the search that follows it are the expensive part, and for a recognized
    // pair both are avoidable entirely.
    if let Some(analytic) = intersect_analytic_surfaces(a, b, options)
        && let Some(intersections) = analytic_intersections(analytic?, a, b, options)
    {
        return Ok(intersections);
    }

    let nurbs_a = a.to_nurbs()?;
    let nurbs_b = b.to_nurbs()?;
    intersect_nurbs_surfaces(a, b, &nurbs_a, &nurbs_b, options)
}

/// Presents a closed-form answer in the general solver's result vocabulary.
///
/// The branches this builds are marked certified: their curve is exact and
/// each pcurve carries a measured deviation, which is a stronger guarantee
/// than a traced branch can offer, not a weaker one.
///
/// Returns `None` for an answer the vocabulary cannot carry. A branch is
/// normalized over `[0, 1]`, so an unbounded section -- two planes meeting in
/// a line, a plane running along a cylinder -- would be silently truncated to
/// whatever window it was written over. Those callers get the general solver,
/// which realizes both supports over finite patches first; the closed form is
/// still available directly from [`intersect_analytic_surfaces`].
pub fn analytic_intersections(
    analytic: AnalyticSurfaceIntersection,
    a: &Surface,
    b: &Surface,
    options: IntersectionOptions,
) -> Option<SurfaceSurfaceIntersections> {
    if analytic
        .sections()
        .iter()
        .any(|section| matches!(section.curve, Curve::Line(_)))
    {
        return None;
    }
    Some(match analytic {
        AnalyticSurfaceIntersection::Empty => {
            SurfaceSurfaceIntersections::new(Vec::new(), IntersectionCoverage::Complete)
        }
        AnalyticSurfaceIntersection::TangentPoint(point) => {
            let uv_a = a.closest_parameter(point).unwrap_or(Point2::origin());
            let uv_b = b.closest_parameter(point).unwrap_or(Point2::origin());
            SurfaceSurfaceIntersections::new(
                vec![SurfaceSurfaceIntersection::Point(
                    SurfaceIntersectionPoint {
                        point,
                        uv_a,
                        uv_b,
                        kind: SurfaceIntersectionPointKind::Tangent,
                        residual: 0.0,
                    },
                )],
                IntersectionCoverage::Complete,
            )
        }
        // Which part of two identical supports is shared is a trimming
        // question, and answering it is no more in scope here than it is for
        // the traced path.
        AnalyticSurfaceIntersection::Coincident => SurfaceSurfaceIntersections::new(
            vec![SurfaceSurfaceIntersection::OverlapCandidate(
                overlap_candidate(a, b),
            )],
            IntersectionCoverage::Incomplete(vec![
                IntersectionIncompleteReason::CoincidentRegionResolutionNotImplemented,
            ]),
        ),
        AnalyticSurfaceIntersection::Sections(sections) => {
            let mut branches = Vec::with_capacity(sections.len());
            let mut reasons = Vec::new();
            for section in sections {
                let deviation = section.fidelity.deviation();
                if deviation > options.fit_tolerance {
                    push_reason(
                        &mut reasons,
                        IntersectionIncompleteReason::SynchronizedFitToleranceExceeded,
                    );
                }
                branches.push(SurfaceSurfaceIntersection::Branch(analytic_branch(
                    section, deviation, options,
                )));
            }
            SurfaceSurfaceIntersections::new(
                branches,
                if reasons.is_empty() {
                    IntersectionCoverage::Complete
                } else {
                    IntersectionCoverage::Incomplete(reasons)
                },
            )
        }
    })
}

/// Builds the branch record for one closed-form section.
fn analytic_branch(
    section: AnalyticSection,
    deviation: f64,
    options: IntersectionOptions,
) -> SurfaceIntersectionBranch {
    let samples = (0..=ANALYTIC_BRANCH_SAMPLES)
        .map(|index| {
            let parameter = index as f64 / ANALYTIC_BRANCH_SAMPLES as f64;
            let point = section.curve.point_at(parameter);
            SurfaceIntersectionPoint {
                point,
                uv_a: section.pcurve_a.point_at(parameter),
                uv_b: section.pcurve_b.point_at(parameter),
                kind: SurfaceIntersectionPointKind::Transverse,
                residual: 0.0,
            }
        })
        .collect::<Vec<_>>();
    let closed = (section.curve.point_at(0.0) - section.curve.point_at(1.0)).norm()
        <= options.linear_tolerance;
    SurfaceIntersectionBranch {
        curve_3d: section.curve,
        pcurve_a: section.pcurve_a,
        pcurve_b: section.pcurve_b,
        samples,
        closed,
        kind: SurfaceIntersectionBranchKind::Transverse,
        quality: IntersectionQuality {
            max_residual: 0.0,
            max_fit_error: deviation,
            certified: deviation <= options.fit_tolerance,
        },
    }
}

/// Returns the whole-domain overlap candidate for two coincident supports.
fn overlap_candidate(a: &Surface, b: &Surface) -> SurfaceOverlapCandidate {
    let (a_u, a_v) = a.domain();
    let (b_u, b_v) = b.domain();
    SurfaceOverlapCandidate {
        domain_a_u: a_u,
        domain_a_v: a_v,
        domain_b_u: b_u,
        domain_b_v: b_v,
    }
}

/// Intersects two surfaces already realized over operation-specific domains.
pub fn intersect_prepared_surfaces(
    a: &PreparedSurface,
    b: &PreparedSurface,
    options: IntersectionOptions,
) -> Result<SurfaceSurfaceIntersections, IntersectionError> {
    if !options.validate() {
        return Err(IntersectionError::InvalidOptions);
    }
    intersect_nurbs_surfaces(a.source(), b.source(), a.nurbs(), b.nurbs(), options)
}

fn intersect_nurbs_surfaces(
    source_a: &Surface,
    source_b: &Surface,
    a: &NurbsSurface,
    b: &NurbsSurface,
    options: IntersectionOptions,
) -> Result<SurfaceSurfaceIntersections, IntersectionError> {
    count_surface_surface_call();
    if !has_supported_weights(a) || !has_supported_weights(b) {
        return Ok(SurfaceSurfaceIntersections::new(
            Vec::new(),
            IntersectionCoverage::Incomplete(vec![
                IntersectionIncompleteReason::UnsupportedControlPointWeights,
            ]),
        ));
    }
    if control_hulls_are_disjoint(a, b, options) || bezier_span_hulls_are_disjoint(a, b, options)? {
        return Ok(SurfaceSurfaceIntersections::new(
            Vec::new(),
            IntersectionCoverage::Complete,
        ));
    }
    if same_nurbs_surface(a, b, options) {
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
                IntersectionIncompleteReason::CoincidentRegionResolutionNotImplemented,
            ]),
        ));
    }

    // A planar operand is searched by Bernstein sign arguments on the other
    // surface; otherwise the pair is subdivided until its normal cones separate.
    let seed_search = match planar_seeds(a, b, options)? {
        Some(search) => search,
        None => pair_seeds(a, b, options)?,
    };
    let mut intersections = Vec::new();
    let mut reasons = Vec::new();
    // Seeding cannot certify more than the curve/surface searches it is built
    // from, so its limitations become limitations here.
    for reason in &seed_search.incomplete_reasons {
        push_reason(&mut reasons, *reason);
    }
    // Tangential branches are already exact, so they are adopted before the
    // traced ones and keep any traced duplicate from being added on top.
    for branch in seed_search.tangencies {
        if !contains_equivalent_branch(&intersections, &branch, options) {
            intersections.push(SurfaceSurfaceIntersection::Branch(branch));
        }
    }
    if seed_search.overlap_boundary_found {
        push_reason(
            &mut reasons,
            IntersectionIncompleteReason::TangentOrSingularContact,
        );
    }

    for seed in seed_search.seeds {
        if branch_contains_seed(&intersections, seed, options) {
            continue;
        }
        let Some(outcome) = trace_from_seed(a, b, seed, options) else {
            push_singular_point(&mut intersections, seed, options);
            push_reason(
                &mut reasons,
                IntersectionIncompleteReason::TangentOrSingularContact,
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
        let mut branch = fit_branch(source_a, source_b, outcome.states, outcome.closed, options)?;
        if !branch.quality.certified {
            branch = refit_with_denser_trace(source_a, source_b, a, b, seed, branch, options)?;
        }
        if !branch.quality.certified {
            push_reason(
                &mut reasons,
                IntersectionIncompleteReason::SynchronizedFitToleranceExceeded,
            );
        }
        if !contains_equivalent_branch(&intersections, &branch, options) {
            intersections.push(SurfaceSurfaceIntersection::Branch(branch));
        }
    }

    let coverage = if reasons.is_empty() {
        IntersectionCoverage::Complete
    } else {
        IntersectionCoverage::Incomplete(reasons)
    };
    Ok(SurfaceSurfaceIntersections::new(intersections, coverage))
}

/// How many times a branch that missed the fit tolerance is retraced.
/// Samples recorded on a closed-form branch.
///
/// The curve is exact, so these exist only for callers that node a branch
/// against its samples rather than against its curve.
const ANALYTIC_BRANCH_SAMPLES: usize = 32;

const MAX_FIT_REFINEMENTS: usize = 4;

/// Sample count past which a denser trace costs more than the fit can gain.
///
/// Interpolation through the trace is dense, so doubling the sample count
/// multiplies the fitting work; a branch this long is reported uncertified
/// instead of retraced again.
const MAX_FIT_SAMPLES: usize = 512;

/// Retraces a branch with shorter steps until its fitted curves are certified.
///
/// A curve of higher degree than the interpolant — a cylinder/cylinder quartic,
/// say — needs more samples than the default step produces. Each round halves
/// the step and keeps the result only while the measured fit error improves, so
/// a branch that cannot be certified costs a bounded amount of work and is still
/// reported with its best fit rather than silently accepted.
fn refit_with_denser_trace(
    source_a: &Surface,
    source_b: &Surface,
    a: &NurbsSurface,
    b: &NurbsSurface,
    seed: TraceState,
    mut branch: SurfaceIntersectionBranch,
    options: IntersectionOptions,
) -> Result<SurfaceIntersectionBranch, IntersectionError> {
    let mut refined_options = options;
    for _ in 0..MAX_FIT_REFINEMENTS {
        refined_options.max_trace_step *= 0.5;
        refined_options.min_trace_step *= 0.5;
        let Some(outcome) = trace_from_seed(a, b, seed, refined_options) else {
            break;
        };
        if outcome.states.len() < 2 || outcome.states.len() > MAX_FIT_SAMPLES {
            break;
        }
        let refined = fit_branch(
            source_a,
            source_b,
            outcome.states,
            outcome.closed,
            refined_options,
        )?;
        if refined.quality.max_fit_error >= branch.quality.max_fit_error {
            break;
        }
        let certified = refined.quality.certified;
        branch = refined;
        if certified {
            break;
        }
    }
    Ok(branch)
}

fn branch_contains_seed(
    intersections: &[SurfaceSurfaceIntersection],
    seed: TraceState,
    options: IntersectionOptions,
) -> bool {
    intersections.iter().any(|intersection| {
        let SurfaceSurfaceIntersection::Branch(branch) = intersection else {
            return false;
        };
        let parameter = branch.curve_3d.param_at(seed.point);
        (branch.curve_3d.point_at(parameter) - seed.point).norm() <= options.fit_tolerance
    })
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
    reasons: &mut Vec<IntersectionIncompleteReason>,
    reason: IntersectionIncompleteReason,
) {
    if !reasons.contains(&reason) {
        reasons.push(reason);
    }
}
