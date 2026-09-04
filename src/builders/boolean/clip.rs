//! Synchronized interval clipping; never reconnect filtered branch samples.

use crate::builders::faces::FaceImprint;
use crate::geometry::{
    CurveIntersectionOptions, IntersectionOptions, Interval, SurfaceIntersectionBranch,
};

use super::{BooleanError, trim::FaceTrimDomain};

/// Cuts at crossings on both faces and preserves the exact synchronized curves.
pub(crate) fn clip_branch(
    branch: &SurfaceIntersectionBranch,
    first: &FaceTrimDomain,
    second: &FaceTrimDomain,
    options: IntersectionOptions,
) -> Result<Vec<[FaceImprint; 2]>, BooleanError> {
    let curve_options = CurveIntersectionOptions {
        linear_tolerance: options.parameter_tolerance,
        parameter_tolerance: options.parameter_tolerance,
        bbox_tolerance: options.parameter_tolerance,
        max_subdivision_depth: options.max_subdivision_depth,
        leaf_diagonal_tolerance: options.parameter_tolerance * 10.0,
        newton_max_iterations: options.newton_max_iterations,
    };
    let mut parameters = vec![0.0, 1.0];
    first.crossings(&branch.pcurve_a, curve_options, &mut parameters)?;
    second.crossings(&branch.pcurve_b, curve_options, &mut parameters)?;
    // A closed branch needs distinct endpoints for the existing network representation.
    if branch.closed {
        parameters.push(0.5);
    }
    parameters.sort_by(f64::total_cmp);
    parameters.dedup_by(|a, b| (*a - *b).abs() <= options.parameter_tolerance);
    let mut fragments = Vec::new();
    for pair in parameters.windows(2) {
        let midpoint = (pair[0] + pair[1]) * 0.5;
        if !first.contains(branch.pcurve_a.point_at(midpoint))
            || !second.contains(branch.pcurve_b.point_at(midpoint))
        {
            continue;
        }
        let interval = Interval::new(pair[0], pair[1]);
        let curve = super::graph::normalized_subcurve(&branch.curve_3d, interval)?;
        fragments.push([
            FaceImprint::new(curve.clone(), branch.pcurve_a.trimmed(interval)?),
            FaceImprint::new(curve, branch.pcurve_b.trimmed(interval)?),
        ]);
    }
    Ok(fragments)
}
