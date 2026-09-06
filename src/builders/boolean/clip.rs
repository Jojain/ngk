//! Synchronized interval clipping; never reconnect filtered branch samples.

use nalgebra::Vector2;

use crate::builders::faces::FaceImprint;
use crate::geometry::{
    ControlPolygon, ControlPolygon2, Curve, Curve2, CurveIntersectionOptions, HPoint, HPoint2,
    IntersectionOptions, Interval, Line2, NurbsCurve, NurbsCurve2, NurbsError, Point2, Point3,
    Surface, SurfaceIntersectionBranch, SurfacePeriodicity,
};

use super::{BooleanError, trim::FaceTrimDomain};

/// One face of a branch's pair: its surface and its trimmed domain.
pub(crate) type ClipSide<'a> = (&'a Surface, &'a FaceTrimDomain);

/// A parameter at which an exact contact point nodes a fitted branch.
///
/// The branch is a fit, so its own crossing parameters carry the fit's error.
/// Where an operand already knows a point exactly — an operand vertex on the
/// other face, say — that point is the truthful node, and the fragments meeting
/// there are corrected onto it.
struct BranchNode {
    parameter: f64,
    point: Point3,
    /// Parameter-space correction on each face, measured at the branch itself
    /// so a periodic surface stays on the branch's own image of the domain.
    correction: [Vector2<f64>; 2],
}

/// Cuts at crossings on both faces and preserves the exact synchronized curves.
///
/// `anchors` are points both operands already agree on exactly. A crossing that
/// falls within `capture` of one is that anchor seen through the fit's error, so
/// the anchor replaces it and the fragments are corrected onto it.
pub(crate) fn clip_branch(
    branch: &SurfaceIntersectionBranch,
    first: ClipSide<'_>,
    second: ClipSide<'_>,
    anchors: &[Point3],
    capture: f64,
    options: IntersectionOptions,
) -> Result<Vec<[FaceImprint; 2]>, BooleanError> {
    let pcurve_a = periodic_pcurve_image(&branch.pcurve_a, first.0, first.1)?;
    let pcurve_b = periodic_pcurve_image(&branch.pcurve_b, second.0, second.1)?;
    let curve_options = CurveIntersectionOptions {
        linear_tolerance: options.parameter_tolerance,
        parameter_tolerance: options.parameter_tolerance,
        bbox_tolerance: options.parameter_tolerance,
        max_subdivision_depth: options.max_subdivision_depth,
        leaf_diagonal_tolerance: options.parameter_tolerance * 10.0,
        newton_max_iterations: options.newton_max_iterations,
    };
    let mut crossings = Vec::new();
    first
        .1
        .crossings(&pcurve_a, curve_options, &mut crossings)?;
    second
        .1
        .crossings(&pcurve_b, curve_options, &mut crossings)?;
    let nodes = branch_nodes(
        branch,
        first.0,
        second.0,
        [&pcurve_a, &pcurve_b],
        anchors,
        capture,
        &crossings,
    );
    let captured = |parameter: f64| {
        nodes
            .iter()
            .any(|node| (branch.curve_3d.point_at(parameter) - node.point).norm() <= capture)
    };
    crossings.retain(|parameter| !captured(*parameter));
    let mut parameters = vec![0.0, 1.0];
    parameters.append(&mut crossings);
    parameters.extend(nodes.iter().map(|node| node.parameter));
    // A closed branch needs distinct endpoints for the existing network representation.
    if branch.closed {
        parameters.push(0.5);
    }
    parameters.sort_by(f64::total_cmp);
    parameters.dedup_by(|a, b| (*a - *b).abs() <= options.parameter_tolerance);
    let mut fragments = Vec::new();
    for pair in parameters.windows(2) {
        let midpoint = (pair[0] + pair[1]) * 0.5;
        if !first.1.contains(pcurve_a.point_at(midpoint))
            || !second.1.contains(pcurve_b.point_at(midpoint))
        {
            continue;
        }
        let interval = Interval::new(pair[0], pair[1]);
        let mut curve = super::graph::normalized_subcurve(&branch.curve_3d, interval)?;
        let mut pcurve_a = pcurve_a.trimmed(interval)?;
        let mut pcurve_b = pcurve_b.trimmed(interval)?;
        for (index, at_start) in [(0usize, true), (1usize, false)] {
            let Some(node) = nodes
                .iter()
                .find(|node| (node.parameter - pair[index]).abs() <= options.parameter_tolerance)
            else {
                continue;
            };
            let end = if at_start { 0.0 } else { 1.0 };
            if (curve.point_at(end) - node.point).norm() > options.linear_tolerance {
                curve = snapped_curve(&curve, at_start, node.point)?;
            }
            pcurve_a = snapped_pcurve(&pcurve_a, at_start, node.correction[0])?;
            pcurve_b = snapped_pcurve(&pcurve_b, at_start, node.correction[1])?;
        }
        fragments.push([
            FaceImprint::new(curve.clone(), pcurve_a),
            FaceImprint::new(curve, pcurve_b),
        ]);
    }
    Ok(fragments)
}

/// Locates the anchors that correct a crossing of this branch.
///
/// An anchor away from every crossing is not a node: the branch does not leave
/// the trimmed region there, and cutting it anyway would leave the network with
/// a section end no other cell answers.
fn branch_nodes(
    branch: &SurfaceIntersectionBranch,
    first: &Surface,
    second: &Surface,
    pcurves: [&Curve2; 2],
    anchors: &[Point3],
    capture: f64,
    crossings: &[f64],
) -> Vec<BranchNode> {
    let mut nodes: Vec<BranchNode> = Vec::new();
    for anchor in anchors.iter().copied() {
        let parameter = branch.curve_3d.param_at(anchor).clamp(0.0, 1.0);
        let fitted = branch.curve_3d.point_at(parameter);
        if (fitted - anchor).norm() > capture {
            continue;
        }
        if !crossings
            .iter()
            .any(|crossing| (branch.curve_3d.point_at(*crossing) - anchor).norm() <= capture)
        {
            continue;
        }
        // The pcurves are fits of their own, so each carries a parameter-space
        // error the exact point settles independently of the 3D curve's.
        let correction = [(first, pcurves[0]), (second, pcurves[1])].map(|(surface, pcurve)| {
            let fitted_uv = pcurve.point_at(parameter);
            let Ok(exact_uv) = surface.closest_parameter(anchor) else {
                return Vector2::zeros();
            };
            nearest_periodic_image(surface, exact_uv, fitted_uv) - fitted_uv
        });
        if nodes
            .iter()
            .any(|node| (node.parameter - parameter).abs() <= f64::EPSILON)
        {
            continue;
        }
        nodes.push(BranchNode {
            parameter,
            point: anchor,
            correction,
        });
    }
    nodes
}

/// Moves a synchronized pcurve to the periodic image occupied by the face trim.
fn periodic_pcurve_image(
    curve: &Curve2,
    surface: &Surface,
    trim: &FaceTrimDomain,
) -> Result<Curve2, NurbsError> {
    let reference = curve.point_at(0.5);
    let center = trim.chart_center();
    let mut offset = Vector2::zeros();
    let nearest_shift =
        |value: f64, target: f64, period: f64| ((target - value) / period).round() * period;
    match surface.periodicity() {
        SurfacePeriodicity::UPeriodic(period) => {
            offset.x = nearest_shift(reference.x, center.x, period)
        }
        SurfacePeriodicity::VPeriodic(period) => {
            offset.y = nearest_shift(reference.y, center.y, period)
        }
        SurfacePeriodicity::UVPeriodic(u_period, v_period) => {
            offset.x = nearest_shift(reference.x, center.x, u_period);
            offset.y = nearest_shift(reference.y, center.y, v_period);
        }
        SurfacePeriodicity::None => {}
    }
    curve.translated(offset)
}

/// Shifts `uv` by whole periods until it is the image nearest `reference`.
fn nearest_periodic_image(surface: &Surface, mut uv: Point2, reference: Point2) -> Point2 {
    let fold = |value: &mut f64, target: f64, period: f64| {
        *value += ((target - *value) / period).round() * period;
    };
    match surface.periodicity() {
        SurfacePeriodicity::UPeriodic(period) => fold(&mut uv.x, reference.x, period),
        SurfacePeriodicity::VPeriodic(period) => fold(&mut uv.y, reference.y, period),
        SurfacePeriodicity::UVPeriodic(u_period, v_period) => {
            fold(&mut uv.x, reference.x, u_period);
            fold(&mut uv.y, reference.y, v_period);
        }
        SurfacePeriodicity::None => {}
    }
    uv
}

/// Moves one end of a fitted curve onto an exact point, leaving the rest in place.
///
/// A clamped NURBS interpolates its end control points, so replacing one moves
/// exactly that end and nothing else beyond its first span. Only a fit is
/// rewritten: an analytically simplified branch is exact geometry, and
/// re-expressing it as a NURBS would trade its own parameterization for the
/// rational one and desynchronize it from its pcurves.
fn snapped_curve(curve: &Curve, at_start: bool, point: Point3) -> Result<Curve, NurbsError> {
    let Curve::Nurbs(nurbs) = curve else {
        return Ok(curve.clone());
    };
    let mut points = nurbs.control_points().as_slice().to_vec();
    let index = if at_start { 0 } else { points.len() - 1 };
    let weight = points[index].weight();
    points[index] = HPoint::from_cartesian(point, weight);
    Ok(Curve::Nurbs(NurbsCurve::new(
        nurbs.degree(),
        ControlPolygon::new(points)?,
        nurbs.knots().clone(),
    )?))
}

/// Applies a parameter-space correction to one end of a fitted pcurve.
///
/// An analytic pcurve is exact wherever its curve is, so it is left alone for
/// the same reason its curve is.
fn snapped_pcurve(
    curve: &Curve2,
    at_start: bool,
    correction: Vector2<f64>,
) -> Result<Curve2, NurbsError> {
    if correction.norm() == 0.0 {
        return Ok(curve.clone());
    }
    match curve {
        Curve2::Line(line) => {
            let (mut start, mut end) = (line.point_at(0.0), line.point_at(1.0));
            *(if at_start { &mut start } else { &mut end }) += correction;
            Ok(Curve2::Line(Line2::new(start, end)))
        }
        Curve2::Nurbs(nurbs) => {
            let mut points = nurbs.control_points().as_slice().to_vec();
            let index = if at_start { 0 } else { points.len() - 1 };
            let weight = points[index].weight();
            let moved: Point2 = points[index].to_cartesian() + correction;
            points[index] = HPoint2::from_cartesian(moved, weight);
            Ok(Curve2::Nurbs(NurbsCurve2::new(
                nurbs.degree(),
                ControlPolygon2::new(points)?,
                nurbs.knots().clone(),
            )?))
        }
        Curve2::Circle(_) | Curve2::Ellipse(_) => Ok(curve.clone()),
    }
}
