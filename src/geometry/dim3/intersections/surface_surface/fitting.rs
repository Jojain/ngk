use super::super::IntersectionOptions;
use super::simplification::{recognize_curve_3d, simplify_curve_2d};
use super::tracer::TraceState;
use crate::geometry::{
    Curve, Curve2, IntersectionError, IntersectionQuality, NurbsCurve, NurbsCurve2, NurbsSurface,
    Point2, SurfaceIntersectionBranch, SurfaceIntersectionBranchKind, SurfaceIntersectionPointKind,
};

/// Fits synchronized 3D and parameter-space curves using one chord-length parameterization.
pub(super) fn fit_branch(
    a: &NurbsSurface,
    b: &NurbsSurface,
    mut states: Vec<TraceState>,
    closed: bool,
    options: IntersectionOptions,
) -> Result<SurfaceIntersectionBranch, IntersectionError> {
    canonicalize_states(&mut states, closed);
    let points = states.iter().map(|state| state.point).collect::<Vec<_>>();
    let uv_a = states
        .iter()
        .map(|state| Point2::new(state.parameters.x, state.parameters.y))
        .collect::<Vec<_>>();
    let uv_b = states
        .iter()
        .map(|state| Point2::new(state.parameters.z, state.parameters.w))
        .collect::<Vec<_>>();
    let chord_parameters = NurbsCurve::chord_length_parameters(&points)?;
    let chord_curve_3d = NurbsCurve::interpolate_with_parameters(&points, &chord_parameters)?;
    let chord_pcurve_a = NurbsCurve2::interpolate_with_parameters(&uv_a, &chord_parameters)?;
    let chord_pcurve_b = NurbsCurve2::interpolate_with_parameters(&uv_b, &chord_parameters)?;
    let nurbs_fallback = || {
        (
            Curve::Nurbs(chord_curve_3d.clone()),
            Curve2::Nurbs(chord_pcurve_a.clone()),
            Curve2::Nurbs(chord_pcurve_b.clone()),
            chord_parameters.clone(),
        )
    };
    let (curve_3d, pcurve_a, pcurve_b, parameters) = if options.simplify_curves {
        let analytical = recognize_curve_3d(&states, closed, options.fit_tolerance);
        let parameters = analytical
            .as_ref()
            .map(|curve| curve.parameters.clone())
            .unwrap_or_else(|| chord_parameters.clone());
        let fitted_curve_3d = NurbsCurve::interpolate_with_parameters(&points, &parameters)?;
        let fitted_pcurve_a = NurbsCurve2::interpolate_with_parameters(&uv_a, &parameters)?;
        let fitted_pcurve_b = NurbsCurve2::interpolate_with_parameters(&uv_b, &parameters)?;
        let proposed_curve_3d = analytical
            .map(|curve| curve.curve)
            .unwrap_or(Curve::Nurbs(fitted_curve_3d));
        let proposed_pcurve_a =
            simplify_curve_2d(fitted_pcurve_a, &uv_a, &parameters, options.fit_tolerance);
        let proposed_pcurve_b =
            simplify_curve_2d(fitted_pcurve_b, &uv_b, &parameters, options.fit_tolerance);
        let proposed_error = validate_fit(
            a,
            b,
            &proposed_curve_3d,
            &proposed_pcurve_a,
            &proposed_pcurve_b,
            &states,
            &parameters,
        );
        if proposed_error <= options.fit_tolerance {
            (
                proposed_curve_3d,
                proposed_pcurve_a,
                proposed_pcurve_b,
                parameters,
            )
        } else {
            nurbs_fallback()
        }
    } else {
        nurbs_fallback()
    };
    let max_residual = states
        .iter()
        .map(|state| state.residual)
        .fold(0.0_f64, f64::max);
    let max_fit_error = validate_fit(a, b, &curve_3d, &pcurve_a, &pcurve_b, &states, &parameters);
    let certified =
        max_residual <= options.residual_tolerance && max_fit_error <= options.fit_tolerance;
    let samples = states
        .into_iter()
        .map(|state| state.sample(SurfaceIntersectionPointKind::Transverse))
        .collect();

    Ok(SurfaceIntersectionBranch {
        curve_3d,
        pcurve_a,
        pcurve_b,
        samples,
        closed,
        kind: SurfaceIntersectionBranchKind::Transverse,
        quality: IntersectionQuality {
            max_residual,
            max_fit_error,
            certified,
        },
    })
}

fn canonicalize_states(states: &mut [TraceState], closed: bool) {
    if closed {
        if states
            .first()
            .zip(states.last())
            .is_some_and(|(start, end)| state_key(*start) > state_key(*end))
        {
            states.reverse();
        }
        return;
    }
    if states
        .first()
        .zip(states.last())
        .is_some_and(|(start, end)| state_key(*start) > state_key(*end))
    {
        states.reverse();
    }
}

fn state_key(state: TraceState) -> f64 {
    state.parameters.x * 1.0e9
        + state.parameters.y * 1.0e6
        + state.parameters.z * 1.0e3
        + state.parameters.w
}

fn validate_fit(
    a: &NurbsSurface,
    b: &NurbsSurface,
    curve_3d: &Curve,
    pcurve_a: &Curve2,
    pcurve_b: &Curve2,
    states: &[TraceState],
    parameters: &[f64],
) -> f64 {
    let mut checkpoints = parameters.to_vec();
    checkpoints.extend(
        parameters
            .windows(2)
            .map(|window| 0.5 * (window[0] + window[1])),
    );
    let mut max_fit_error: f64 = 0.0;
    for parameter in checkpoints {
        let point = curve_3d.point_at(parameter);
        let uv_a = pcurve_a.point_at(parameter);
        let uv_b = pcurve_b.point_at(parameter);
        let point_a = a.point_at(uv_a.x, uv_a.y);
        let point_b = b.point_at(uv_b.x, uv_b.y);
        max_fit_error = max_fit_error
            .max((point_a - point_b).norm())
            .max((point - point_a).norm())
            .max((point - point_b).norm());
    }
    for (state, parameter) in states.iter().zip(parameters.iter().copied()) {
        max_fit_error = max_fit_error.max((curve_3d.point_at(parameter) - state.point).norm());
    }
    max_fit_error
}
