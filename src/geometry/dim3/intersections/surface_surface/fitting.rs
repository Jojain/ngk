use super::super::IntersectionOptions;
use super::simplification::{recognize_curve_3d, simplify_curve_2d};
use super::tracer::TraceState;
use crate::geometry::nurbs::basis::basis_functions;
use crate::geometry::{
    ControlPolygon, ControlPolygon2, Curve, Curve2, Degree, IntersectionError, IntersectionQuality,
    KnotVector, NurbsCurve, NurbsCurve2, Point2, Point3, Surface, SurfaceIntersectionBranch,
    SurfaceIntersectionBranchKind, SurfaceIntersectionPointKind, SurfacePeriodicity,
};
use nalgebra::DMatrix;

const INITIAL_FIT_CONTROL_POINTS: usize = 8;
const MAX_FIT_CONTROL_POINTS: usize = 256;

struct SynchronizedNurbsFit {
    curve_3d: NurbsCurve,
    pcurve_a: NurbsCurve2,
    pcurve_b: NurbsCurve2,
}

struct BranchSamples<'a> {
    points: &'a [Point3],
    uv_a: &'a [Point2],
    uv_b: &'a [Point2],
    states: &'a [TraceState],
    parameters: &'a [f64],
}

/// Fits synchronized 3D and parameter-space curves using one chord-length parameterization.
pub(super) fn fit_branch(
    a: &Surface,
    b: &Surface,
    mut states: Vec<TraceState>,
    closed: bool,
    options: IntersectionOptions,
) -> Result<SurfaceIntersectionBranch, IntersectionError> {
    canonicalize_states(&mut states, closed);
    for state in &mut states {
        let uv_a = a.closest_parameter(state.point)?;
        let uv_b = b.closest_parameter(state.point)?;
        state.parameters.x = uv_a.x;
        state.parameters.y = uv_a.y;
        state.parameters.z = uv_b.x;
        state.parameters.w = uv_b.y;
    }
    unwrap_surface_parameters(&mut states, a.periodicity(), 0, 1);
    unwrap_surface_parameters(&mut states, b.periodicity(), 2, 3);
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
    let analytical = options
        .simplify_curves
        .then(|| recognize_curve_3d(&states, closed, options.fit_tolerance))
        .flatten();
    let parameters = analytical
        .as_ref()
        .map(|curve| curve.parameters.clone())
        .unwrap_or(chord_parameters);
    let fitted = if closed {
        SynchronizedNurbsFit {
            curve_3d: NurbsCurve::interpolate_with_parameters(&points, &parameters)?,
            pcurve_a: NurbsCurve2::interpolate_with_parameters(&uv_a, &parameters)?,
            pcurve_b: NurbsCurve2::interpolate_with_parameters(&uv_b, &parameters)?,
        }
    } else {
        approximate_open_branch(
            a,
            b,
            BranchSamples {
                points: &points,
                uv_a: &uv_a,
                uv_b: &uv_b,
                states: &states,
                parameters: &parameters,
            },
            options,
        )?
    };
    let nurbs_fallback = || {
        (
            Curve::Nurbs(fitted.curve_3d.clone()),
            Curve2::Nurbs(fitted.pcurve_a.clone()),
            Curve2::Nurbs(fitted.pcurve_b.clone()),
        )
    };
    let (curve_3d, pcurve_a, pcurve_b) = if options.simplify_curves {
        let proposed_curve_3d = analytical
            .map(|curve| curve.curve)
            .unwrap_or_else(|| Curve::Nurbs(fitted.curve_3d.clone()));
        let proposed_pcurve_a = simplify_curve_2d(
            fitted.pcurve_a.clone(),
            &uv_a,
            &parameters,
            options.fit_tolerance,
        );
        let proposed_pcurve_b = simplify_curve_2d(
            fitted.pcurve_b.clone(),
            &uv_b,
            &parameters,
            options.fit_tolerance,
        );
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
            (proposed_curve_3d, proposed_pcurve_a, proposed_pcurve_b)
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

/// Approximates one open walking line with a compact synchronized spline triple.
///
/// Control count grows only until the 3D curve and both pcurves jointly meet the
/// surface-intersection fit tolerance. Endpoints remain exact constraints.
fn approximate_open_branch(
    a: &Surface,
    b: &Surface,
    samples: BranchSamples<'_>,
    options: IntersectionOptions,
) -> Result<SynchronizedNurbsFit, IntersectionError> {
    let maximum = samples.points.len().min(MAX_FIT_CONTROL_POINTS);
    let mut control_count = samples.points.len().min(INITIAL_FIT_CONTROL_POINTS);
    let target_tolerance = options.fit_tolerance.min(options.residual_tolerance);
    loop {
        let fitted = least_squares_synchronized(
            samples.points,
            samples.uv_a,
            samples.uv_b,
            samples.parameters,
            control_count,
        )?;
        let error = validate_fit(
            a,
            b,
            &Curve::Nurbs(fitted.curve_3d.clone()),
            &Curve2::Nurbs(fitted.pcurve_a.clone()),
            &Curve2::Nurbs(fitted.pcurve_b.clone()),
            samples.states,
            samples.parameters,
        );
        if error <= target_tolerance || control_count == maximum {
            return Ok(fitted);
        }
        control_count = (control_count * 2).min(maximum);
    }
}

/// Solves all seven synchronized coordinates against one shared B-spline basis.
fn least_squares_synchronized(
    points: &[Point3],
    uv_a: &[Point2],
    uv_b: &[Point2],
    parameters: &[f64],
    control_count: usize,
) -> Result<SynchronizedNurbsFit, IntersectionError> {
    if control_count == points.len() {
        return Ok(SynchronizedNurbsFit {
            curve_3d: NurbsCurve::interpolate_with_parameters(points, parameters)?,
            pcurve_a: NurbsCurve2::interpolate_with_parameters(uv_a, parameters)?,
            pcurve_b: NurbsCurve2::interpolate_with_parameters(uv_b, parameters)?,
        });
    }
    let degree = Degree::new(3.min(control_count - 1))?;
    let knots = KnotVector::uniform_clamped(control_count, degree);
    let internal_count = control_count - 2;
    let mut coefficients = DMatrix::zeros(points.len(), internal_count);
    let mut right_hand_side = DMatrix::zeros(points.len(), 7);
    let endpoints = [
        synchronized_coordinates(points[0], uv_a[0], uv_b[0]),
        synchronized_coordinates(
            *points
                .last()
                .expect("an intersection branch has an endpoint"),
            *uv_a
                .last()
                .expect("an intersection branch has a pcurve endpoint"),
            *uv_b
                .last()
                .expect("an intersection branch has a pcurve endpoint"),
        ),
    ];
    for (row, parameter) in parameters.iter().copied().enumerate() {
        let sample = synchronized_coordinates(points[row], uv_a[row], uv_b[row]);
        for coordinate in 0..7 {
            right_hand_side[(row, coordinate)] = sample[coordinate];
        }
        let span = knots.find_span(control_count - 1, degree, parameter);
        let basis = basis_functions(span, parameter, degree, &knots);
        for (offset, value) in basis.into_iter().enumerate() {
            let control = span - degree.get() + offset;
            match control {
                0 => subtract_endpoint(&mut right_hand_side, row, value, &endpoints[0]),
                control if control + 1 == control_count => {
                    subtract_endpoint(&mut right_hand_side, row, value, &endpoints[1]);
                }
                control => coefficients[(row, control - 1)] = value,
            }
        }
    }
    let transpose = coefficients.transpose();
    let normal = &transpose * &coefficients;
    let projected = transpose * right_hand_side;
    let internal = normal
        .lu()
        .solve(&projected)
        .ok_or(crate::geometry::NurbsError::SingularInterpolationSystem)?;
    let coordinate = |control: usize, axis: usize| {
        if control == 0 {
            endpoints[0][axis]
        } else if control + 1 == control_count {
            endpoints[1][axis]
        } else {
            internal[(control - 1, axis)]
        }
    };
    let points_3d = (0..control_count)
        .map(|control| {
            Point3::new(
                coordinate(control, 0),
                coordinate(control, 1),
                coordinate(control, 2),
            )
        })
        .collect();
    let points_a = (0..control_count)
        .map(|control| Point2::new(coordinate(control, 3), coordinate(control, 4)))
        .collect();
    let points_b = (0..control_count)
        .map(|control| Point2::new(coordinate(control, 5), coordinate(control, 6)))
        .collect();
    let weights = vec![1.0; control_count];
    Ok(SynchronizedNurbsFit {
        curve_3d: NurbsCurve::new(
            degree,
            ControlPolygon::from_cartesian(points_3d, &weights)?,
            knots.clone(),
        )?,
        pcurve_a: NurbsCurve2::new(
            degree,
            ControlPolygon2::from_cartesian(points_a, &weights)?,
            knots.clone(),
        )?,
        pcurve_b: NurbsCurve2::new(
            degree,
            ControlPolygon2::from_cartesian(points_b, &weights)?,
            knots,
        )?,
    })
}

fn synchronized_coordinates(point: Point3, uv_a: Point2, uv_b: Point2) -> [f64; 7] {
    [point.x, point.y, point.z, uv_a.x, uv_a.y, uv_b.x, uv_b.y]
}

fn subtract_endpoint(
    right_hand_side: &mut DMatrix<f64>,
    row: usize,
    basis: f64,
    endpoint: &[f64; 7],
) {
    for coordinate in 0..7 {
        right_hand_side[(row, coordinate)] -= basis * endpoint[coordinate];
    }
}

fn unwrap_surface_parameters(
    states: &mut [TraceState],
    periodicity: SurfacePeriodicity,
    u_index: usize,
    v_index: usize,
) {
    match periodicity {
        SurfacePeriodicity::None => {}
        SurfacePeriodicity::UPeriodic(period) => unwrap_parameter(states, u_index, period),
        SurfacePeriodicity::VPeriodic(period) => unwrap_parameter(states, v_index, period),
        SurfacePeriodicity::UVPeriodic(u_period, v_period) => {
            unwrap_parameter(states, u_index, u_period);
            unwrap_parameter(states, v_index, v_period);
        }
    }
}

fn unwrap_parameter(states: &mut [TraceState], index: usize, period: f64) {
    for current in 1..states.len() {
        let previous = states[current - 1].parameters[index];
        let value = &mut states[current].parameters[index];
        while *value + period * 0.5 < previous {
            *value += period;
        }
        while *value - period * 0.5 > previous {
            *value -= period;
        }
    }
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
    a: &Surface,
    b: &Surface,
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
    const GLOBAL_CHECKPOINTS: usize = 128;
    checkpoints
        .extend((0..=GLOBAL_CHECKPOINTS).map(|index| index as f64 / GLOBAL_CHECKPOINTS as f64));
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
