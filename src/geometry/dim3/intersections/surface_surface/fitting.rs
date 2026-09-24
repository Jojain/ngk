use super::super::IntersectionOptions;
use super::simplification::{recognize_curve_3d, simplify_curve_2d};
use super::tracer::TraceState;
use crate::geometry::counters::count_branch_fit;
use crate::geometry::nurbs::basis::basis_functions;
use crate::geometry::parameter::Fraction;
use crate::geometry::{
    ControlPolygon, ControlPolygon2, Curve, Curve2, Degree, IntersectionError, IntersectionQuality,
    Interval, KnotVector, NurbsCurve, NurbsCurve2, Point2, Point3, Surface,
    SurfaceIntersectionBranch, SurfaceIntersectionBranchKind, SurfaceIntersectionPointKind,
    SurfacePeriodicity, TrimmedCurve, TrimmedCurve2,
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
    count_branch_fit();
    canonicalize_states(&mut states, closed);
    // The trace is ordered and finely stepped, so each state's parameters
    // start the next one's projection. A state lies on both surfaces within
    // its residual, which is how close a projection must land to be its own.
    let mut previous: Option<(Point2, Point2)> = None;
    for state in &mut states {
        let tolerance = state.residual.max(options.linear_tolerance);
        let (uv_a, uv_b) = match previous {
            Some((hint_a, hint_b)) => (
                a.param_near(state.point, hint_a, tolerance)?,
                b.param_near(state.point, hint_b, tolerance)?,
            ),
            None => (a.param_at(state.point)?, b.param_at(state.point)?),
        };
        previous = Some((uv_a, uv_b));
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
    let curve_interval = analytical
        .as_ref()
        .map(|curve| curve.interval)
        .unwrap_or(Interval::new(0.0, 1.0));
    let fitted = if closed {
        SynchronizedNurbsFit {
            curve_3d: NurbsCurve::interpolate_with_parameters(&points, &parameters)?,
            pcurve_a: interpolate_closed_pcurve(&uv_a, &parameters)?,
            pcurve_b: interpolate_closed_pcurve(&uv_b, &parameters)?,
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
            TrimmedCurve::new(
                Curve::Nurbs(fitted.curve_3d.clone()),
                Interval::new(0.0, 1.0),
            ),
            TrimmedCurve2::whole(Curve2::Nurbs(fitted.pcurve_a.clone())),
            TrimmedCurve2::whole(Curve2::Nurbs(fitted.pcurve_b.clone())),
        )
    };
    let (curve_3d, pcurve_a, pcurve_b) = if options.simplify_curves {
        let proposed_curve_3d = TrimmedCurve::new(
            analytical
                .map(|curve| curve.curve)
                .unwrap_or_else(|| Curve::Nurbs(fitted.curve_3d.clone())),
            curve_interval,
        );
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

/// Interpolates a closed branch's pcurve so its seam carries no end condition.
///
/// A closed branch returns to its own start in space, but its parameter samples
/// come back a period away: the trace leaves the domain through one edge and
/// re-enters through the other, and unwrapping keeps that drift rather than
/// folding it. Interpolated as an open curve, the seam would be fitted with a
/// one-sided tangent while the 3D curve -- which does close, and gets the
/// wrapped one -- is fitted with the true one, and the triple would stop
/// agreeing over exactly the first span. Subtracting the drift closes the
/// samples; adding it back afterwards is exact, a cubic reproducing a function
/// linear in the parameter at its Greville abscissae.
///
/// A pcurve that already closes -- the other surface's, wherever the branch
/// crosses no seam of its own -- has no drift, and this is the plain
/// interpolation.
fn interpolate_closed_pcurve(
    uv: &[Point2],
    parameters: &[f64],
) -> Result<NurbsCurve2, IntersectionError> {
    let (Some(first), Some(last)) = (uv.first(), uv.last()) else {
        return Err(
            crate::geometry::NurbsError::InsufficientInterpolationPoints {
                minimum: 2,
                got: uv.len(),
            }
            .into(),
        );
    };
    let drift = last - first;
    let closed = uv
        .iter()
        .zip(parameters)
        .map(|(point, parameter)| point - drift * *parameter)
        .collect::<Vec<_>>();
    let curve = NurbsCurve2::interpolate_with_parameters(&closed, parameters)?;
    let degree = curve.degree();
    let knots = curve.knots().clone();
    let weights = curve
        .control_points()
        .as_slice()
        .iter()
        .map(|point| point.weight())
        .collect::<Vec<_>>();
    let restored = curve
        .control_points()
        .as_slice()
        .iter()
        .enumerate()
        .map(|(index, point)| point.to_cartesian() + drift * greville(&knots, degree, index))
        .collect();
    Ok(NurbsCurve2::new(
        degree,
        ControlPolygon2::from_cartesian(restored, &weights)?,
        knots,
    )?)
}

/// The parameter a control point pulls its curve towards.
///
/// Averaging the `degree` knots after a control point's own index is the
/// standard abscissa, and it is what makes a spline reproduce a linear function
/// exactly rather than approximately.
fn greville(knots: &KnotVector, degree: Degree, control: usize) -> f64 {
    let degree = degree.get();
    (1..=degree)
        .map(|offset| knots.get(control + offset))
        .sum::<f64>()
        / degree as f64
}

/// Approximates one open walking line with a compact synchronized spline triple.
///
/// Control count grows only until the 3D curve and both pcurves jointly meet the
/// surface-intersection fit tolerance. Endpoints remain exact constraints.
///
/// Every round carries the breaks the two surfaces impose and refines only
/// between them, and the best round wins rather than the last: a control count
/// dense enough to leave a knot span without a sample has no unique least
/// squares solution, and the one solved for there can be worse than the
/// coarser fit before it.
fn approximate_open_branch(
    a: &Surface,
    b: &Surface,
    samples: BranchSamples<'_>,
    options: IntersectionOptions,
) -> Result<SynchronizedNurbsFit, IntersectionError> {
    let maximum = samples.points.len().min(MAX_FIT_CONTROL_POINTS);
    let degree = Degree::new(3.min(maximum.saturating_sub(1)).max(1))?;
    let breaks = fit_breaks(a, b, &samples, degree);
    let mut control_count = samples.points.len().min(INITIAL_FIT_CONTROL_POINTS);
    let target_tolerance = options.fit_tolerance.min(options.residual_tolerance);
    let mut best: Option<(f64, SynchronizedNurbsFit)> = None;
    loop {
        let fitted = least_squares_synchronized(
            samples.points,
            samples.uv_a,
            samples.uv_b,
            samples.parameters,
            FitBasis {
                degree,
                control_count,
                breaks: &breaks,
            },
        )?;
        let error = validate_fit(
            a,
            b,
            &TrimmedCurve::new(
                Curve::Nurbs(fitted.curve_3d.clone()),
                Interval::new(0.0, 1.0),
            ),
            &TrimmedCurve2::whole(Curve2::Nurbs(fitted.pcurve_a.clone())),
            &TrimmedCurve2::whole(Curve2::Nurbs(fitted.pcurve_b.clone())),
            samples.states,
            samples.parameters,
        );
        if best.as_ref().is_none_or(|(lowest, _)| error < *lowest) {
            best = Some((error, fitted));
        }
        if error <= target_tolerance || control_count == maximum {
            return Ok(best.expect("a round has been recorded").1);
        }
        control_count = (control_count * 2).min(maximum);
    }
}

/// The basis one fitting round solves against.
struct FitBasis<'a> {
    degree: Degree,
    /// Control points the round asks for, the breaks' own knots included.
    control_count: usize,
    breaks: &'a [FitBreak],
}

/// A branch parameter the fit has to be free to lose smoothness at, and the
/// knot multiplicity that buys it.
struct FitBreak {
    parameter: f64,
    multiplicity: usize,
}

/// The branch parameters where the fitted triple has to be allowed a corner.
///
/// A circle written as rational arcs joins its pieces at a knot of full
/// multiplicity, so a surface skinned through one is geometrically smooth
/// across that knot line while its *parameterization* is only C0 there. A
/// pcurve crossing it has a corner however smooth the 3D curve is, and one
/// B-spline basis carrying no knot at that parameter approximates a corner to
/// first order whatever control count it is given -- the error halves when the
/// count doubles instead of dropping by sixteen, so such a branch never
/// reaches the fit tolerance and the Boolean that asked for it aborts.
/// Answering the surface's own break with a knot of the fit's degree gives the
/// basis that corner back.
fn fit_breaks(
    a: &Surface,
    b: &Surface,
    samples: &BranchSamples<'_>,
    degree: Degree,
) -> Vec<FitBreak> {
    let mut breaks = Vec::new();
    for (surface, uv) in [(a, samples.uv_a), (b, samples.uv_b)] {
        let (along_u, along_v) = parametric_breaks(surface);
        for (coordinates, values) in [
            (uv_coordinates(uv, 0), along_u),
            (uv_coordinates(uv, 1), along_v),
        ] {
            for (value, continuity) in values {
                let multiplicity = degree.get().saturating_sub(continuity);
                if multiplicity == 0 {
                    continue;
                }
                for parameter in crossings(&coordinates, samples.parameters, value) {
                    breaks.push(FitBreak {
                        parameter,
                        multiplicity,
                    });
                }
            }
        }
    }
    let mut merged = merge_breaks(breaks, samples.parameters);
    // No round can ask for more control points than there are samples to
    // determine them by, so a branch too short to carry every break keeps the
    // ones it has room for and is reported uncertified if that is not enough.
    while merged.iter().map(|item| item.multiplicity).sum::<usize>() + degree.get() + 1
        > samples.points.len()
    {
        merged.pop();
    }
    merged
}

/// One coordinate of a pcurve's samples, `0` for `u` and `1` for `v`.
fn uv_coordinates(uv: &[Point2], axis: usize) -> Vec<f64> {
    uv.iter()
        .map(|point| if axis == 0 { point.x } else { point.y })
        .collect()
}

/// Orders breaks and folds together the ones a single knot can serve.
///
/// Two surfaces can break at the same place, and two knots closer together
/// than the samples are leave a span with nothing to condition it, so
/// neighbouring breaks become one carrying the larger multiplicity. A break
/// within one sample of an end is dropped: a clamped fit already ends in a
/// knot of full multiplicity, so the corner is representable there already.
fn merge_breaks(mut breaks: Vec<FitBreak>, parameters: &[f64]) -> Vec<FitBreak> {
    let spacing = parameters
        .windows(2)
        .map(|window| window[1] - window[0])
        .fold(0.0_f64, f64::max)
        .max(f64::MIN_POSITIVE);
    breaks.retain(|item| item.parameter > spacing && item.parameter < 1.0 - spacing);
    breaks.sort_by(|first, second| first.parameter.total_cmp(&second.parameter));
    let mut merged: Vec<FitBreak> = Vec::with_capacity(breaks.len());
    for item in breaks {
        match merged.last_mut() {
            Some(last) if item.parameter - last.parameter <= spacing => {
                last.multiplicity = last.multiplicity.max(item.multiplicity);
            }
            _ => merged.push(item),
        }
    }
    merged
}

/// The branch parameters at which `values` crosses `target`.
fn crossings(values: &[f64], parameters: &[f64], target: f64) -> Vec<f64> {
    let mut found = Vec::new();
    for index in 1..values.len() {
        let (before, after) = (values[index - 1], values[index]);
        if (before < target) == (after < target) {
            continue;
        }
        let span = after - before;
        let fraction = if span == 0.0 {
            0.5
        } else {
            ((target - before) / span).clamp(0.0, 1.0)
        };
        found.push(parameters[index - 1] + fraction * (parameters[index] - parameters[index - 1]));
    }
    found
}

/// A knot value in a support's own parameter space, with the order of
/// continuity the support keeps across it.
type ParametricBreak = (f64, usize);

/// Where a surface's own parameterization stops being smooth, per direction.
///
/// A caller reads the reported continuity to ask for exactly the freedom the
/// surface loses. An analytic support reports nothing: its parameterization is
/// smooth over its whole domain.
fn parametric_breaks(surface: &Surface) -> (Vec<ParametricBreak>, Vec<ParametricBreak>) {
    match surface {
        Surface::Nurbs(nurbs) => (
            knot_breaks(nurbs.knots_u(), nurbs.degree_u()),
            knot_breaks(nurbs.knots_v(), nurbs.degree_v()),
        ),
        // Both walk their generating curve in `u` and are smooth in `v`.
        Surface::Ruled(ruled) => (curve_breaks(ruled.curve()), Vec::new()),
        Surface::Revolution(revolution) => (curve_breaks(revolution.curve()), Vec::new()),
        Surface::Plane(_)
        | Surface::Cylinder(_)
        | Surface::Sphere(_)
        | Surface::Cone(_)
        | Surface::Torus(_) => (Vec::new(), Vec::new()),
    }
}

/// The breaks a generating curve hands to the surface built on it.
fn curve_breaks(curve: &Curve) -> Vec<ParametricBreak> {
    match curve {
        Curve::Nurbs(nurbs) => knot_breaks(nurbs.knots(), nurbs.degree()),
        _ => Vec::new(),
    }
}

/// Interior knots of `knots`, each with the continuity a spline of `degree`
/// keeps across it.
///
/// A knot of multiplicity `m` in a degree-`p` spline leaves `C^(p - m)`, so a
/// knot of full multiplicity -- an arc join -- reports `0`: continuous, with a
/// corner.
fn knot_breaks(knots: &KnotVector, degree: Degree) -> Vec<ParametricBreak> {
    let p = degree.get();
    let values = knots.as_slice();
    let domain = knots.domain(degree);
    let mut breaks = Vec::new();
    let mut index = 0;
    while index < values.len() {
        let value = values[index];
        let mut multiplicity = 1;
        while index + multiplicity < values.len() && values[index + multiplicity] == value {
            multiplicity += 1;
        }
        if value > domain.start.value() && value < domain.end.value() {
            breaks.push((value, p.saturating_sub(multiplicity)));
        }
        index += multiplicity;
    }
    breaks
}

/// Builds the clamped knot vector one fitting round solves against, with the
/// control count it actually carries.
///
/// The breaks go in first and keep their multiplicity, so a round can end up
/// with more control points than it asked for; whatever is left over is spread
/// over the segments the breaks cut `[0, 1]` into, in proportion to each
/// segment's length. Refining therefore never walks a uniform knot onto a
/// break and leaves a span too short to condition.
fn fit_knots(basis: &FitBasis<'_>) -> (KnotVector, usize) {
    let p = basis.degree.get();
    let fixed = basis
        .breaks
        .iter()
        .map(|item| item.multiplicity)
        .sum::<usize>();
    let uniform = basis.control_count.saturating_sub(p + 1 + fixed);
    let mut bounds = Vec::with_capacity(basis.breaks.len() + 2);
    bounds.push(0.0);
    bounds.extend(basis.breaks.iter().map(|item| item.parameter));
    bounds.push(1.0);

    let mut interior = Vec::with_capacity(uniform + fixed);
    let mut placed = 0;
    for (segment, window) in bounds.windows(2).enumerate() {
        let (start, end) = (window[0], window[1]);
        // Each stretch takes what brings the running total to its end's share
        // of the whole. Rounding every stretch on its own instead lets the
        // rounding pile up: many equal stretches that each round up spend the
        // budget before the branch ends, and its last stretches get no knot
        // at all however many control points the round asks for.
        let share = if segment + 2 == bounds.len() {
            uniform - placed
        } else {
            ((end * uniform as f64).round() as usize).clamp(placed, uniform) - placed
        };
        placed += share;
        for step in 1..=share {
            interior.push(start + (end - start) * step as f64 / (share + 1) as f64);
        }
        if let Some(item) = basis.breaks.get(segment) {
            interior.extend(std::iter::repeat_n(item.parameter, item.multiplicity));
        }
    }

    let control_count = interior.len() + p + 1;
    let mut knots = vec![0.0; p + 1];
    knots.extend(interior);
    knots.extend(std::iter::repeat_n(1.0, p + 1));
    (
        KnotVector::new(knots).expect("knots are built in non-decreasing order"),
        control_count,
    )
}

/// Solves all seven synchronized coordinates against one shared B-spline basis.
fn least_squares_synchronized(
    points: &[Point3],
    uv_a: &[Point2],
    uv_b: &[Point2],
    parameters: &[f64],
    basis: FitBasis<'_>,
) -> Result<SynchronizedNurbsFit, IntersectionError> {
    let fixed = basis
        .breaks
        .iter()
        .map(|item| item.multiplicity)
        .sum::<usize>();
    // An interpolating round has one condition per sample and needs no least
    // squares -- but only where no break asks for a knot the averaged vector
    // interpolation builds does not carry.
    if basis.control_count == points.len() && fixed == 0 {
        return Ok(SynchronizedNurbsFit {
            curve_3d: NurbsCurve::interpolate_with_parameters(points, parameters)?,
            pcurve_a: NurbsCurve2::interpolate_with_parameters(uv_a, parameters)?,
            pcurve_b: NurbsCurve2::interpolate_with_parameters(uv_b, parameters)?,
        });
    }
    let degree = basis.degree;
    let (knots, control_count) = fit_knots(&basis);
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
    curve_3d: &TrimmedCurve,
    pcurve_a: &TrimmedCurve2,
    pcurve_b: &TrimmedCurve2,
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
        let point = curve_3d.point_at(Fraction::new(parameter));
        let uv_a = pcurve_a.point_at(Fraction::new(parameter));
        let uv_b = pcurve_b.point_at(Fraction::new(parameter));
        let point_a = a.point_at(uv_a.x, uv_a.y);
        let point_b = b.point_at(uv_b.x, uv_b.y);
        max_fit_error = max_fit_error
            .max((point_a - point_b).norm())
            .max((point - point_a).norm())
            .max((point - point_b).norm());
    }
    for (state, parameter) in states.iter().zip(parameters.iter().copied()) {
        max_fit_error =
            max_fit_error.max((curve_3d.point_at(Fraction::new(parameter)) - state.point).norm());
    }
    max_fit_error
}

#[cfg(test)]
mod tests {
    use super::{FitBasis, FitBreak, fit_knots};
    use crate::geometry::Degree;

    #[test]
    fn fit_knots_spread_the_free_knots_over_every_stretch_between_breaks() {
        // A branch across a skinned surface crosses one knot line per
        // section, evenly. Every stretch between two of them is as long as
        // the next, so each has to get its share of the free knots -- the
        // last one as much as the first.
        let breaks = (1..133)
            .map(|index| FitBreak {
                parameter: index as f64 / 133.0,
                multiplicity: 1,
            })
            .collect::<Vec<_>>();
        for control_count in [256, 512, 1024] {
            let (knots, _) = fit_knots(&FitBasis {
                degree: Degree::new(3).unwrap(),
                control_count,
                breaks: &breaks,
            });
            let knots = knots.as_slice();
            let widest = knots
                .windows(2)
                .map(|pair| pair[1] - pair[0])
                .fold(0.0_f64, f64::max);
            let interior = control_count - 4;
            assert!(
                widest <= 2.0 / interior as f64,
                "{control_count} control points leave a knot span {widest} wide"
            );
        }
    }
}
