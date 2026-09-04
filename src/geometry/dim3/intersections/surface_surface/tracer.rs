use nalgebra::{Matrix2, Matrix3, Matrix4, Vector2, Vector4};

use super::super::options::IntersectionOptions;
use crate::geometry::{
    Interval, NurbsSurface, Point2, Point3, SurfaceIntersectionIncompleteReason,
    SurfaceIntersectionPoint, SurfaceIntersectionPointKind,
};

#[derive(Debug, Clone, Copy)]
pub(super) struct TraceState {
    pub parameters: Vector4<f64>,
    pub point: Point3,
    pub residual: f64,
}

impl TraceState {
    /// Builds a synchronized state from the average of both surface evaluations.
    pub(super) fn new(a: &NurbsSurface, b: &NurbsSurface, parameters: Vector4<f64>) -> Self {
        let point_a = a.point_at(parameters.x, parameters.y);
        let point_b = b.point_at(parameters.z, parameters.w);
        Self {
            parameters,
            point: Point3::from((point_a.coords + point_b.coords) * 0.5),
            residual: (point_a - point_b).norm(),
        }
    }

    /// Converts the internal state into the public corrected sample.
    pub(super) fn sample(self, kind: SurfaceIntersectionPointKind) -> SurfaceIntersectionPoint {
        SurfaceIntersectionPoint {
            point: self.point,
            uv_a: Point2::new(self.parameters.x, self.parameters.y),
            uv_b: Point2::new(self.parameters.z, self.parameters.w),
            kind,
            residual: self.residual,
        }
    }
}

pub(super) struct TraceOutcome {
    pub states: Vec<TraceState>,
    pub closed: bool,
    pub incomplete_reasons: Vec<SurfaceIntersectionIncompleteReason>,
}

enum DirectionStop {
    Boundary,
    Closed,
    MinimumStep,
    Budget,
}

/// Traces a regular one-dimensional solution component in both directions.
pub(super) fn trace_from_seed(
    a: &NurbsSurface,
    b: &NurbsSurface,
    seed: TraceState,
    options: IntersectionOptions,
) -> Option<TraceOutcome> {
    let tangent = parameter_tangent(a, b, seed.parameters, options)?;
    let (mut backward, backward_stop) = trace_direction(a, b, seed, -tangent, options);
    let (forward, forward_stop) = trace_direction(a, b, seed, tangent, options);

    backward.reverse();
    if backward.last().is_some_and(|state| {
        parameter_distance(state.parameters, seed.parameters) <= options.parameter_tolerance
    }) {
        backward.pop();
    }
    backward.extend(forward);
    dedup_consecutive(&mut backward, options);

    let closed = matches!(backward_stop, DirectionStop::Closed)
        || matches!(forward_stop, DirectionStop::Closed)
        || backward
            .first()
            .zip(backward.last())
            .is_some_and(|(start, end)| {
                backward.len() > 3
                    && (start.point - end.point).norm() <= options.linear_tolerance * 10.0
            });
    let mut incomplete_reasons = Vec::new();
    for stop in [backward_stop, forward_stop] {
        match stop {
            DirectionStop::MinimumStep => push_reason(
                &mut incomplete_reasons,
                SurfaceIntersectionIncompleteReason::MinimumTraceStepReached,
            ),
            DirectionStop::Budget => push_reason(
                &mut incomplete_reasons,
                SurfaceIntersectionIncompleteReason::TraceBudgetExhausted,
            ),
            DirectionStop::Boundary | DirectionStop::Closed => {}
        }
    }

    Some(TraceOutcome {
        states: backward,
        closed,
        incomplete_reasons,
    })
}

fn trace_direction(
    a: &NurbsSurface,
    b: &NurbsSurface,
    seed: TraceState,
    initial_tangent: Vector4<f64>,
    options: IntersectionOptions,
) -> (Vec<TraceState>, DirectionStop) {
    let domains = domains(a, b);
    let mut states = vec![seed];
    let mut tangent = initial_tangent;
    let mut step = options.max_trace_step;

    for _ in 0..options.max_trace_steps {
        let current = *states.last().expect("a trace always contains its seed");
        let boundary_step = distance_to_boundary(current.parameters, tangent, domains, options);
        if boundary_step <= options.parameter_tolerance {
            return (states, DirectionStop::Boundary);
        }
        let trial_step = step.min(boundary_step);
        let reaches_boundary = boundary_step <= step;
        let mut accepted = None;
        let mut correction_step = trial_step;
        while correction_step >= options.min_trace_step
            || reaches_boundary && correction_step > options.parameter_tolerance
        {
            let predicted =
                clamp_parameters(current.parameters + tangent * correction_step, domains);
            let corrected =
                correct_state(a, b, predicted, tangent, domains, options).or_else(|| {
                    reaches_boundary
                        .then(|| correct_boundary_state(a, b, predicted, domains, options))?
                });
            if let Some(corrected) = corrected
                && corrected.residual <= options.residual_tolerance
            {
                accepted = Some(corrected);
                break;
            }
            correction_step *= 0.5;
        }

        let Some(next) = accepted else {
            return (states, DirectionStop::MinimumStep);
        };
        if (next.point - current.point).norm() <= options.linear_tolerance {
            return (
                states,
                if reaches_boundary {
                    DirectionStop::Boundary
                } else {
                    DirectionStop::MinimumStep
                },
            );
        }
        states.push(next);

        let Some(mut next_tangent) = parameter_tangent(a, b, next.parameters, options) else {
            return (states, DirectionStop::MinimumStep);
        };
        if next_tangent.dot(&tangent) < 0.0 {
            next_tangent = -next_tangent;
        }
        tangent = next_tangent;

        if states.len() > 8
            && (next.point - seed.point).norm() <= options.linear_tolerance * 10.0
            && tangent.dot(&initial_tangent) > 0.9
        {
            return (states, DirectionStop::Closed);
        }
        if reaches_boundary {
            return (states, DirectionStop::Boundary);
        }

        step = if correction_step < trial_step {
            correction_step
        } else {
            (step * 1.25).min(options.max_trace_step)
        };
    }

    (states, DirectionStop::Budget)
}

/// Corrects an event on a parameter-domain boundary while holding that boundary coordinate fixed.
fn correct_boundary_state(
    a: &NurbsSurface,
    b: &NurbsSurface,
    predicted: Vector4<f64>,
    domains: [Interval; 4],
    options: IntersectionOptions,
) -> Option<TraceState> {
    let fixed = (0..4).min_by(|&left, &right| {
        distance_from_interval_end(predicted[left], domains[left]).total_cmp(
            &distance_from_interval_end(predicted[right], domains[right]),
        )
    })?;
    let mut parameters = predicted;
    parameters[fixed] = if (parameters[fixed] - domains[fixed].start).abs()
        <= (parameters[fixed] - domains[fixed].end).abs()
    {
        domains[fixed].start
    } else {
        domains[fixed].end
    };

    for _ in 0..options.newton_max_iterations {
        let point_a = a.point_at(parameters.x, parameters.y);
        let point_b = b.point_at(parameters.z, parameters.w);
        let residual = point_a - point_b;
        if residual.norm() <= options.residual_tolerance {
            return Some(TraceState::new(a, b, parameters));
        }
        let (a_u, a_v) = a.derivatives_uv(parameters.x, parameters.y);
        let (b_u, b_v) = b.derivatives_uv(parameters.z, parameters.w);
        let columns = [a_u, a_v, -b_u, -b_v];
        let free = (0..4).filter(|index| *index != fixed).collect::<Vec<_>>();
        let jacobian =
            Matrix3::from_columns(&[columns[free[0]], columns[free[1]], columns[free[2]]]);
        let delta = jacobian.lu().solve(&(-residual))?;
        let current_norm = residual.norm();
        let mut damping = 1.0;
        let mut improved = None;
        while damping >= 1.0 / 128.0 {
            let mut trial = parameters;
            for (offset, index) in free.iter().copied().enumerate() {
                trial[index] += delta[offset] * damping;
            }
            trial = clamp_parameters(trial, domains);
            trial[fixed] = parameters[fixed];
            let trial_norm = (a.point_at(trial.x, trial.y) - b.point_at(trial.z, trial.w)).norm();
            if trial_norm < current_norm {
                improved = Some(trial);
                break;
            }
            damping *= 0.5;
        }
        parameters = improved?;
        if (delta * damping).norm() <= options.parameter_tolerance {
            break;
        }
    }
    let state = TraceState::new(a, b, parameters);
    (state.residual <= options.residual_tolerance).then_some(state)
}

/// Computes a four-parameter tangent whose mapped 3D derivatives agree.
fn parameter_tangent(
    a: &NurbsSurface,
    b: &NurbsSurface,
    parameters: Vector4<f64>,
    options: IntersectionOptions,
) -> Option<Vector4<f64>> {
    let (a_u, a_v) = a.derivatives_uv(parameters.x, parameters.y);
    let (b_u, b_v) = b.derivatives_uv(parameters.z, parameters.w);
    let normal_a = a_u.cross(&a_v);
    let normal_b = b_u.cross(&b_v);
    let tangent_3d = normal_a.cross(&normal_b);
    let scale = normal_a.norm() * normal_b.norm();
    if scale == 0.0 || tangent_3d.norm() <= options.angular_tolerance * scale {
        return None;
    }
    let tangent_3d = tangent_3d.normalize();
    let a_parameters = map_tangent(a_u, a_v, tangent_3d)?;
    let b_parameters = map_tangent(b_u, b_v, tangent_3d)?;
    let tangent = Vector4::new(
        a_parameters.x,
        a_parameters.y,
        b_parameters.x,
        b_parameters.y,
    );
    (tangent.norm() > options.parameter_tolerance).then(|| tangent.normalize())
}

fn map_tangent(
    derivative_u: nalgebra::Vector3<f64>,
    derivative_v: nalgebra::Vector3<f64>,
    tangent: nalgebra::Vector3<f64>,
) -> Option<Vector2<f64>> {
    Matrix2::new(
        derivative_u.dot(&derivative_u),
        derivative_u.dot(&derivative_v),
        derivative_u.dot(&derivative_v),
        derivative_v.dot(&derivative_v),
    )
    .lu()
    .solve(&Vector2::new(
        derivative_u.dot(&tangent),
        derivative_v.dot(&tangent),
    ))
}

/// Corrects a prediction with the three surface equations and one normal-plane equation.
fn correct_state(
    a: &NurbsSurface,
    b: &NurbsSurface,
    predicted: Vector4<f64>,
    tangent: Vector4<f64>,
    domains: [Interval; 4],
    options: IntersectionOptions,
) -> Option<TraceState> {
    let mut parameters = predicted;
    for _ in 0..options.newton_max_iterations {
        let point_a = a.point_at(parameters.x, parameters.y);
        let point_b = b.point_at(parameters.z, parameters.w);
        let residual = point_a - point_b;
        let constraint = (parameters - predicted).dot(&tangent);
        if residual.norm() <= options.residual_tolerance
            && constraint.abs() <= options.parameter_tolerance
        {
            return Some(TraceState::new(a, b, parameters));
        }

        let (a_u, a_v) = a.derivatives_uv(parameters.x, parameters.y);
        let (b_u, b_v) = b.derivatives_uv(parameters.z, parameters.w);
        let jacobian = Matrix4::new(
            a_u.x, a_v.x, -b_u.x, -b_v.x, a_u.y, a_v.y, -b_u.y, -b_v.y, a_u.z, a_v.z, -b_u.z,
            -b_v.z, tangent.x, tangent.y, tangent.z, tangent.w,
        );
        let rhs = Vector4::new(-residual.x, -residual.y, -residual.z, -constraint);
        let delta = jacobian.lu().solve(&rhs)?;
        let current_objective = residual.norm() + constraint.abs();
        let mut damping = 1.0;
        let mut improved = None;
        while damping >= 1.0 / 128.0 {
            let trial = clamp_parameters(parameters + delta * damping, domains);
            let trial_residual =
                (a.point_at(trial.x, trial.y) - b.point_at(trial.z, trial.w)).norm();
            let trial_constraint = (trial - predicted).dot(&tangent).abs();
            if trial_residual + trial_constraint < current_objective {
                improved = Some(trial);
                break;
            }
            damping *= 0.5;
        }
        parameters = improved?;
        if (delta * damping).norm() <= options.parameter_tolerance {
            break;
        }
    }

    let state = TraceState::new(a, b, parameters);
    (state.residual <= options.residual_tolerance).then_some(state)
}

fn domains(a: &NurbsSurface, b: &NurbsSurface) -> [Interval; 4] {
    [a.domain_u(), a.domain_v(), b.domain_u(), b.domain_v()]
}

fn distance_to_boundary(
    parameters: Vector4<f64>,
    direction: Vector4<f64>,
    domains: [Interval; 4],
    options: IntersectionOptions,
) -> f64 {
    let mut distance = f64::INFINITY;
    for index in 0..4 {
        let component = direction[index];
        if component > options.parameter_tolerance {
            distance = distance.min((domains[index].end - parameters[index]) / component);
        } else if component < -options.parameter_tolerance {
            distance = distance.min((domains[index].start - parameters[index]) / component);
        }
    }
    distance.max(0.0)
}

fn clamp_parameters(parameters: Vector4<f64>, domains: [Interval; 4]) -> Vector4<f64> {
    Vector4::new(
        parameters.x.clamp(domains[0].start, domains[0].end),
        parameters.y.clamp(domains[1].start, domains[1].end),
        parameters.z.clamp(domains[2].start, domains[2].end),
        parameters.w.clamp(domains[3].start, domains[3].end),
    )
}

fn distance_from_interval_end(value: f64, domain: Interval) -> f64 {
    (value - domain.start).abs().min((value - domain.end).abs())
}

fn parameter_distance(a: Vector4<f64>, b: Vector4<f64>) -> f64 {
    (a - b).norm()
}

fn dedup_consecutive(states: &mut Vec<TraceState>, options: IntersectionOptions) {
    states.dedup_by(|a, b| {
        parameter_distance(a.parameters, b.parameters) <= options.parameter_tolerance
            || (a.point - b.point).norm() <= options.linear_tolerance
    });
}

fn push_reason(
    reasons: &mut Vec<SurfaceIntersectionIncompleteReason>,
    reason: SurfaceIntersectionIncompleteReason,
) {
    if !reasons.contains(&reason) {
        reasons.push(reason);
    }
}
