use std::f64::consts::{PI, TAU};

use nalgebra::{Matrix3, SymmetricEigen, Vector3};

use super::tracer::TraceState;
use crate::geometry::parameter::{Fraction, NativeParam};
use crate::geometry::{
    Circle, Circle2, Curve, Curve2, Interval, NurbsCurve2, Plane, Point2, Point3, TrimmedCurve2,
};

type Curve3Recognizer = fn(&[TraceState], bool, f64) -> Option<AnalyticalCurve3>;
type Curve2Recognizer = fn(&[Point2], &[f64], f64) -> Option<TrimmedCurve2>;

const CURVE_3_RECOGNIZERS: [Curve3Recognizer; 2] = [recognize_line_3d, recognize_circle_3d];
const CURVE_2_RECOGNIZERS: [Curve2Recognizer; 2] = [recognize_line_2d, recognize_circle_2d];

/// An analytical 3D curve together with its normalized natural sample parameters.
pub(super) struct AnalyticalCurve3 {
    pub curve: Curve,
    pub interval: Interval,
    pub parameters: Vec<f64>,
}

/// Tries registered 3D recognizers in increasing complexity.
///
/// A successful recognizer owns the common parameterization used to rebuild
/// both pcurves. Lines use projected distance and circles use unwrapped angle.
pub(super) fn recognize_curve_3d(
    states: &[TraceState],
    closed: bool,
    tolerance: f64,
) -> Option<AnalyticalCurve3> {
    CURVE_3_RECOGNIZERS
        .iter()
        .find_map(|recognizer| recognizer(states, closed, tolerance))
}

/// Tries registered 2D recognizers while preserving the shared branch parameter.
pub(super) fn simplify_curve_2d(
    fallback: NurbsCurve2,
    points: &[Point2],
    parameters: &[f64],
    tolerance: f64,
) -> TrimmedCurve2 {
    CURVE_2_RECOGNIZERS
        .iter()
        .find_map(|recognizer| recognizer(points, parameters, tolerance))
        .unwrap_or_else(|| {
            let domain = fallback.domain();
            TrimmedCurve2::new(Curve2::Nurbs(fallback), domain)
        })
}

fn recognize_line_3d(
    states: &[TraceState],
    closed: bool,
    tolerance: f64,
) -> Option<AnalyticalCurve3> {
    if closed {
        return None;
    }
    let start = states.first()?.point;
    let end = states.last()?.point;
    let direction = end - start;
    let length_squared = direction.norm_squared();
    if length_squared <= tolerance * tolerance {
        return None;
    }
    let candidate = Curve::line(start, end);
    let parameters = states
        .iter()
        .map(|state| (state.point - start).dot(&direction) / length_squared)
        .collect::<Vec<_>>();
    if !parameters_are_ordered(&parameters, tolerance)
        || !curve_3_matches_samples(&candidate, states, &parameters, tolerance)
    {
        return None;
    }
    Some(AnalyticalCurve3 {
        curve: candidate,
        interval: Interval::new(0.0, 1.0),
        parameters,
    })
}

fn recognize_circle_3d(
    states: &[TraceState],
    closed: bool,
    tolerance: f64,
) -> Option<AnalyticalCurve3> {
    if !closed || states.len() < 4 {
        return None;
    }
    let points = states.iter().map(|state| state.point).collect::<Vec<_>>();
    let centroid = Point3::from(
        points
            .iter()
            .fold(Vector3::zeros(), |sum, point| sum + point.coords)
            / points.len() as f64,
    );
    let covariance = points.iter().fold(Matrix3::zeros(), |sum, point| {
        let offset = point - centroid;
        sum + offset * offset.transpose()
    });
    let eigen = SymmetricEigen::new(covariance);
    let normal_index = (0..3)
        .min_by(|left, right| eigen.eigenvalues[*left].total_cmp(&eigen.eigenvalues[*right]))?;
    let mut normal = eigen.eigenvectors.column(normal_index).into_owned();
    if normal.norm() <= tolerance {
        return None;
    }
    normal.normalize_mut();

    let reference = if normal.cross(&Vector3::z()).norm() > 0.1 {
        Vector3::z()
    } else {
        Vector3::y()
    };
    let x_axis = normal.cross(&reference).normalize();
    let y_axis = normal.cross(&x_axis).normalize();
    let mut normal_matrix = Matrix3::zeros();
    let mut rhs = Vector3::zeros();
    for point in &points {
        let offset = point - centroid;
        let x = offset.dot(&x_axis);
        let y = offset.dot(&y_axis);
        let row = Vector3::new(2.0 * x, 2.0 * y, 1.0);
        normal_matrix += row * row.transpose();
        rhs += row * (x * x + y * y);
    }
    let solution = normal_matrix.lu().solve(&rhs)?;
    let center = centroid + x_axis * solution.x + y_axis * solution.y;
    let radius = points
        .iter()
        .map(|point| (point - center).norm())
        .sum::<f64>()
        / points.len() as f64;
    if !radius.is_finite() || radius <= tolerance {
        return None;
    }
    if points.iter().any(|point| {
        let offset = point - center;
        offset.dot(&normal).abs() > tolerance || (offset.norm() - radius).abs() > tolerance
    }) {
        return None;
    }

    // The first point is only within `tolerance` of the fitted plane; its
    // radial direction is taken in the plane so it can be the circle's x axis.
    let first_offset = points.first()? - center;
    let first_radial = first_offset - normal * first_offset.dot(&normal);
    if first_radial.norm() <= tolerance {
        return None;
    }
    // The signed area the samples sweep round the centre. A single sample's
    // turn from the first one cannot say it: a closed loop reaches the same
    // largest turn on both sides, a quarter round each way.
    let orientation = points
        .windows(2)
        .map(|pair| (pair[0] - center).cross(&(pair[1] - center)).dot(&normal))
        .sum::<f64>();
    if orientation < 0.0 {
        normal = -normal;
    }
    let circle = Circle::new(Plane::new(center, first_radial, normal), radius);
    let mut angles = Vec::with_capacity(points.len());
    for point in &points {
        let mut angle = circle.parameter_at(*point).value();
        if let Some(previous) = angles.last().copied() {
            while angle + PI < previous {
                angle += TAU;
            }
            while angle - PI > previous {
                angle -= TAU;
            }
        }
        angles.push(angle);
    }
    let start_angle = *angles.first()?;
    for angle in &mut angles {
        *angle -= start_angle;
    }
    if (angles.last()? - TAU).abs() * radius > tolerance * 10.0 {
        return None;
    }
    let parameters = angles
        .into_iter()
        .map(|angle| (angle / TAU).clamp(0.0, 1.0))
        .collect::<Vec<_>>();
    if !parameters_are_ordered(&parameters, tolerance / radius) {
        return None;
    }
    let candidate = Curve::Circle(circle);
    let native_parameters = parameters
        .iter()
        .map(|parameter| parameter * TAU)
        .collect::<Vec<_>>();
    curve_3_matches_samples(&candidate, states, &native_parameters, tolerance).then_some(
        AnalyticalCurve3 {
            curve: candidate,
            interval: Interval::new(0.0, TAU),
            parameters,
        },
    )
}

fn recognize_line_2d(
    points: &[Point2],
    parameters: &[f64],
    tolerance: f64,
) -> Option<TrimmedCurve2> {
    if points.len() != parameters.len() {
        return None;
    }
    let start = *points.first()?;
    let end = *points.last()?;
    if (end - start).norm() <= tolerance {
        return None;
    }
    let candidate = TrimmedCurve2::segment(start, end);
    points
        .iter()
        .zip(parameters)
        .all(|(point, parameter)| {
            (candidate.point_at(Fraction::new(*parameter)) - point).norm() <= tolerance
        })
        .then_some(candidate)
}

fn recognize_circle_2d(
    points: &[Point2],
    parameters: &[f64],
    tolerance: f64,
) -> Option<TrimmedCurve2> {
    if points.len() < 4
        || points.len() != parameters.len()
        || (points.first()? - points.last()?).norm() > tolerance * 10.0
    {
        return None;
    }
    let mut normal_matrix = Matrix3::zeros();
    let mut rhs = Vector3::zeros();
    for point in points {
        let row = Vector3::new(2.0 * point.x, 2.0 * point.y, 1.0);
        normal_matrix += row * row.transpose();
        rhs += row * point.coords.norm_squared();
    }
    let solution = normal_matrix.lu().solve(&rhs)?;
    let center = Point2::new(solution.x, solution.y);
    let radius = points
        .iter()
        .map(|point| (point - center).norm())
        .sum::<f64>()
        / points.len() as f64;
    if !radius.is_finite()
        || radius <= tolerance
        || points
            .iter()
            .any(|point| ((point - center).norm() - radius).abs() > tolerance)
    {
        return None;
    }
    let start_direction = points.first()? - center;
    // The support starts at the first sample, so its angle is zero there and
    // the span states which way the branch runs round it.
    let support = Curve2::Circle(Circle2::new(center, start_direction, radius));
    [TAU, -TAU].into_iter().find_map(|sweep| {
        let candidate = TrimmedCurve2::new(support.clone(), Interval::new(0.0, sweep));
        points
            .iter()
            .zip(parameters)
            .all(|(point, parameter)| {
                (candidate.point_at(Fraction::new(*parameter)) - point).norm() <= tolerance
            })
            .then_some(candidate)
    })
}

fn curve_3_matches_samples(
    candidate: &Curve,
    states: &[TraceState],
    parameters: &[f64],
    tolerance: f64,
) -> bool {
    states.iter().zip(parameters).all(|(state, parameter)| {
        (candidate.point_at(NativeParam::new(*parameter)) - state.point).norm() <= tolerance
    })
}

fn parameters_are_ordered(parameters: &[f64], tolerance: f64) -> bool {
    parameters
        .iter()
        .all(|parameter| (-tolerance..=1.0 + tolerance).contains(parameter))
        && parameters
            .windows(2)
            .all(|window| window[1] + tolerance >= window[0])
}

#[cfg(test)]
mod tests {
    use std::f64::consts::TAU;

    use nalgebra::Vector4;

    use super::{TraceState, recognize_circle_3d};
    use crate::geometry::parameter::NativeParam;
    use crate::geometry::{Curve, Point3};

    fn circle_states(radius: f64, count: usize, turn: f64) -> Vec<TraceState> {
        (0..=count)
            .map(|i| {
                let angle = turn * TAU * i as f64 / count as f64;
                TraceState {
                    parameters: Vector4::zeros(),
                    point: Point3::new(radius * angle.cos(), radius * angle.sin(), 0.0),
                    residual: 0.0,
                }
            })
            .collect()
    }

    #[test]
    fn closed_circle_is_recognized_in_either_traversal_direction() {
        // Evenly spaced samples turn a quarter round both ways from the first
        // one, so the direction has to come from the whole loop.
        for turn in [1.0, -1.0] {
            let states = circle_states(0.5, 32, turn);
            let recognized = recognize_circle_3d(&states, true, 1.0e-6)
                .unwrap_or_else(|| panic!("no circle for turn {turn}"));
            let second = recognized
                .curve
                .point_at(NativeParam::new(recognized.parameters[1] * TAU));
            assert!((second - states[1].point).norm() <= 1.0e-6);
        }
    }

    #[test]
    fn circle_is_recognized_when_its_first_sample_sits_off_the_fitted_plane() {
        // Every sample is within tolerance of the circle, but the first one
        // leaves the fitted plane by far more than a frame's perpendicularity
        // check allows between its radial direction and the normal.
        let (radius, tolerance) = (0.5, 1.0e-4);
        let mut states = circle_states(radius, 32, 1.0);
        states[0].point.z = 0.5 * tolerance;

        let recognized = recognize_circle_3d(&states, true, tolerance).expect("a circle");
        let Curve::Circle(circle) = recognized.curve else {
            panic!("expected a circle, got {:?}", recognized.curve);
        };
        assert!((circle.radius() - radius).abs() <= tolerance);
    }
}
