use std::f64::consts::{PI, TAU};

use nalgebra::{Matrix3, SymmetricEigen, Vector3};

use super::tracer::TraceState;
use crate::geometry::{
    Bounded, Circle, Circle2, Curve, Curve2, Interval, Line2, NurbsCurve2, Plane, Point2, Point3,
};

type Curve3Recognizer = fn(&[TraceState], bool, f64) -> Option<AnalyticalCurve3>;
type Curve2Recognizer = fn(&[Point2], &[f64], f64) -> Option<Curve2>;

const CURVE_3_RECOGNIZERS: [Curve3Recognizer; 2] = [recognize_line_3d, recognize_circle_3d];
const CURVE_2_RECOGNIZERS: [Curve2Recognizer; 2] = [recognize_line_2d, recognize_circle_2d];

/// An analytical 3D curve together with its normalized natural sample parameters.
pub(super) struct AnalyticalCurve3 {
    pub curve: Curve,
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
) -> Curve2 {
    CURVE_2_RECOGNIZERS
        .iter()
        .find_map(|recognizer| recognizer(points, parameters, tolerance))
        .unwrap_or(Curve2::Nurbs(fallback))
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

    let first_radial = points.first()? - center;
    if first_radial.norm() <= tolerance {
        return None;
    }
    let orientation = points
        .iter()
        .skip(1)
        .map(|point| first_radial.cross(&(point - center)).dot(&normal))
        .max_by(|left, right| left.abs().total_cmp(&right.abs()))?;
    if orientation < 0.0 {
        normal = -normal;
    }
    let circle = Circle::new(Plane::new(center, first_radial, normal), radius);
    let mut angles = Vec::with_capacity(points.len());
    for point in &points {
        let mut angle = circle.param_at(*point);
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
    let candidate = Curve::Bounded(Box::new(Bounded::new(
        Curve::Circle(circle),
        Interval::new(0.0, TAU),
    )));
    curve_3_matches_samples(&candidate, states, &parameters, tolerance).then_some(
        AnalyticalCurve3 {
            curve: candidate,
            parameters,
        },
    )
}

fn recognize_line_2d(points: &[Point2], parameters: &[f64], tolerance: f64) -> Option<Curve2> {
    if points.len() != parameters.len() {
        return None;
    }
    let start = *points.first()?;
    let end = *points.last()?;
    if (end - start).norm() <= tolerance {
        return None;
    }
    let candidate = Curve2::Line(Line2::new(start, end));
    points
        .iter()
        .zip(parameters)
        .all(|(point, parameter)| (candidate.point_at(*parameter) - point).norm() <= tolerance)
        .then_some(candidate)
}

fn recognize_circle_2d(points: &[Point2], parameters: &[f64], tolerance: f64) -> Option<Curve2> {
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
    [TAU, -TAU].into_iter().find_map(|sweep| {
        let candidate = Curve2::Circle(Circle2::new(center, start_direction, radius, sweep));
        points
            .iter()
            .zip(parameters)
            .all(|(point, parameter)| (candidate.point_at(*parameter) - point).norm() <= tolerance)
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
        (candidate.point_at(*parameter) - state.point).norm() <= tolerance
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
