//! Rebuilding the parameter curve of a fused boundary.
//!
//! A fused edge invalidates the parameter curves of the pieces it replaced, so
//! every face that carries it needs a new one. Planar faces get an exact answer
//! from the same projection the profile builders use. Other surfaces get a
//! fitted line or arc in parameter space, accepted only when lifting it back to
//! three dimensions traces the fused edge within tolerance — a check in model
//! units, which keeps it meaningful on a surface whose parameters are angles.

use crate::builders::profiles::curve_pcurve;
use crate::geometry::{Circle2, Curve, Curve2, Line2, Point2, Point3, Surface, Vector2};

use super::curve::{SUPPORT_SAMPLES, sample_between};

/// Builds the parameter curve of a boundary running along `curve` from `start`
/// to `end` on `surface`.
pub fn boundary_pcurve(
    surface: &Surface,
    curve: &Curve,
    start: Point3,
    end: Point3,
    linear: f64,
) -> Option<Curve2> {
    if let Surface::Plane(plane) = surface {
        return curve_pcurve(curve, start, end, plane).ok();
    }

    let samples = sample_between(curve, start, end, SUPPORT_SAMPLES);
    let parameters = samples
        .iter()
        .map(|&point| surface.closest_parameter(point).ok())
        .collect::<Option<Vec<_>>>()?;
    let last = parameters.len() - 1;

    let straight = Curve2::Line(Line2::new(parameters[0], parameters[last]));
    if traces(surface, &straight, &samples, linear) {
        return Some(straight);
    }
    let curved = Curve2::Circle(circle2_through(
        parameters[0],
        parameters[last / 2],
        parameters[last],
        linear,
    )?);
    traces(surface, &curved, &samples, linear).then_some(curved)
}

/// Reports whether lifting `candidate` onto `surface` traces `samples`.
///
/// The comparison is a two-sided polyline distance in model units: every lifted
/// point must sit on the sampled edge and every sampled point must be reached
/// by the lifted curve.
fn traces(surface: &Surface, candidate: &Curve2, samples: &[Point3], linear: f64) -> bool {
    let lifted = candidate
        .sample(4 * SUPPORT_SAMPLES)
        .into_iter()
        .map(|uv| surface.point_at(uv.x, uv.y))
        .collect::<Vec<_>>();
    polylines_agree(&lifted, samples, linear) && polylines_agree(samples, &lifted, linear)
}

/// Reports whether every point of `points` lies within `tolerance` of the
/// polyline through `polyline`.
fn polylines_agree(points: &[Point3], polyline: &[Point3], tolerance: f64) -> bool {
    points.iter().all(|&point| {
        polyline
            .windows(2)
            .map(|segment| segment_distance(point, segment[0], segment[1]))
            .fold(f64::INFINITY, f64::min)
            <= tolerance
    })
}

/// Returns the distance from a point to a segment.
fn segment_distance(point: Point3, start: Point3, end: Point3) -> f64 {
    let direction = end - start;
    let length_squared = direction.norm_squared();
    if length_squared <= f64::EPSILON {
        return (point - start).norm();
    }
    let parameter = ((point - start).dot(&direction) / length_squared).clamp(0.0, 1.0);
    (point - (start + direction * parameter)).norm()
}

/// Returns the parameter-space arc through three points, sweeping from the
/// first through the second to the third.
fn circle2_through(first: Point2, second: Point2, third: Point2, linear: f64) -> Option<Circle2> {
    let determinant = 2.0
        * (first.x * (second.y - third.y)
            + second.x * (third.y - first.y)
            + third.x * (first.y - second.y));
    if determinant.abs() <= linear {
        return None;
    }
    let square = |point: Point2| point.x * point.x + point.y * point.y;
    let center = Point2::new(
        (square(first) * (second.y - third.y)
            + square(second) * (third.y - first.y)
            + square(third) * (first.y - second.y))
            / determinant,
        (square(first) * (third.x - second.x)
            + square(second) * (first.x - third.x)
            + square(third) * (second.x - first.x))
            / determinant,
    );

    let x_dir = first - center;
    let radius = x_dir.norm();
    if radius <= linear {
        return None;
    }
    let y_dir = Vector2::new(-x_dir.y, x_dir.x);
    let angle = |point: Point2| {
        let radial = point - center;
        radial.dot(&y_dir).atan2(radial.dot(&x_dir))
    };

    let interior = angle(second);
    if interior == 0.0 {
        return None;
    }
    let closing = angle(third);
    let sweep = if interior > 0.0 {
        if closing > interior {
            closing
        } else {
            closing + std::f64::consts::TAU
        }
    } else if closing < interior {
        closing
    } else {
        closing - std::f64::consts::TAU
    };
    Some(Circle2::new(center, x_dir, radius, sweep))
}
