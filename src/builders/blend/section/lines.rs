//! Straight-line helpers the closed-form sections and treatments share.

use nalgebra::Vector3;

use crate::geometry::parameter::Fraction;
use crate::geometry::{Curve, LINEAR_TOLERANCE, Point3, TrimmedCurve};

/// Whether a span runs along its chord, whichever curve type carries it.
pub(crate) fn is_straight(span: &TrimmedCurve) -> bool {
    if matches!(span.curve(), Curve::Line(_)) {
        return true;
    }
    let (start, end) = (span.start(), span.end());
    let chord = end - start;
    if chord.norm() <= LINEAR_TOLERANCE {
        return false;
    }
    (1..8).all(|index| {
        let point = span.point_at(Fraction::new(f64::from(index) / 8.0));
        distance_to_line(point, start, chord) <= LINEAR_TOLERANCE
    })
}

/// Where two lines meet, or `None` when they are parallel or miss each other.
pub(crate) fn line_line(
    first: Point3,
    first_direction: Vector3<f64>,
    second: Point3,
    second_direction: Vector3<f64>,
) -> Option<Point3> {
    let d = first_direction.normalize();
    let e = second_direction.normalize();
    let between = second - first;
    let cosine = d.dot(&e);
    let denominator = 1.0 - cosine * cosine;
    if denominator <= LINEAR_TOLERANCE {
        return None;
    }
    let s = (between.dot(&d) - cosine * between.dot(&e)) / denominator;
    let t = (cosine * between.dot(&d) - between.dot(&e)) / denominator;
    let on_first = first + d * s;
    let on_second = second + e * t;
    ((on_first - on_second).norm() <= LINEAR_TOLERANCE.sqrt())
        .then(|| Point3::from((on_first.coords + on_second.coords) * 0.5))
}

/// Distance from `point` to the line through `origin` along `direction`.
pub(crate) fn distance_to_line(point: Point3, origin: Point3, direction: Vector3<f64>) -> f64 {
    let direction = direction.normalize();
    let offset = point - origin;
    (offset - direction * offset.dot(&direction)).norm()
}
