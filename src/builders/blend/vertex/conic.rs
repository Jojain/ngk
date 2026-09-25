//! Arcs of the conic sections vertex treatments cut.

use nalgebra::Vector3;

use crate::geometry::parameter::Fraction;
use crate::geometry::{
    Circle, Curve, Ellipse, Frame, LINEAR_TOLERANCE, Plane, Point3, TrimmedCurve,
};

/// The arc from `from` to `to` of the section of a cylinder by a plane, on
/// whichever side passes nearer `near`.
///
/// The cylinder's axis runs through `origin` along `axis`. Square to the axis
/// the section is a circle; otherwise it is an ellipse whose minor radius is
/// the cylinder's and whose major radius is stretched by the obliquity. A
/// plane running along the axis cuts rulings, not a conic, and gives `None`,
/// as does an end point off the section.
#[allow(clippy::too_many_arguments)]
pub(crate) fn cylinder_section_arc(
    origin: Point3,
    axis: Vector3<f64>,
    radius: f64,
    plane_point: Point3,
    plane_normal: Vector3<f64>,
    from: Point3,
    to: Point3,
    near: Point3,
) -> Option<TrimmedCurve> {
    let axis = axis.normalize();
    let normal = plane_normal.normalize();
    let alignment = normal.dot(&axis);
    if alignment.abs() <= LINEAR_TOLERANCE.sqrt() {
        return None;
    }
    let center = origin + axis * ((plane_point - origin).dot(&normal) / alignment);
    let conic = if 1.0 - alignment.abs() <= LINEAR_TOLERANCE {
        Curve::Circle(Circle::new(Plane::new(center, from - center, axis), radius))
    } else {
        let minor = axis.cross(&normal).normalize();
        let major = normal.cross(&minor).normalize();
        Curve::Ellipse(Ellipse::new(
            Frame::from_xy(center, major, minor),
            radius / alignment.abs(),
            radius,
        ))
    };
    nearest_arc(conic, from, to, near)
}

/// The arc from `from` to `to` of a closed conic, whichever of its two passes
/// nearer `near`; `None` when either end is off the conic.
pub(crate) fn nearest_arc(
    conic: Curve,
    from: Point3,
    to: Point3,
    near: Point3,
) -> Option<TrimmedCurve> {
    let on = |point: Point3| {
        (conic.point_at(conic.parameter_at(point)) - point).norm() <= LINEAR_TOLERANCE.sqrt()
    };
    if !on(from) || !on(to) {
        return None;
    }
    let forward = TrimmedCurve::between(conic.clone(), from, to);
    let backward = TrimmedCurve::between(conic.reversed(), from, to);
    let distance = |arc: &TrimmedCurve| (arc.point_at(Fraction::new(0.5)) - near).norm();
    Some(if distance(&forward) <= distance(&backward) {
        forward
    } else {
        backward
    })
}
