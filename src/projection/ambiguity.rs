//! Where the nearest point of an analytic target is not unique.
//!
//! A surface of revolution or a circle is equally near a point on its axis
//! from a whole ring of feet; an ellipse is equally near the points of the
//! major axis inside its evolute from two. These predicates name those cases.
//! NURBS and ruled targets are not searched for ties.

use crate::geometry::{Curve, Frame, Point3, Surface};

/// Whether the nearest point of `surface` to `point` is not unique.
///
/// `foot` is the nearest point the surface reported for `point`.
pub(super) fn surface_feet_tie(surface: &Surface, point: Point3, foot: Point3, tol: f64) -> bool {
    match surface {
        Surface::Plane(_) | Surface::Ruled(_) | Surface::Nurbs(_) => false,
        Surface::Sphere(sphere) => (point - sphere.frame().origin).norm() <= tol,
        Surface::Cylinder(cylinder) => radial_distance(&cylinder.frame, point) <= tol,
        Surface::Cone(cone) => {
            let on_axis = radial_distance(cone.frame(), point) <= tol;
            let at_apex = cone
                .apex_parameter()
                .is_some_and(|v| (cone.point_at(0.0, v) - point).norm() <= tol);
            on_axis && !at_apex
        }
        Surface::Torus(torus) => {
            let frame = torus.frame();
            let radial = radial_distance(frame, point);
            let height = frame.coordinates_of(point).z;
            let on_core_circle =
                (radial - torus.major_radius()).abs() <= tol && height.abs() <= tol;
            radial <= tol || on_core_circle
        }
        // The ring of feet from a point on the axis collapses to one only when
        // the nearest point is itself on the axis.
        Surface::Revolution(revolution) => {
            let from_axis = |p: Point3| (p - revolution.axis.project(p)).norm();
            from_axis(point) <= tol && from_axis(foot) > tol
        }
    }
}

/// Whether the nearest point of `curve` to `point` is not unique.
pub(super) fn curve_feet_tie(curve: &Curve, point: Point3, tol: f64) -> bool {
    match curve {
        Curve::Line(_) | Curve::Nurbs(_) => false,
        Curve::Circle(circle) => radial_distance(&circle.plane().frame, point) <= tol,
        Curve::Helix(helix) => radial_distance(helix.frame(), point) <= tol,
        Curve::Ellipse(ellipse) => {
            let (a, b) = (ellipse.major_radius(), ellipse.minor_radius());
            let local = ellipse.frame().coordinates_of(point);
            if local.z.abs() > tol {
                return false;
            }
            if (a - b).abs() <= tol {
                return local.x.hypot(local.y) <= tol;
            }
            // The points of the long axis inside the evolute are equally near
            // the two vertices of the short axis.
            let reach = (a * a - b * b).abs() / a.max(b);
            let (along_long, across_long) = if a > b {
                (local.x, local.y)
            } else {
                (local.y, local.x)
            };
            across_long.abs() <= tol && along_long.abs() < reach
        }
    }
}

/// The distance from `point` to the `z` axis of `frame`.
fn radial_distance(frame: &Frame, point: Point3) -> f64 {
    let local = frame.coordinates_of(point);
    local.x.hypot(local.y)
}
