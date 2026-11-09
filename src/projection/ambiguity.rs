//! Where the nearest point of a target is not unique.
//!
//! A surface of revolution or a circle is equally near a point on its axis
//! from a whole ring of points; an ellipse is equally near the points of the
//! major axis inside its evolute from two. These predicates name those cases.
//!
//! An analytic target is checked in closed form. A target whose nearest point
//! is searched numerically (a NURBS curve or surface, a ruled surface) is
//! checked on the local nearest points that search finds: the nearest point
//! is not unique when two of them, farther apart than the tolerance, are
//! equally near within the tolerance.

use crate::geometry::{Curve, Frame, Point3, Surface};

/// Whether `point` has several nearest points on `surface`, farther apart than
/// `tol` and equally near within `tol`.
///
/// `reported` is the nearest point the surface reported for `point`.
pub(super) fn has_several_nearest_points_on_surface(
    surface: &Surface,
    point: Point3,
    reported: Point3,
    tol: f64,
) -> bool {
    match surface {
        Surface::Plane(_) => false,
        Surface::Ruled(_) | Surface::Nurbs(_) => {
            let local_nearest = surface
                .local_nearest_parameters(point)
                .unwrap_or_default()
                .into_iter()
                .map(|uv| surface.point_at(uv.x, uv.y));
            has_several_equally_near_points(point, reported, local_nearest, tol)
        }
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
        // The ring of nearest points from a point on the axis collapses to one only when
        // the nearest point is itself on the axis.
        Surface::Revolution(revolution) => {
            let from_axis = |p: Point3| (p - revolution.axis.closest_point(p)).norm();
            from_axis(point) <= tol && from_axis(reported) > tol
        }
    }
}

/// Whether `point` has several nearest points on `curve`, farther apart than
/// `tol` and equally near within `tol`.
///
/// `reported` is the nearest point the curve reported for `point`.
pub(super) fn has_several_nearest_points_on_curve(
    curve: &Curve,
    point: Point3,
    reported: Point3,
    tol: f64,
) -> bool {
    match curve {
        Curve::Line(_) => false,
        Curve::Nurbs(_) => {
            let local_nearest = curve
                .local_nearest_parameters(point, None)
                .into_iter()
                .map(|t| curve.point_at(t));
            has_several_equally_near_points(point, reported, local_nearest, tol)
        }
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

/// Whether, among `reported` and `local_nearest`, the points nearest `point`
/// within `tol` of the nearest distance are not all within `tol` of one
/// another.
fn has_several_equally_near_points(
    point: Point3,
    reported: Point3,
    local_nearest: impl IntoIterator<Item = Point3>,
    tol: f64,
) -> bool {
    let reach = |p: &Point3| (p - point).norm();
    let all: Vec<Point3> = std::iter::once(reported).chain(local_nearest).collect();
    let nearest = all.iter().map(reach).fold(f64::INFINITY, f64::min);
    let mut equally_near = all.iter().filter(|p| reach(p) <= nearest + tol);
    let Some(first) = equally_near.next() else {
        return false;
    };
    equally_near.any(|p| (p - first).norm() > tol)
}

/// The distance from `point` to the `z` axis of `frame`.
fn radial_distance(frame: &Frame, point: Point3) -> f64 {
    let local = frame.coordinates_of(point);
    local.x.hypot(local.y)
}
