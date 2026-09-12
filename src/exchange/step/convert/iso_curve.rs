//! The curve a surface traces along one of its parameter directions.
//!
//! STEP cannot spell a face whose parameterization closes on itself: a
//! cylinder wall, a sphere, a torus all have to arrive with their domain cut
//! open along a seam, and that seam is an edge like any other — it needs a
//! 3D curve. NGK stores no such edge, because a seam is a property of a
//! *reading* of a periodic face rather than of the shape, so the curve has to
//! be derived from the support on the way out.
//!
//! Every cut a domain makes runs along a parameter direction, so the only
//! curves needed here are the parameter lines, and on the analytic supports
//! each is a line or a circle in closed form. Anything else declines, and the
//! caller refuses the face by name rather than approximating a boundary.
//!
//! **The parameterizations agree.** Each curve below is anchored so that its
//! own parameter *is* the surface parameter that varies: a cylinder's
//! latitude circle starts at the cylinder's own reference direction, a
//! sphere's meridian starts on the equator. An edge derives its span from its
//! bounding vertices, so a curve anchored anywhere else would still pass
//! through both ends and take the wrong way round between them.

use crate::geometry::{Axis2, LINEAR_TOLERANCE};
use crate::geometry::{Circle, Cone, Curve, Cylinder, Line, Plane, Point2, Sphere, Surface, Torus};

use nalgebra::Vector3;

/// Returns the curve a surface traces from `from` to `to`, when the two lie on
/// one parameter line and that line has a closed form.
///
/// Declines a pair that varies in both parameters — which is not a cut — and a
/// support whose parameter lines this build has no analytic spelling for.
pub fn iso_curve(surface: &Surface, from: Point2, to: Point2) -> Option<Curve> {
    let axis = varying_axis(from, to)?;
    match (surface, axis) {
        // A plane is its own parameter space, so both directions are lines and
        // neither is a special case.
        (Surface::Plane(plane), _) => Some(Curve::Line(Line::through(
            plane.point_at(from.x, from.y),
            plane.point_at(to.x, to.y),
        ))),
        (Surface::Cylinder(cylinder), Axis2::U) => Some(cylinder_latitude(cylinder, from.y)),
        (Surface::Cylinder(cylinder), Axis2::V) => Some(Curve::Line(Line::through(
            cylinder.point_at(from.x, from.y),
            cylinder.point_at(to.x, to.y),
        ))),
        (Surface::Sphere(sphere), Axis2::U) => sphere_latitude(sphere, from.y),
        (Surface::Sphere(sphere), Axis2::V) => Some(sphere_meridian(sphere, from.x)),
        (Surface::Cone(cone), Axis2::U) => cone_latitude(cone, from.y),
        (Surface::Cone(cone), Axis2::V) => Some(Curve::Line(Line::through(
            cone.point_at(from.x, from.y),
            cone.point_at(to.x, to.y),
        ))),
        (Surface::Torus(torus), Axis2::U) => torus_latitude(torus, from.y),
        (Surface::Torus(torus), Axis2::V) => Some(torus_tube(torus, from.x)),
        _ => None,
    }
}

/// Which parameter varies between two points, when exactly one of them does.
///
/// A cut runs along a parameter line, so a pair that moves in both directions
/// is not one and gets no curve. A pair that moves in neither is not one
/// either: it is a collapsed row, which carries no edge at all.
fn varying_axis(from: Point2, to: Point2) -> Option<Axis2> {
    let (du, dv) = ((to.x - from.x).abs(), (to.y - from.y).abs());
    if du <= LINEAR_TOLERANCE && dv <= LINEAR_TOLERANCE {
        return None;
    }
    if dv <= LINEAR_TOLERANCE {
        Some(Axis2::U)
    } else if du <= LINEAR_TOLERANCE {
        Some(Axis2::V)
    } else {
        None
    }
}

/// The circle a cylinder traces at one height, parameterized by its own `u`.
fn cylinder_latitude(cylinder: &Cylinder, v: f64) -> Curve {
    let plane = Plane::new(
        cylinder.origin() + *cylinder.axis() * v,
        cylinder.x_dir(),
        cylinder.axis(),
    );
    Curve::Circle(Circle::new(plane, cylinder.radius))
}

/// The circle a sphere traces at one latitude, parameterized by its own `u`.
///
/// Declines a pole, where the whole row is one point and the cut across it
/// carries no edge at all.
fn sphere_latitude(sphere: &Sphere, v: f64) -> Option<Curve> {
    let frame = sphere.frame();
    let radius = sphere.radius() * v.cos();
    if radius.abs() <= LINEAR_TOLERANCE {
        return None;
    }
    let plane = Plane::new(
        frame.origin + *frame.z_dir * (sphere.radius() * v.sin()),
        frame.x_dir,
        frame.z_dir,
    );
    Some(Curve::Circle(Circle::new(plane, radius)))
}

/// The great circle a sphere traces at one longitude, parameterized by `v`.
///
/// Its plane's x runs to the equator at that longitude and its y is the polar
/// axis, so the circle's own angle is latitude — which is what makes the
/// meridian's parameter the surface's `v` rather than a rotation of it.
fn sphere_meridian(sphere: &Sphere, u: f64) -> Curve {
    let frame = sphere.frame();
    let radial = radial_direction(frame.x_dir.into_inner(), frame.y_dir.into_inner(), u);
    let plane = Plane::from_xy(frame.origin, radial, frame.z_dir);
    Curve::Circle(Circle::new(plane, sphere.radius()))
}

/// The circle a cone traces at one generatrix distance.
///
/// Declines the apex, and declines the far nappe: past the apex the radius the
/// surface traces is negative, and a `CIRCLE` of negative radius is not a
/// thing the schema has.
fn cone_latitude(cone: &Cone, v: f64) -> Option<Curve> {
    let radius = cone.radius_at(v);
    if radius <= LINEAR_TOLERANCE {
        return None;
    }
    let frame = cone.frame();
    let plane = Plane::new(
        frame.origin + *frame.z_dir * (v * cone.half_angle().cos()),
        frame.x_dir,
        frame.z_dir,
    );
    Some(Curve::Circle(Circle::new(plane, radius)))
}

/// The circle a torus traces at one tube angle, parameterized by its own `u`.
fn torus_latitude(torus: &Torus, v: f64) -> Option<Curve> {
    let frame = torus.frame();
    let radius = torus.major_radius() + torus.minor_radius() * v.cos();
    if radius.abs() <= LINEAR_TOLERANCE {
        return None;
    }
    let plane = Plane::new(
        frame.origin + *frame.z_dir * (torus.minor_radius() * v.sin()),
        frame.x_dir,
        frame.z_dir,
    );
    Some(Curve::Circle(Circle::new(plane, radius)))
}

/// The tube circle a torus traces at one longitude, parameterized by `v`.
fn torus_tube(torus: &Torus, u: f64) -> Curve {
    let frame = torus.frame();
    let radial = radial_direction(frame.x_dir.into_inner(), frame.y_dir.into_inner(), u);
    let plane = Plane::from_xy(
        frame.origin + radial * torus.major_radius(),
        radial,
        frame.z_dir,
    );
    Curve::Circle(Circle::new(plane, torus.minor_radius()))
}

/// The in-plane direction at angle `u` from a frame's x towards its y.
fn radial_direction(x: Vector3<f64>, y: Vector3<f64>, u: f64) -> Vector3<f64> {
    x * u.cos() + y * u.sin()
}
