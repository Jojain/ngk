use std::f64::consts::FRAC_PI_2;

use nalgebra::Vector3;
use ngk::geometry::{
    Curve, Cylinder, LINEAR_TOLERANCE, Line, Plane, Point3, PointCoincidence, RuledSurface,
    Surface, SurfaceOfRevolution,
};

fn assert_point_near(actual: Point3, expected: Point3) {
    assert!(
        actual.coincides(expected, LINEAR_TOLERANCE),
        "expected {expected:?}, got {actual:?}"
    );
}

#[test]
fn plane_new_orthonormalizes_frame() {
    let plane = Plane::new(
        Point3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 1.0),
        Vector3::new(0.0, 0.0, 1.0),
    );

    assert!(plane.frame.x_dir.dot(&plane.frame.z_dir).abs() < 1e-10);
    assert!(plane.x_dir().dot(&plane.normal()).abs() < 1e-10);
    assert!(plane.y_dir().dot(&plane.normal()).abs() < 1e-10);
    assert_point_near(plane.point_at(2.0, 3.0), Point3::new(2.0, 3.0, 0.0));
}

#[test]
fn cylinder_point_at_wraps_around_axis() {
    let cylinder = Cylinder::new(
        Point3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
        2.0,
    );

    assert_point_near(cylinder.origin(), Point3::new(0.0, 0.0, 0.0));
    assert!(cylinder.x_dir().dot(&cylinder.axis()).abs() < 1e-10);
    assert_point_near(cylinder.point_at(0.0, 0.0), Point3::new(2.0, 0.0, 0.0));
    assert_point_near(
        cylinder.point_at(FRAC_PI_2, 0.0),
        Point3::new(0.0, 2.0, 0.0),
    );
}

#[test]
fn cylinder_point_at_moves_along_axis() {
    let surface = Surface::Cylinder(Cylinder::new(
        Point3::new(1.0, 2.0, 3.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
        0.5,
    ));

    assert_point_near(surface.point_at(0.0, 4.0), Point3::new(1.5, 2.0, 7.0));
}

#[test]
fn plane_surface_converts_to_matching_nurbs_patch() {
    let surface = Surface::Plane(Plane::new(
        Point3::new(1.0, 2.0, 3.0),
        Vector3::x(),
        Vector3::z(),
    ));
    let nurbs = surface.to_nurbs().unwrap();

    assert_eq!(nurbs.degree_u().get(), 1);
    assert_eq!(nurbs.degree_v().get(), 1);
    assert_point_near(nurbs.point_at(0.0, 0.0), surface.point_at(0.0, 0.0));
    assert_point_near(nurbs.point_at(0.25, 0.75), surface.point_at(0.25, 0.75));
    assert_point_near(nurbs.point_at(1.0, 1.0), surface.point_at(1.0, 1.0));
}

#[test]
fn cylinder_surface_converts_to_matching_rational_nurbs_patch() {
    let surface = Surface::Cylinder(Cylinder::new(
        Point3::new(1.0, 2.0, 3.0),
        Vector3::x(),
        Vector3::z(),
        0.5,
    ));
    let nurbs = surface.to_nurbs().unwrap();

    assert_eq!(nurbs.degree_u().get(), 2);
    assert_eq!(nurbs.degree_v().get(), 1);
    for u in [
        0.0,
        std::f64::consts::FRAC_PI_4,
        std::f64::consts::FRAC_PI_2,
        std::f64::consts::PI,
        std::f64::consts::TAU,
    ] {
        assert_point_near(nurbs.point_at(u, 0.25), surface.point_at(u, 0.25));
    }
}

#[test]
fn ruled_surface_converts_to_matching_nurbs_patch() {
    let surface = Surface::Ruled(RuledSurface::new(
        Curve::Line(Line::new(
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
        )),
        Vector3::new(0.0, 0.0, 2.0),
    ));
    let nurbs = surface.to_nurbs().unwrap();

    assert_eq!(nurbs.degree_u().get(), 1);
    assert_eq!(nurbs.degree_v().get(), 1);
    assert_point_near(nurbs.point_at(0.25, 0.75), surface.point_at(0.25, 0.75));
}

#[test]
fn surface_of_revolution_converts_to_matching_nurbs_patch() {
    let surface = Surface::Revolution(SurfaceOfRevolution::new(
        Curve::Line(Line::new(
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 3.0),
        )),
        Point3::origin(),
        Vector3::z(),
    ));
    let nurbs = surface.to_nurbs().unwrap();

    assert_eq!(nurbs.degree_u().get(), 1);
    assert_eq!(nurbs.degree_v().get(), 2);
    for v in [
        0.0,
        std::f64::consts::FRAC_PI_4,
        std::f64::consts::FRAC_PI_2,
        std::f64::consts::PI,
        std::f64::consts::TAU,
    ] {
        assert_point_near(nurbs.point_at(0.25, v), surface.point_at(0.25, v));
    }
}
