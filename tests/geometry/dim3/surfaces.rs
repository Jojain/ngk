use std::f64::consts::FRAC_PI_2;

use nalgebra::{Rotation3, Vector3};
use ngk::geometry::{
    Circle, Curve, Cylinder, Interval, LINEAR_TOLERANCE, Plane, Point3, PointCoincidence,
    RuledSurface, Surface, SurfaceOfRevolution, SurfacePeriodicity, axis::Axis3,
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
fn surfaces_report_parameter_periodicity() {
    let plane = Surface::Plane(Plane::xy());
    let cylinder = Surface::Cylinder(Cylinder::new(
        Point3::origin(),
        Vector3::x(),
        Vector3::z(),
        1.0,
    ));
    let ruled_circle = Surface::Ruled(RuledSurface::new(
        Curve::Circle(Circle::new(Plane::xy(), 1.0)),
        Vector3::z(),
    ));
    let revolution = Surface::Revolution(SurfaceOfRevolution::new(
        Curve::line(Point3::new(1.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)),
        Axis3::new(Point3::origin(), Vector3::z()),
    ));

    assert_eq!(plane.periodicity(), SurfacePeriodicity::None);
    assert_eq!(
        cylinder.periodicity(),
        SurfacePeriodicity::UPeriodic(std::f64::consts::TAU)
    );
    assert_eq!(ruled_circle.periodicity(), cylinder.periodicity());
    assert_eq!(
        revolution.periodicity(),
        SurfacePeriodicity::VPeriodic(std::f64::consts::TAU)
    );
    let uv_periodicity = SurfacePeriodicity::UVPeriodic(2.0, 3.0);
    let SurfacePeriodicity::UVPeriodic(u_period, v_period) = uv_periodicity else {
        unreachable!();
    };
    assert_eq!((u_period, v_period), (2.0, 3.0));
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
fn plane_closest_parameter_returns_frame_coordinates() {
    let plane = Plane::from_xy(Point3::new(1.0, 2.0, 3.0), Vector3::x(), Vector3::z());
    let uv = Surface::Plane(plane)
        .closest_parameter(Point3::new(4.0, 2.0, 8.0))
        .expect("plane projection should not need NURBS conversion");

    assert!((uv.x - 3.0).abs() <= LINEAR_TOLERANCE);
    assert!((uv.y - 5.0).abs() <= LINEAR_TOLERANCE);
}

#[test]
fn nurbs_surface_closest_parameter_recovers_surface_point_parameters() {
    let surface = Surface::Ruled(RuledSurface::new(
        Curve::line(Point3::new(1.0, 0.0, 0.0), Point3::new(3.0, 0.0, 0.0)),
        Vector3::new(0.0, 0.0, 2.0),
    ))
    .to_nurbs()
    .expect("ruled surface should convert to NURBS");
    let point = surface.point_at(0.35, 0.8);
    let uv = surface.closest_parameter(point);

    assert!((uv.x - 0.35).abs() <= 1.0e-8);
    assert!((uv.y - 0.8).abs() <= 1.0e-8);
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
fn cylinder_nurbs_patch_spans_the_requested_height_interval() {
    let surface = Surface::Cylinder(Cylinder::new(
        Point3::new(0.0, 0.0, 0.0),
        Vector3::x(),
        Vector3::z(),
        2.0,
    ));
    let nurbs = surface
        .to_nurbs_over(
            Interval::new(0.0, std::f64::consts::TAU),
            Interval::new(0.0, 5.0),
        )
        .unwrap();

    // `Cylinder::point_at` moves `v` units along the axis, so a face five units
    // tall must still be five units tall after the conversion the intersection
    // engine runs on. Sampling the height is what catches a truncated patch;
    // sampling only the seam does not.
    for v in [0.0, 1.0, 2.5, 5.0] {
        assert_point_near(nurbs.point_at(0.0, v), surface.point_at(0.0, v));
    }
}

#[test]
fn ruled_surface_converts_to_matching_nurbs_patch() {
    let surface = Surface::Ruled(RuledSurface::new(
        Curve::line(Point3::new(1.0, 0.0, 0.0), Point3::new(3.0, 0.0, 0.0)),
        Vector3::new(0.0, 0.0, 2.0),
    ));
    let nurbs = surface.to_nurbs().unwrap();

    assert_eq!(nurbs.degree_u().get(), 1);
    assert_eq!(nurbs.degree_v().get(), 1);
    assert_point_near(nurbs.point_at(0.25, 0.75), surface.point_at(0.25, 0.75));
}

#[test]
fn surface_of_revolution_converts_to_matching_nurbs_patch() {
    let axis = Axis3::new(Point3::new(2.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 3.0));
    let profile = Curve::line(Point3::new(3.0, 0.0, 0.0), Point3::new(4.0, 0.0, 0.0));
    let surface = Surface::Revolution(SurfaceOfRevolution::new(profile, axis));
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

#[test]
fn surface_of_revolution_normal_is_radial_on_a_cylinder() {
    let surface = SurfaceOfRevolution::new(
        Curve::line(Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 1.0)),
        Axis3::new(Point3::origin(), Vector3::z()),
    );

    // Central differences of the surface point are order 1e-6 apart, so their
    // cross product lands under the degeneracy tolerance and reports no normal
    // at all; the normal has to come from the analytic partials.
    for angle in [0.0, std::f64::consts::FRAC_PI_2, 2.0] {
        let normal = *surface.normal_at(0.5, angle);
        let point = surface.point_at(0.5, angle);
        let radial = Vector3::new(point.x, point.y, 0.0).normalize();

        assert!(
            (normal + radial).norm() <= 1.0e-9,
            "normal at {angle} should be the inward radial direction, got {normal:?}"
        );
    }
}

#[test]
fn rotated_surface_keeps_its_parameterisation() {
    let axis = Axis3::new(Point3::origin(), Vector3::z());
    let surface = Surface::Plane(Plane::from_xy(
        Point3::new(1.0, 0.0, 0.0),
        Vector3::x(),
        Vector3::z(),
    ));
    let rotated = surface
        .rotated(axis, std::f64::consts::FRAC_PI_2)
        .expect("a plane should rotate");

    for (u, v) in [(0.0, 0.0), (0.3, -0.7), (2.0, 1.5)] {
        let expected = Rotation3::from_axis_angle(&axis.direction, std::f64::consts::FRAC_PI_2)
            * surface.point_at(u, v);
        assert!(
            (rotated.point_at(u, v) - expected).norm() <= 1.0e-9,
            "rotating must not re-parameterise the surface"
        );
    }
}

#[test]
fn surface_domains_report_unbounded_directions() {
    let plane = Surface::Plane(Plane::new(Point3::origin(), Vector3::x(), Vector3::z()));
    let (u, v) = plane.domain();
    assert!(
        !u.is_finite() && !v.is_finite(),
        "a plane is unbounded in both directions"
    );

    let cylinder = Surface::Cylinder(Cylinder::new(
        Point3::origin(),
        Vector3::x(),
        Vector3::z(),
        1.0,
    ));
    let (u, v) = cylinder.domain();
    assert_eq!(u, Interval::new(0.0, std::f64::consts::TAU));
    assert!(!v.is_finite(), "a cylinder is unbounded along its axis");
}
