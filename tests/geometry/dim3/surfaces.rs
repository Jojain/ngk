use std::f64::consts::FRAC_PI_2;

use nalgebra::{Rotation3, Vector3};
use ngk::geometry::{
    Circle, Curve, Cylinder, Fraction, Interval, LINEAR_TOLERANCE, Plane, Point2, Point3,
    PointCoincidence, Rigid, RuledSurface, Surface, SurfaceGeometry, SurfaceOfRevolution,
    SurfacePeriodicity, axis::Axis3,
};
use radians::Rad64;

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
        .param_at(Point3::new(4.0, 2.0, 8.0))
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
fn cylinder_param_map_matches_off_knot_analytic_parameters() {
    let surface = Surface::Cylinder(Cylinder::new(
        Point3::new(1.0, 2.0, 3.0),
        Vector3::x(),
        Vector3::z(),
        2.0,
    ));
    let u = Interval::new(0.23, 2.41);
    let v = Interval::new(-1.7, 4.2);
    let nurbs = surface.to_nurbs_over(u, v).unwrap();
    let map = surface.param_map_over(u, v);

    for (analytic_u, analytic_v) in [(0.37, -0.9), (1.14, 0.6), (2.19, 3.8)] {
        let mapped = map.map(Point2::new(analytic_u, analytic_v));
        assert_point_near(
            nurbs.point_at(mapped.x, mapped.y),
            surface.point_at(analytic_u, analytic_v),
        );
        let recovered = map.inverse(mapped);
        assert!((recovered.x - analytic_u).abs() <= 1.0e-12);
        assert!((recovered.y - analytic_v).abs() <= 1.0e-12);
    }
}

#[test]
fn cylinder_param_map_handles_shifted_and_reversed_full_turns() {
    let surface = Surface::Cylinder(Cylinder::new(
        Point3::origin(),
        Vector3::x(),
        Vector3::z(),
        1.7,
    ));
    for u in [
        Interval::new(0.23, 0.23 + std::f64::consts::TAU),
        Interval::new(2.4, -0.7),
    ] {
        let v = Interval::new(-1.0, 2.0);
        let nurbs = surface.to_nurbs_over(u, v).unwrap();
        let map = surface.param_map_over(u, v);
        for fraction in [0.0, 0.17, 0.43, 0.79, 1.0] {
            let analytic_u = u.start + (u.end - u.start) * fraction;
            let mapped = map.map(Point2::new(analytic_u.value(), 0.37));
            assert_point_near(
                nurbs.point_at(mapped.x, mapped.y),
                surface.point_at(analytic_u.value(), 0.37),
            );
            let recovered = map.inverse(mapped).x;
            assert!(
                (recovered - analytic_u.value()).abs() <= 1.0e-11,
                "interval {u:?}, fraction {fraction}: mapped {}, recovered {recovered}, expected {analytic_u}",
                mapped.x,
            );
        }
    }
}

#[test]
fn cylinder_bbox_over_contains_a_trimmed_patch() {
    let cylinder = Cylinder::new(
        Point3::new(1.0, 2.0, 3.0),
        Vector3::y(),
        Vector3::new(1.0, 0.0, 1.0),
        2.3,
    );
    let u = Interval::new(0.31, 4.77);
    let v = Interval::new(-2.4, 5.1);
    let bounds = cylinder
        .bbox_over(u, v)
        .expect("a finite cylinder patch has exact bounds");

    for iu in 0..=64 {
        for iv in 0..=8 {
            let parameter_u = u.at(Fraction::new(iu as f64 / 64.0)).value();
            let parameter_v = v.at(Fraction::new(iv as f64 / 8.0)).value();
            assert!(
                bounds.contains_point(
                    cylinder.point_at(parameter_u, parameter_v),
                    LINEAR_TOLERANCE,
                ),
                "cylinder point ({parameter_u}, {parameter_v}) escaped its bounds"
            );
        }
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
fn moved_surface_keeps_its_parameterisation() {
    let axis = Axis3::new(Point3::origin(), Vector3::z());
    let surface = Surface::Plane(Plane::from_xy(
        Point3::new(1.0, 0.0, 0.0),
        Vector3::x(),
        Vector3::z(),
    ));
    let rotated = surface.moved(&Rigid::rotation(axis, Rad64::QUARTER_TURN));

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

/// A revolution's parameters come back as the ones it was evaluated at.
///
/// The surface reads `v` as a true sweep angle, but its exact NURBS form carries
/// the sweep as a piecewise rational quadratic, whose parameter equals the angle
/// only at the quarter-turn knots — and carries a conic profile in `u` the same
/// way. Answering this by projecting onto that form returns the form's
/// parameters, which name a *different point* everywhere between the knots: on a
/// torus of tube radius 1 the miss reaches a twentieth of a unit, four orders
/// above any tolerance a model is fitted to. The mid-span samples below are the
/// whole point of the test; the knots agree either way.
#[test]
fn revolution_closest_parameter_inverts_its_own_evaluation() {
    let profile = Plane::new(Point3::new(3.0, 0.0, 0.0), Vector3::x(), Vector3::y());
    let torus = SurfaceOfRevolution::new(
        Curve::Circle(Circle::new(profile, 1.0)),
        Axis3::new(Point3::origin(), Vector3::z()),
    );
    let (u_domain, v_domain) = SurfaceGeometry::domain(&torus);

    let mut worst: f64 = 0.0;
    for i in 0..=12 {
        for j in 0..=12 {
            let (u, v) = (
                u_domain.at(Fraction::new(i as f64 / 12.0)),
                v_domain.at(Fraction::new(j as f64 / 12.0)),
            );
            let point = torus.point_at(u.value(), v.value());
            let uv = torus.closest_parameter(point);
            worst = worst.max((torus.point_at(uv.x, uv.y) - point).norm());
        }
    }
    assert!(
        worst <= LINEAR_TOLERANCE,
        "a revolution's parameters should name the point they were read from, missed by {worst:e}"
    );
}

/// The inversion crosses the seam rather than stopping at it.
///
/// Both of a torus's directions wrap, so a point a hair before the domain's end
/// is a hair away from one at its start. A projection clamped to the NURBS
/// patch answers the boundary for everything past it, which is exactly where a
/// traced intersection branch closes on itself.
#[test]
fn revolution_closest_parameter_reads_both_sides_of_the_seam() {
    let torus = SurfaceOfRevolution::new(
        Curve::Circle(Circle::new(
            Plane::new(Point3::new(3.0, 0.0, 0.0), Vector3::x(), Vector3::y()),
            1.0,
        )),
        Axis3::new(Point3::origin(), Vector3::z()),
    );
    let turn = std::f64::consts::TAU;

    for offset in [1.0e-3, 1.0e-2, 0.1] {
        for (u, v) in [(offset, 0.4), (turn - offset, 0.4), (0.4, turn - offset)] {
            let point = torus.point_at(u, v);
            let uv = torus.closest_parameter(point);
            assert!(
                (torus.point_at(uv.x, uv.y) - point).norm() <= LINEAR_TOLERANCE,
                "({u}, {v}) came back as {uv:?}, which is a different point"
            );
        }
    }
}
