use nalgebra::Vector3;
use ngk::geometry::axis::Axis3;
use ngk::geometry::{Circle, Curve, LINEAR_TOLERANCE, Point3, PointCoincidence};

fn assert_point_near(actual: Point3, expected: Point3) {
    assert!(
        actual.coincides(expected, LINEAR_TOLERANCE),
        "expected {expected:?}, got {actual:?}"
    );
}

#[test]
fn circle_from_axis_handles_z_axis() {
    let circle = Circle::from_axis(Axis3::new(Point3::new(1.0, 2.0, 3.0), Vector3::z()), 2.0);

    assert_point_near(circle.point_at(0.0), Point3::new(-1.0, 2.0, 3.0));
}

#[test]
fn line_curve_converts_to_matching_nurbs_curve() {
    let curve = Curve::line(Point3::new(1.0, 2.0, 3.0), Point3::new(4.0, 6.0, 8.0));
    let nurbs = curve.to_nurbs().unwrap();

    assert_eq!(nurbs.degree().get(), 1);
    assert_point_near(nurbs.point_at(0.0), curve.point_at(0.0));
    assert_point_near(nurbs.point_at(0.25), curve.point_at(0.25));
    assert_point_near(nurbs.point_at(1.0), curve.point_at(1.0));
}

#[test]
fn circle_curve_converts_to_matching_rational_nurbs_curve() {
    let curve = Curve::Circle(Circle::new(
        ngk::geometry::Plane::new(Point3::new(1.0, 2.0, 3.0), Vector3::x(), Vector3::z()),
        2.5,
    ));
    let nurbs = curve.to_nurbs().unwrap();

    assert_eq!(nurbs.degree().get(), 2);
    assert!(nurbs.is_rational());
    for t in [
        0.0,
        std::f64::consts::FRAC_PI_4,
        std::f64::consts::FRAC_PI_2,
        std::f64::consts::PI,
        std::f64::consts::TAU,
    ] {
        assert_point_near(nurbs.point_at(t), curve.point_at(t));
    }
}
