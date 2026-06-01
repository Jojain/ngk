use nalgebra::Vector3;
use ngk::geometry::axis::Axis3;
use ngk::geometry::{Circle, Curve, LINEAR_TOLERANCE, Point3, PointCoincidence};

fn assert_point_near(actual: Point3, expected: Point3) {
    assert!(
        actual.coincides(expected, LINEAR_TOLERANCE),
        "expected {expected:?}, got {actual:?}"
    );
}

fn assert_vector_near(actual: Vector3<f64>, expected: Vector3<f64>, tol: f64) {
    let error = (actual - expected).norm();
    assert!(
        error <= tol,
        "expected {expected:?}, got {actual:?}, error {error}"
    );
}

#[test]
fn circle_from_axis_handles_z_axis() {
    let circle = Circle::from_axis(Axis3::new(Point3::new(1.0, 2.0, 3.0), Vector3::z()), 2.0);

    assert_point_near(circle.point_at(0.0), Point3::new(-1.0, 2.0, 3.0));
}

#[test]
fn line_curve_converts_to_matching_nurbs_curve() {
    let curve = Curve::line(Axis3::from_points(
        Point3::new(1.0, 2.0, 3.0),
        Point3::new(4.0, 6.0, 8.0),
    ));
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

#[test]
fn curve_derivative_dispatches_to_analytic_and_nurbs_curves() {
    let line = Curve::line(Axis3::from_points(
        Point3::new(1.0, 2.0, 3.0),
        Point3::new(4.0, 6.0, 8.0),
    ));
    assert_vector_near(
        line.derivative_at(0.4, 1),
        Vector3::new(3.0, 4.0, 5.0),
        1e-12,
    );
    assert_vector_near(line.derivative_at(0.4, 2), Vector3::zeros(), 1e-12);

    let circle = Curve::Circle(Circle::new(
        ngk::geometry::Plane::new(Point3::origin(), Vector3::x(), Vector3::z()),
        2.0,
    ));
    assert_vector_near(
        circle.derivative_at(0.0, 1),
        Vector3::new(0.0, 2.0, 0.0),
        1e-12,
    );
    assert_vector_near(
        circle.derivative_at(0.0, 2),
        Vector3::new(-2.0, 0.0, 0.0),
        1e-12,
    );

    let nurbs = line.to_nurbs().unwrap();
    let nurbs_curve = Curve::Nurbs(nurbs);
    assert_vector_near(
        nurbs_curve.derivative_at(0.4, 1),
        Vector3::new(3.0, 4.0, 5.0),
        1e-12,
    );
}

#[test]
fn nurbs_circle_length_matches_analytic_circle_length() {
    let circle = Circle::new(
        ngk::geometry::Plane::new(Point3::origin(), Vector3::x(), Vector3::z()),
        2.5,
    );
    let curve = Curve::Nurbs(circle.to_nurbs().unwrap());

    let length = curve.length(0.0, std::f64::consts::TAU);

    assert!((length - 2.5 * std::f64::consts::TAU).abs() < 1e-7);
}
