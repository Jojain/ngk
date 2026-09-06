use nalgebra::{Rotation3, Vector3};
use ngk::geometry::axis::Axis3;
use ngk::geometry::{
    Bounded, Circle, Curve, Interval, LINEAR_TOLERANCE, Line, Plane, Point3, PointCoincidence,
};

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

#[test]
fn circle_nurbs_conversion_stays_on_the_circle_between_knots() {
    let plane = Plane::new(Point3::new(1.0, 2.0, 3.0), Vector3::x(), Vector3::z());
    let circle = Circle::new(plane, 2.5);
    let nurbs = Curve::Circle(circle.clone()).to_nurbs().unwrap();

    // The rational quadratic reproduces the circle as a point set, but its
    // parameter is a projective — not linear — function of the angle, so the
    // invariant that holds off-knot is membership, not `point_at` agreement.
    // Every parameter here is deliberately neither a knot nor a span midpoint.
    for t in [0.3, 0.9, 1.7, 2.9, 4.4, 5.8] {
        let radius = (nurbs.point_at(t) - circle.plane().origin()).norm();
        assert!(
            (radius - 2.5).abs() <= 1.0e-9,
            "point at {t} sits at radius {radius}, not on the circle"
        );
    }
}

#[test]
fn curve_derivative_dispatches_to_analytic_and_nurbs_curves() {
    let line = Curve::line(Point3::new(1.0, 2.0, 3.0), Point3::new(4.0, 6.0, 8.0));
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

#[test]
fn bounded_circle_converts_to_trimmed_nurbs_curve() {
    let circle = Curve::Circle(Circle::new(
        ngk::geometry::Plane::new(Point3::origin(), Vector3::x(), Vector3::z()),
        2.0,
    ));
    let bounded = Curve::Bounded(Box::new(Bounded::new(
        circle.clone(),
        Interval::new(0.25, 1.75),
    )));

    let nurbs = bounded.to_nurbs().unwrap();

    assert_eq!(nurbs.domain(), Interval::new(0.25, 1.75));
    assert_point_near(nurbs.point_at(0.25), circle.point_at(0.25));
    assert_point_near(nurbs.point_at(1.75), circle.point_at(1.75));
}

#[test]
fn arc_spanning_more_than_half_a_turn_reports_its_own_span() {
    let plane = Plane::new(Point3::origin(), Vector3::x(), Vector3::z());
    let span = 3.0 * std::f64::consts::FRAC_PI_2;
    let arc = Curve::arc(plane, 1.0, Interval::new(0.0, span));

    let start = arc.point_at(0.0);
    let end = arc.point_at(1.0);
    let interval = arc.parameters_between(start, end);

    // The end sits at -90 degrees on the circle's own atan2 branch. Reading it
    // back there would describe the complementary quarter instead of this arc.
    assert!((interval.start - 0.0).abs() <= 1.0e-9);
    assert!((interval.end - 1.0).abs() <= 1.0e-9);

    let midpoint = arc.point_at(0.5);
    let expected = 0.5 * span;
    assert!((midpoint.x - expected.cos()).abs() <= 1.0e-9);
    assert!((midpoint.y - expected.sin()).abs() <= 1.0e-9);
}

#[test]
fn rotated_curve_keeps_its_parameterisation() {
    let axis = Axis3::new(Point3::origin(), Vector3::z());
    let curve = Curve::line(Point3::new(1.0, 0.0, 0.0), Point3::new(2.0, 0.0, 1.0));
    let rotated = curve
        .rotated(axis, std::f64::consts::FRAC_PI_2)
        .expect("a bounded line should rotate");

    for t in [0.0, 0.25, 1.0] {
        let expected = Rotation3::from_axis_angle(&axis.direction, std::f64::consts::FRAC_PI_2)
            * curve.point_at(t);
        assert!(
            (rotated.point_at(t) - expected).norm() <= 1.0e-9,
            "rotating must not re-parameterise the curve"
        );
    }
}

#[test]
fn circle_curve_projects_onto_the_nearest_circle_point() {
    let curve = Curve::Circle(Circle::new(
        Plane::new(Point3::origin(), Vector3::x(), Vector3::z()),
        2.5,
    ));

    assert_point_near(
        curve.project(Point3::new(5.0, 0.0, 3.0)),
        Point3::new(2.5, 0.0, 0.0),
    );
    // A point on the axis is equidistant from every point of the circle, so
    // the plane's x direction is returned to keep the result deterministic.
    assert_point_near(
        curve.project(Point3::new(0.0, 0.0, 4.0)),
        Point3::new(2.5, 0.0, 0.0),
    );
}

#[test]
fn nurbs_curve_projects_onto_the_nearest_curve_point() {
    let circle = Circle::new(
        Plane::new(Point3::origin(), Vector3::x(), Vector3::z()),
        2.5,
    );
    let curve = Curve::Nurbs(circle.to_nurbs().unwrap());

    // The NURBS projection is a sampled seed refined by Newton, so it converges
    // to the analytic answer rather than reproducing it exactly.
    assert_vector_near(
        curve.project(Point3::new(5.0, 0.0, 3.0)).coords,
        Point3::new(2.5, 0.0, 0.0).coords,
        1.0e-7,
    );
}

#[test]
fn curve_domains_distinguish_bounded_from_unbounded_supports() {
    let line = Curve::Line(Line::new(Axis3::new(Point3::origin(), Vector3::x())));
    assert!(
        !line.domain().is_finite(),
        "an untrimmed line extends without bound"
    );

    let circle = Curve::Circle(Circle::new(
        Plane::new(Point3::origin(), Vector3::x(), Vector3::z()),
        1.0,
    ));
    assert_eq!(circle.domain(), Interval::new(0.0, std::f64::consts::TAU));

    let segment = Curve::line(Point3::origin(), Point3::new(3.0, 0.0, 0.0));
    assert_eq!(segment.domain(), Interval::new(0.0, 1.0));
}
