use nalgebra::{Rotation3, Vector3};
use ngk::geometry::axis::Axis3;
use ngk::geometry::{
    Circle, Curve, Ellipse, Frame, Helix, Interval, LINEAR_TOLERANCE, Line, NativeParam, Plane,
    Point3, PointCoincidence, Rigid,
};
use radians::Rad64;

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

    assert_point_near(
        circle.point_at(NativeParam::new(0.0)),
        Point3::new(-1.0, 2.0, 3.0),
    );
}

#[test]
fn line_curve_converts_to_matching_nurbs_curve() {
    let curve = Curve::line(Point3::new(1.0, 2.0, 3.0), Point3::new(4.0, 6.0, 8.0));
    let nurbs = curve.to_nurbs().unwrap();

    assert_eq!(nurbs.degree().get(), 1);
    assert_point_near(nurbs.point_at(0.0), curve.point_at(NativeParam::new(0.0)));
    assert_point_near(nurbs.point_at(0.25), curve.point_at(NativeParam::new(0.25)));
    assert_point_near(nurbs.point_at(1.0), curve.point_at(NativeParam::new(1.0)));
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
        assert_point_near(nurbs.point_at(t), curve.point_at(NativeParam::new(t)));
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
        line.derivative_at(NativeParam::new(0.4), 1),
        Vector3::new(3.0, 4.0, 5.0),
        1e-12,
    );
    assert_vector_near(
        line.derivative_at(NativeParam::new(0.4), 2),
        Vector3::zeros(),
        1e-12,
    );

    let circle = Curve::Circle(Circle::new(
        ngk::geometry::Plane::new(Point3::origin(), Vector3::x(), Vector3::z()),
        2.0,
    ));
    assert_vector_near(
        circle.derivative_at(NativeParam::new(0.0), 1),
        Vector3::new(0.0, 2.0, 0.0),
        1e-12,
    );
    assert_vector_near(
        circle.derivative_at(NativeParam::new(0.0), 2),
        Vector3::new(-2.0, 0.0, 0.0),
        1e-12,
    );

    let nurbs = line.to_nurbs().unwrap();
    let nurbs_curve = Curve::Nurbs(nurbs);
    assert_vector_near(
        nurbs_curve.derivative_at(NativeParam::new(0.4), 1),
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

    let length = curve.length(
        NativeParam::new(0.0),
        NativeParam::new(std::f64::consts::TAU),
    );

    assert!((length - 2.5 * std::f64::consts::TAU).abs() < 1e-7);
}

#[test]
fn circle_native_interval_converts_to_normalized_nurbs_segment() {
    let circle = Curve::Circle(Circle::new(
        ngk::geometry::Plane::new(Point3::origin(), Vector3::x(), Vector3::z()),
        2.0,
    ));
    let Curve::Nurbs(nurbs) = circle.trimmed_native(Interval::new(0.25, 1.75)).unwrap() else {
        panic!("native trimming should produce a NURBS segment");
    };

    assert_eq!(nurbs.domain(), Interval::new(0.0, 1.0));
    assert_point_near(nurbs.point_at(0.0), circle.point_at(NativeParam::new(0.25)));
    assert_point_near(nurbs.point_at(1.0), circle.point_at(NativeParam::new(1.75)));
}

#[test]
fn arc_spanning_more_than_half_a_turn_reports_its_own_span() {
    let plane = Plane::new(Point3::origin(), Vector3::x(), Vector3::z());
    let span = 3.0 * std::f64::consts::FRAC_PI_2;
    let arc = Curve::circle(plane, 1.0);

    let start = arc.point_at(NativeParam::new(0.0));
    let end = arc.point_at(NativeParam::new(span));
    let interval = arc.interval_between(start, end);

    // The end sits at -90 degrees on the circle's own atan2 branch. Reading it
    // back there would describe the complementary quarter instead of this arc.
    assert!((interval.start - 0.0).value().abs() <= 1.0e-9);
    assert!((interval.end - span).value().abs() <= 1.0e-9);

    let midpoint = arc.point_at(NativeParam::new(0.5 * span));
    let expected = 0.5 * span;
    assert!((midpoint.x - expected.cos()).abs() <= 1.0e-9);
    assert!((midpoint.y - expected.sin()).abs() <= 1.0e-9);
}

#[test]
fn moved_curve_keeps_its_parameterisation() {
    let axis = Axis3::new(Point3::origin(), Vector3::z());
    let curve = Curve::line(Point3::new(1.0, 0.0, 0.0), Point3::new(2.0, 0.0, 1.0));
    let rotated = curve.moved(&Rigid::rotation(axis, Rad64::QUARTER_TURN));

    for t in [0.0, 0.25, 1.0] {
        let expected = Rotation3::from_axis_angle(&axis.direction, std::f64::consts::FRAC_PI_2)
            * curve.point_at(NativeParam::new(t));
        assert!(
            (rotated.point_at(NativeParam::new(t)) - expected).norm() <= 1.0e-9,
            "rotating must not re-parameterise the curve"
        );
    }
}

#[test]
fn reversed_analytic_curve_preserves_support_and_flips_parameter_direction() {
    let curves = [
        Curve::circle(Plane::xy(), 2.0),
        Curve::Ellipse(Ellipse::new(Frame::xyz(), 3.0, 1.5)),
        Curve::line(Point3::new(-2.0, 1.0, 0.5), Point3::new(4.0, 3.0, 2.0)),
    ];

    for curve in curves {
        let reversed = curve.reversed();
        assert!(
            !matches!(reversed, Curve::Nurbs(_)),
            "reversing an analytic curve must not degrade it to NURBS"
        );
        for parameter in [0.0, 0.17, 0.63, 1.0] {
            assert_point_near(
                reversed.point_at(NativeParam::new(parameter)),
                curve.point_at(NativeParam::new(-parameter)),
            );
        }
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
    assert!(!segment.domain().is_finite());
    assert_eq!(
        segment.interval_between(Point3::origin(), Point3::new(3.0, 0.0, 0.0)),
        Interval::new(0.0, 1.0)
    );
}

#[test]
fn line_trims_a_span_anchored_outside_its_construction_vector() {
    // The support runs to infinity, so `[-1, 0]` names the stretch *before*
    // the construction vector, and the section over it is a real segment.
    let line = Curve::line(Point3::new(14.0, 0.0, 0.0), Point3::new(34.0, 0.0, 0.0));

    let section = line.trimmed_native(Interval::new(-1.0, 0.0)).unwrap();

    assert_point_near(
        section.point_at(NativeParam::new(0.0)),
        Point3::new(-6.0, 0.0, 0.0),
    );
    assert_point_near(
        section.point_at(NativeParam::new(1.0)),
        Point3::new(14.0, 0.0, 0.0),
    );
}

#[test]
fn helix_evaluates_with_axial_pitch_per_turn() {
    let helix = Helix::new(Frame::xyz(), 2.0, 6.0);

    assert_point_near(
        helix.point_at(NativeParam::new(0.0)),
        Point3::new(2.0, 0.0, 0.0),
    );
    assert_point_near(
        helix.point_at(NativeParam::new(std::f64::consts::TAU)),
        Point3::new(2.0, 0.0, 6.0),
    );
    assert_vector_near(
        helix.derivative_at(NativeParam::new(0.0), 1),
        Vector3::new(0.0, 2.0, 6.0 / std::f64::consts::TAU),
        1.0e-12,
    );
}

#[test]
fn helix_parameter_and_projection_are_consistent() {
    let helix = Helix::new(Frame::xyz(), 2.0, 6.0);
    let parameter = NativeParam::new(1.25 * std::f64::consts::TAU);
    let point = helix.point_at(parameter);

    assert!((helix.parameter_at(point).value() - parameter.value()).abs() <= 1.0e-10);
    assert_point_near(helix.project(point), point);
}

#[test]
fn reversed_helix_flips_parameter_direction_without_changing_points() {
    let helix = Helix::new(Frame::xyz(), 2.0, 6.0);
    let reversed = Curve::Helix(helix.clone()).reversed();

    for parameter in [0.0, 0.3, 1.7, 2.0 * std::f64::consts::PI] {
        assert_point_near(
            reversed.point_at(NativeParam::new(parameter)),
            helix.point_at(NativeParam::new(-parameter)),
        );
    }
}

#[test]
fn helix_is_unbounded_and_has_exact_finite_span_bounds() {
    let helix = Curve::Helix(Helix::new(Frame::xyz(), 2.0, 6.0));
    assert!(!helix.domain().is_finite());

    let bbox = helix
        .bbox_over(Interval::new(0.0, std::f64::consts::TAU))
        .expect("finite helix spans have a bound");
    assert!((bbox.z_size() - 6.0).abs() <= 1.0e-12);
    assert!((bbox.x_size() - 4.0).abs() <= 1.0e-12);
    assert!((bbox.y_size() - 4.0).abs() <= 1.0e-12);
}
