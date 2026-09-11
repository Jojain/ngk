use ngk::geometry::{
    Circle2, ControlPolygon2, Curve2, CurveCurveIntersection2, Degree, Ellipse2, HPoint2, Interval,
    KnotVector, LINEAR_TOLERANCE, Line2, NurbsCurve2, Point2, TrimmedCurve2, Vector2,
};
use std::f64::consts::{FRAC_PI_2, PI, TAU};

fn assert_point2_close(actual: Point2, expected: Point2) {
    assert!(
        (actual - expected).norm() <= LINEAR_TOLERANCE,
        "expected {expected:?}, got {actual:?}"
    );
}

#[test]
fn line2_counts_its_construction_vector_and_extrapolates_past_it() {
    let line = Line2::new(Point2::new(1.0, 1.0), Vector2::new(2.0, 0.0));

    assert_point2_close(line.point_at(0.0), Point2::new(1.0, 1.0));
    assert_point2_close(line.point_at(1.0), Point2::new(3.0, 1.0));
    // The support is infinite: parameters outside the construction vector are
    // every bit as valid as the ones inside it.
    assert_point2_close(line.point_at(-2.0), Point2::new(-3.0, 1.0));
    assert_point2_close(line.point_at(4.5), Point2::new(10.0, 1.0));
    assert!(!Curve2::Line(line).domain().is_finite());
}

#[test]
fn line2_from_a_zero_vector_collapses_rather_than_going_non_finite() {
    let point = Point2::new(1.0, 1.0);
    let line = Line2::new(point, Vector2::zeros());

    // No line exists, so the support is the constant point. What matters is
    // that a caller measuring it sees a plainly wrong curve, not `NaN`.
    for parameter in [-3.0, 0.0, 1.0, 7.5] {
        assert_point2_close(line.point_at(parameter), point);
    }
}

#[test]
fn arc_helpers_anchor_the_support_and_span_the_sweep() {
    let arc = TrimmedCurve2::arc(Point2::new(1.0, 1.0), Vector2::x(), 2.0, FRAC_PI_2);

    assert_eq!(arc.interval(), Interval::new(0.0, FRAC_PI_2));
    assert_point2_close(arc.start(), Point2::new(3.0, 1.0));
    assert_point2_close(arc.end(), Point2::new(1.0, 3.0));
    assert!(matches!(arc.curve(), Curve2::Circle(_)));

    // A negative sweep is the other way round the same support.
    let back = TrimmedCurve2::arc(Point2::new(1.0, 1.0), Vector2::x(), 2.0, -FRAC_PI_2);
    assert_point2_close(back.end(), Point2::new(1.0, -1.0));
    assert_eq!(back.curve(), arc.curve());

    let ellipse = TrimmedCurve2::ellipse_arc(Point2::origin(), Vector2::x(), 3.0, 1.0, PI);
    assert_eq!(ellipse.interval(), Interval::new(0.0, PI));
    assert_point2_close(ellipse.start(), Point2::new(3.0, 0.0));
    assert_point2_close(ellipse.end(), Point2::new(-3.0, 0.0));
    assert_point2_close(ellipse.point_at(0.5), Point2::new(0.0, 1.0));
}

#[test]
fn circle2_support_closes_and_is_parameterized_by_angle() {
    let circle = Circle2::new(Point2::new(2.0, 3.0), Vector2::x(), 2.0);

    assert_point2_close(circle.point_at(0.0), Point2::new(4.0, 3.0));
    assert_point2_close(circle.point_at(FRAC_PI_2), Point2::new(2.0, 5.0));
    assert_point2_close(circle.point_at(TAU), circle.point_at(0.0));
    assert!((circle.param_at(Point2::new(2.0, 5.0)) - FRAC_PI_2).abs() <= LINEAR_TOLERANCE);
    assert_eq!(Curve2::Circle(circle).domain(), Interval::new(0.0, TAU));
}

#[test]
fn ellipse2_support_is_parameterized_by_eccentric_angle() {
    let ellipse = Ellipse2::new(Point2::new(1.0, -2.0), Vector2::y(), 4.0, 2.0);

    assert_point2_close(ellipse.point_at(0.0), Point2::new(1.0, 2.0));
    assert_point2_close(ellipse.point_at(FRAC_PI_2), Point2::new(-1.0, -2.0));
    assert_point2_close(ellipse.point_at(TAU), ellipse.point_at(0.0));

    let curve = Curve2::Ellipse(ellipse.clone());
    let nurbs = curve.to_nurbs().expect("ellipse should convert exactly");
    assert_eq!(nurbs.degree().get(), 2);
    assert!(
        nurbs
            .control_points()
            .as_slice()
            .iter()
            .any(|point| (point.weight() - 1.0).abs() > 1.0e-12)
    );
    for parameter in [0.07, 0.31, 0.58, 0.89] {
        let point = nurbs.point_at(parameter);
        let offset = point - ellipse.center();
        let x = offset.dot(ellipse.x_dir().as_ref()) / ellipse.major_radius();
        let y = offset.dot(ellipse.y_dir().as_ref()) / ellipse.minor_radius();
        assert!((x * x + y * y - 1.0).abs() <= 1.0e-9);
    }
}

#[test]
fn trimmed_curve2_traverses_its_span_from_zero_to_one() {
    let quarter = TrimmedCurve2::arc(Point2::origin(), Vector2::x(), 1.0, FRAC_PI_2);

    assert_point2_close(quarter.start(), Point2::new(1.0, 0.0));
    assert_point2_close(quarter.end(), Point2::new(0.0, 1.0));
    assert_point2_close(
        quarter.point_at(0.5),
        Point2::new((PI / 4.0).cos(), (PI / 4.0).sin()),
    );
    assert!((quarter.length() - FRAC_PI_2).abs() <= LINEAR_TOLERANCE);
}

#[test]
fn trimmed_curve2_distinguishes_the_minor_arc_from_the_major_one() {
    let support = Curve2::circle(Point2::origin(), Vector2::x(), 1.0);
    let minor = TrimmedCurve2::new(support.clone(), Interval::new(0.0, FRAC_PI_2));
    let major = TrimmedCurve2::new(support, Interval::new(0.0, -3.0 * FRAC_PI_2));

    // Both arcs share their endpoints; only the span tells them apart.
    assert_point2_close(minor.start(), major.start());
    assert_point2_close(minor.end(), major.end());
    assert_point2_close(
        minor.point_at(0.5),
        Point2::from(Vector2::new(1.0, 1.0).normalize()),
    );
    assert_point2_close(
        major.point_at(0.5),
        Point2::from(Vector2::new(-1.0, -1.0).normalize()),
    );
}

#[test]
fn trimmed_curve2_contains_answers_for_the_span_not_the_support() {
    let span = TrimmedCurve2::segment(Point2::new(0.0, 0.0), Point2::new(2.0, 0.0));

    assert!(span.contains(Point2::new(1.0, 0.0), LINEAR_TOLERANCE));
    // On the support's line, but far outside the segment that is meant.
    assert!(!span.contains(Point2::new(9.0, 0.0), LINEAR_TOLERANCE));
    assert!(!span.contains(Point2::new(1.0, 1.0), LINEAR_TOLERANCE));
    assert_eq!(
        span.try_parameter_at(Point2::new(9.0, 0.0), LINEAR_TOLERANCE),
        None
    );
    let fraction = span
        .try_parameter_at(Point2::new(0.5, 0.0), LINEAR_TOLERANCE)
        .expect("a point on the span has a fraction");
    assert!((fraction - 0.25).abs() <= LINEAR_TOLERANCE);
}

#[test]
fn trimmed_curve2_reverse_and_sub_keep_the_analytic_support() {
    let span = TrimmedCurve2::arc(Point2::new(1.0, 2.0), Vector2::x(), 3.0, PI);
    let reversed = span.reversed();
    let (first, second) = span.split_at(0.4);

    for index in 0..=10 {
        let fraction = index as f64 / 10.0;
        assert_point2_close(reversed.point_at(fraction), span.point_at(1.0 - fraction));
    }
    assert_point2_close(first.point_at(1.0), span.point_at(0.4));
    assert_point2_close(second.point_at(0.0), span.point_at(0.4));
    // Narrowing never converts: every fragment still rests on the circle.
    for fragment in [&reversed, &first, &second] {
        assert!(matches!(fragment.curve(), Curve2::Circle(_)));
    }
}

#[test]
fn trimmed_curve2_to_curve_reproduces_the_span_exactly() {
    let span = TrimmedCurve2::new(
        Curve2::circle(Point2::new(-1.0, 2.0), Vector2::x(), 3.0),
        Interval::new(0.3, 0.3 + 1.4 * PI),
    );
    let cut_down = span.to_curve().expect("an arc converts exactly");

    // The cut-down copy is no longer the circle, but it is the same point set.
    // Its parameterization is not the arc's: a conic is not a rational
    // function of its angle, so only the ends are shared parameter for
    // parameter, and the interior is checked as a set instead.
    assert!(matches!(cut_down, Curve2::Nurbs(_)));
    assert_point2_close(cut_down.point_at(0.0), span.start());
    assert_point2_close(cut_down.point_at(1.0), span.end());
    for index in 0..=16 {
        let point = cut_down.point_at(index as f64 / 16.0);
        assert!(
            span.contains(point, LINEAR_TOLERANCE),
            "{point:?} should lie on the arc it was cut from"
        );
    }
}

#[test]
fn trimmed_curve2_translation_preserves_the_span() {
    let span = TrimmedCurve2::ellipse_arc(Point2::new(1.0, 2.0), Vector2::x(), 3.0, 1.5, 1.7 * PI);
    let offset = Vector2::new(4.0, -3.0);
    let translated = span.translated(offset).unwrap();

    assert_eq!(translated.interval(), span.interval());
    for fraction in [0.0, 0.23, 0.67, 1.0] {
        assert_point2_close(
            translated.point_at(fraction),
            span.point_at(fraction) + offset,
        );
    }
}

#[test]
fn line2_span_splits_into_two_spans_sharing_the_split_point() {
    let span = TrimmedCurve2::segment(Point2::new(0.0, 0.0), Point2::new(2.0, 0.0));

    let (first, second) = span.split_at(0.25);

    assert_point2_close(first.start(), Point2::new(0.0, 0.0));
    assert_point2_close(first.end(), Point2::new(0.5, 0.0));
    assert_point2_close(second.start(), Point2::new(0.5, 0.0));
    assert_point2_close(second.end(), Point2::new(2.0, 0.0));
}

#[test]
fn circle2_converts_to_exact_rational_nurbs_geometry() {
    let circle = Curve2::circle(Point2::new(-1.0, 2.0), Vector2::x(), 3.0);
    let nurbs = circle.to_nurbs().unwrap();
    let domain = nurbs.domain();

    for fraction in [0.0, 0.25, 0.5, 0.75, 1.0] {
        assert_point2_close(
            nurbs.point_at(domain.at(fraction)),
            circle.point_at(TAU * fraction),
        );
    }
}

#[test]
fn rational_nurbs_curve2_evaluates_quarter_circle() {
    let weight = std::f64::consts::FRAC_1_SQRT_2;
    let curve = NurbsCurve2::new(
        Degree::new(2).unwrap(),
        ControlPolygon2::new(vec![
            HPoint2::from_cartesian(Point2::new(1.0, 0.0), 1.0),
            HPoint2::from_cartesian(Point2::new(1.0, 1.0), weight),
            HPoint2::from_cartesian(Point2::new(0.0, 1.0), 1.0),
        ])
        .unwrap(),
        KnotVector::new(vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0]).unwrap(),
    )
    .unwrap();

    let midpoint = curve.point_at(0.5);
    assert_point2_close(midpoint, Point2::new(weight, weight));
}

#[test]
fn nurbs_support_keeps_its_own_native_domain() {
    let curve = NurbsCurve2::new(
        Degree::new(1).unwrap(),
        ControlPolygon2::from_cartesian(
            vec![Point2::new(0.0, 0.0), Point2::new(4.0, 0.0)],
            &[1.0, 1.0],
        )
        .unwrap(),
        KnotVector::new(vec![2.0, 2.0, 6.0, 6.0]).unwrap(),
    )
    .unwrap();
    let support = Curve2::Nurbs(curve);

    assert_eq!(support.domain(), Interval::new(2.0, 6.0));
    // The native parameter is the knot parameter, not a normalized fraction.
    assert_point2_close(support.point_at(3.0), Point2::new(1.0, 0.0));
    let span = TrimmedCurve2::new(support.clone(), support.domain());
    assert_point2_close(span.point_at(0.25), Point2::new(1.0, 0.0));
}

#[test]
fn open_interpolation_passes_through_all_samples() {
    let points = vec![
        Point2::new(0.0, 0.0),
        Point2::new(1.0, 2.0),
        Point2::new(3.0, 2.0),
        Point2::new(4.0, 0.0),
    ];
    let curve = NurbsCurve2::interpolate(&points).expect("samples should interpolate");

    for (point, parameter) in points.iter().zip(curve.interpolation_parameters()) {
        assert_point2_close(curve.point_at(*parameter), *point);
    }
}

#[test]
fn closed_interpolation_has_matching_position_and_tangent_at_seam() {
    let points = vec![
        Point2::new(0.0, 0.0),
        Point2::new(2.0, 0.0),
        Point2::new(2.0, 2.0),
        Point2::new(0.0, 2.0),
        Point2::new(0.0, 0.0),
    ];
    let curve = NurbsCurve2::interpolate(&points).expect("closed samples should interpolate");
    let domain = curve.domain();

    assert_point2_close(curve.point_at(domain.start), curve.point_at(domain.end));
    let start_tangent = curve.derivative_at(domain.start, 1).normalize();
    let end_tangent = curve.derivative_at(domain.end, 1).normalize();
    assert!((start_tangent - end_tangent).norm() <= 1.0e-8);
}

#[test]
fn nurbs_curve2_span_reverse_and_split_preserve_geometry() {
    let span = whole_span(
        NurbsCurve2::interpolate(&[
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 1.5),
            Point2::new(3.0, 1.0),
            Point2::new(4.0, 0.0),
        ])
        .unwrap(),
    );
    let reversed = span.reversed();
    for i in 0..=10 {
        let t = i as f64 / 10.0;
        assert_point2_close(reversed.point_at(t), span.point_at(1.0 - t));
    }

    let split_point = span.point_at(0.4);
    let (first, second) = span.split_at(0.4);
    assert_point2_close(first.point_at(1.0), split_point);
    assert_point2_close(second.point_at(0.0), split_point);
}

#[test]
fn nurbs_span_recovers_the_fraction_of_a_point_on_it() {
    let span = whole_span(
        NurbsCurve2::interpolate(&[
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 2.0),
            Point2::new(3.0, 1.0),
            Point2::new(4.0, 0.0),
        ])
        .unwrap(),
    );
    let point = span.point_at(0.37);
    let recovered = span
        .try_parameter_at(point, LINEAR_TOLERANCE)
        .expect("point on the span should have a fraction");

    assert!((recovered - 0.37).abs() <= 1.0e-5);
}

#[test]
fn span_line_intersection_returns_point_and_parameters() {
    let horizontal = TrimmedCurve2::segment(Point2::new(0.0, 0.0), Point2::new(2.0, 0.0));
    let vertical = TrimmedCurve2::segment(Point2::new(0.5, -1.0), Point2::new(0.5, 1.0));

    let intersections = horizontal.intersect_curve(&vertical).unwrap();

    assert_eq!(intersections.len(), 1, "{intersections:?}");
    let CurveCurveIntersection2::Point { point, u_a, u_b } = intersections[0] else {
        panic!("expected point intersection, got {intersections:?}");
    };
    assert_point2_close(point, Point2::new(0.5, 0.0));
    assert!((u_a - 0.25).abs() <= LINEAR_TOLERANCE);
    assert!((u_b - 0.5).abs() <= LINEAR_TOLERANCE);
}

#[test]
fn collinear_spans_return_overlap_intervals() {
    let outer = TrimmedCurve2::segment(Point2::new(0.0, 0.0), Point2::new(3.0, 0.0));
    let inner = TrimmedCurve2::segment(Point2::new(1.0, 0.0), Point2::new(2.0, 0.0));

    let intersections = outer.intersect_curve(&inner).unwrap();

    assert_eq!(intersections.len(), 1, "{intersections:?}");
    let CurveCurveIntersection2::Overlap {
        interval_a,
        interval_b,
    } = intersections[0]
    else {
        panic!("expected overlap, got {intersections:?}");
    };
    assert!((interval_a.start - 1.0 / 3.0).abs() <= LINEAR_TOLERANCE);
    assert!((interval_a.end - 2.0 / 3.0).abs() <= LINEAR_TOLERANCE);
    assert!(interval_b.start.abs() <= LINEAR_TOLERANCE);
    assert!((interval_b.end - 1.0).abs() <= LINEAR_TOLERANCE);
}

#[test]
fn tangent_quadratic_nurbs_spans_return_one_point() {
    let rising = whole_span(quadratic_curve2([
        Point2::new(0.0, 0.0),
        Point2::new(1.0, 1.0),
        Point2::new(2.0, 0.0),
    ]));
    let falling = whole_span(quadratic_curve2([
        Point2::new(0.0, 1.0),
        Point2::new(1.0, 0.0),
        Point2::new(2.0, 1.0),
    ]));

    let intersections = rising.intersect_curve(&falling).unwrap();

    assert_eq!(intersections.len(), 1, "{intersections:?}");
    let CurveCurveIntersection2::Point { point, u_a, u_b } = intersections[0] else {
        panic!("expected point intersection, got {intersections:?}");
    };
    assert_point2_close(point, Point2::new(1.0, 0.5));
    assert!((u_a - 0.5).abs() <= LINEAR_TOLERANCE * 10.0);
    assert!((u_b - 0.5).abs() <= LINEAR_TOLERANCE * 10.0);
}

#[test]
fn transverse_quadratic_nurbs_spans_return_crossing_point() {
    let rising = whole_span(quadratic_curve2([
        Point2::new(0.0, 0.0),
        Point2::new(1.0, 1.0),
        Point2::new(2.0, 2.0),
    ]));
    let falling = whole_span(quadratic_curve2([
        Point2::new(0.0, 2.0),
        Point2::new(1.0, 1.0),
        Point2::new(2.0, 0.0),
    ]));

    let intersections = rising.intersect_curve(&falling).unwrap();

    assert_eq!(intersections.len(), 1, "{intersections:?}");
    let CurveCurveIntersection2::Point { point, u_a, u_b } = intersections[0] else {
        panic!("expected point intersection, got {intersections:?}");
    };
    assert_point2_close(point, Point2::new(1.0, 1.0));
    assert!((u_a - 0.5).abs() <= LINEAR_TOLERANCE * 10.0);
    assert!((u_b - 0.5).abs() <= LINEAR_TOLERANCE * 10.0);
}

#[test]
fn line_and_nurbs_secant_spans_return_two_points() {
    let arch = whole_span(quadratic_curve2([
        Point2::new(0.0, 0.0),
        Point2::new(1.0, 1.0),
        Point2::new(2.0, 0.0),
    ]));
    let line = TrimmedCurve2::segment(Point2::new(0.0, 0.25), Point2::new(2.0, 0.25));

    let intersections = arch.intersect_curve(&line).unwrap();
    let mut points = intersections
        .iter()
        .filter_map(|intersection| match intersection {
            CurveCurveIntersection2::Point { point, .. } => Some(*point),
            CurveCurveIntersection2::Overlap { .. } => None,
        })
        .collect::<Vec<_>>();
    points.sort_by(|left, right| left.x.total_cmp(&right.x));

    assert_eq!(points.len(), 2, "{intersections:?}");
    assert_point2_close(
        points[0],
        Point2::new(1.0 - std::f64::consts::FRAC_1_SQRT_2, 0.25),
    );
    assert_point2_close(
        points[1],
        Point2::new(1.0 + std::f64::consts::FRAC_1_SQRT_2, 0.25),
    );
}

#[test]
fn intersection_parameters_are_fractions_of_each_span() {
    let horizontal = whole_span(
        NurbsCurve2::new(
            Degree::new(1).unwrap(),
            ControlPolygon2::from_cartesian(
                vec![Point2::new(0.0, 0.0), Point2::new(4.0, 0.0)],
                &[1.0, 1.0],
            )
            .unwrap(),
            KnotVector::new(vec![2.0, 2.0, 6.0, 6.0]).unwrap(),
        )
        .unwrap(),
    );
    let vertical = TrimmedCurve2::segment(Point2::new(1.0, -1.0), Point2::new(1.0, 1.0));

    let intersections = horizontal.intersect_curve(&vertical).unwrap();

    let CurveCurveIntersection2::Point { u_a, u_b, .. } = intersections[0] else {
        panic!("expected point intersection, got {intersections:?}");
    };
    assert!((u_a - 0.25).abs() <= LINEAR_TOLERANCE);
    assert!((u_b - 0.5).abs() <= LINEAR_TOLERANCE);
}

/// Saves wrapping every NURBS test curve in its variant before spanning it.
fn whole_span(curve: NurbsCurve2) -> TrimmedCurve2 {
    TrimmedCurve2::whole(Curve2::Nurbs(curve))
}

fn quadratic_curve2(points: [Point2; 3]) -> NurbsCurve2 {
    NurbsCurve2::new(
        Degree::new(2).unwrap(),
        ControlPolygon2::new(
            points
                .into_iter()
                .map(|point| HPoint2::from_cartesian(point, 1.0))
                .collect(),
        )
        .unwrap(),
        KnotVector::new(vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0]).unwrap(),
    )
    .unwrap()
}
