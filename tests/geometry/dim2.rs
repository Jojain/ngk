use ngk::geometry::{
    Circle2, ControlPolygon2, Curve2, CurveCurveIntersection2, Degree, HPoint2, KnotVector,
    LINEAR_TOLERANCE, Line2, NurbsCurve2, Point2, Vector2,
};

fn assert_point2_close(actual: Point2, expected: Point2) {
    assert!(
        (actual - expected).norm() <= LINEAR_TOLERANCE,
        "expected {expected:?}, got {actual:?}"
    );
}

#[test]
fn line2_split_at_returns_two_lines_sharing_split_point() {
    let curve = Curve2::Line(Line2::new(Point2::new(0.0, 0.0), Point2::new(2.0, 0.0)));

    let (first, second) = curve.split_at(0.25).expect("interior split should succeed");

    let Curve2::Line(first) = first else {
        panic!("line should split into a line");
    };
    let Curve2::Line(second) = second else {
        panic!("line should split into a line");
    };
    assert_eq!(first.start, Point2::new(0.0, 0.0));
    assert_eq!(first.end, Point2::new(0.5, 0.0));
    assert_eq!(second.start, Point2::new(0.5, 0.0));
    assert_eq!(second.end, Point2::new(2.0, 0.0));
}

#[test]
fn circle2_uses_normalized_parameter_and_recovers_it() {
    let circle = Curve2::Circle(Circle2::new(
        Point2::new(2.0, 3.0),
        Vector2::x(),
        2.0,
        std::f64::consts::TAU,
    ));

    let quarter = circle.point_at(0.25);

    assert_point2_close(quarter, Point2::new(2.0, 5.0));
    let parameter = circle
        .parameter_at(quarter, LINEAR_TOLERANCE)
        .expect("a point on the circle should have a normalized parameter");
    assert!((parameter - 0.25).abs() <= LINEAR_TOLERANCE);
}

#[test]
fn circle2_reverse_and_split_preserve_geometry() {
    let circle = Curve2::Circle(Circle2::new(
        Point2::origin(),
        Vector2::x(),
        1.0,
        std::f64::consts::PI,
    ));
    let reversed = circle.reversed();
    let (first, second) = circle.split_at(0.4).unwrap();

    for index in 0..=10 {
        let parameter = index as f64 / 10.0;
        assert_point2_close(
            reversed.point_at(parameter),
            circle.point_at(1.0 - parameter),
        );
    }
    assert_point2_close(first.point_at(1.0), circle.point_at(0.4));
    assert_point2_close(second.point_at(0.0), circle.point_at(0.4));
}

#[test]
fn circle2_converts_to_exact_rational_nurbs_geometry() {
    let circle = Curve2::Circle(Circle2::new(
        Point2::new(-1.0, 2.0),
        Vector2::x(),
        3.0,
        std::f64::consts::TAU,
    ));
    let nurbs = circle.to_nurbs().unwrap();

    for parameter in [0.0, 0.25, 0.5, 0.75, 1.0] {
        assert_point2_close(nurbs.point_at(parameter), circle.point_at(parameter));
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
fn curve2_uses_normalized_parameter_over_native_nurbs_domain() {
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

    assert_point2_close(Curve2::Nurbs(curve).point_at(0.25), Point2::new(1.0, 0.0));
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
fn nurbs_curve2_reverse_and_split_preserve_geometry() {
    let curve = NurbsCurve2::interpolate(&[
        Point2::new(0.0, 0.0),
        Point2::new(1.0, 1.5),
        Point2::new(3.0, 1.0),
        Point2::new(4.0, 0.0),
    ])
    .unwrap();
    let reversed = curve.reversed();
    for i in 0..=10 {
        let t = i as f64 / 10.0;
        assert_point2_close(
            Curve2::Nurbs(reversed.clone()).point_at(t),
            Curve2::Nurbs(curve.clone()).point_at(1.0 - t),
        );
    }

    let split_point = Curve2::Nurbs(curve.clone()).point_at(0.4);
    let (first, second) = Curve2::Nurbs(curve)
        .split_at(0.4)
        .expect("interior split should succeed");
    assert_point2_close(first.point_at(1.0), split_point);
    assert_point2_close(second.point_at(0.0), split_point);
}

#[test]
fn curve2_recovers_parameter_for_point_on_nurbs() {
    let curve = Curve2::Nurbs(
        NurbsCurve2::interpolate(&[
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 2.0),
            Point2::new(3.0, 1.0),
            Point2::new(4.0, 0.0),
        ])
        .unwrap(),
    );
    let point = curve.point_at(0.37);
    let recovered = curve
        .parameter_at(point, LINEAR_TOLERANCE)
        .expect("point on curve should have a parameter");

    assert!((recovered - 0.37).abs() <= 1.0e-5);
}

#[test]
fn curve2_line_intersection_returns_point_and_parameters() {
    let horizontal = Curve2::Line(Line2::new(Point2::new(0.0, 0.0), Point2::new(2.0, 0.0)));
    let vertical = Curve2::Line(Line2::new(Point2::new(0.5, -1.0), Point2::new(0.5, 1.0)));

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
fn curve2_collinear_lines_return_overlap_intervals() {
    let outer = Curve2::Line(Line2::new(Point2::new(0.0, 0.0), Point2::new(3.0, 0.0)));
    let inner = Curve2::Line(Line2::new(Point2::new(1.0, 0.0), Point2::new(2.0, 0.0)));

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
fn curve2_tangent_quadratic_nurbs_curves_return_one_point() {
    let rising = quadratic_curve2([
        Point2::new(0.0, 0.0),
        Point2::new(1.0, 1.0),
        Point2::new(2.0, 0.0),
    ]);
    let falling = quadratic_curve2([
        Point2::new(0.0, 1.0),
        Point2::new(1.0, 0.0),
        Point2::new(2.0, 1.0),
    ]);

    let intersections = Curve2::Nurbs(rising)
        .intersect_curve(&Curve2::Nurbs(falling))
        .unwrap();

    assert_eq!(intersections.len(), 1, "{intersections:?}");
    let CurveCurveIntersection2::Point { point, u_a, u_b } = intersections[0] else {
        panic!("expected point intersection, got {intersections:?}");
    };
    assert_point2_close(point, Point2::new(1.0, 0.5));
    assert!((u_a - 0.5).abs() <= LINEAR_TOLERANCE * 10.0);
    assert!((u_b - 0.5).abs() <= LINEAR_TOLERANCE * 10.0);
}

#[test]
fn curve2_transverse_quadratic_nurbs_curves_return_crossing_point() {
    let rising = quadratic_curve2([
        Point2::new(0.0, 0.0),
        Point2::new(1.0, 1.0),
        Point2::new(2.0, 2.0),
    ]);
    let falling = quadratic_curve2([
        Point2::new(0.0, 2.0),
        Point2::new(1.0, 1.0),
        Point2::new(2.0, 0.0),
    ]);

    let intersections = Curve2::Nurbs(rising)
        .intersect_curve(&Curve2::Nurbs(falling))
        .unwrap();

    assert_eq!(intersections.len(), 1, "{intersections:?}");
    let CurveCurveIntersection2::Point { point, u_a, u_b } = intersections[0] else {
        panic!("expected point intersection, got {intersections:?}");
    };
    assert_point2_close(point, Point2::new(1.0, 1.0));
    assert!((u_a - 0.5).abs() <= LINEAR_TOLERANCE * 10.0);
    assert!((u_b - 0.5).abs() <= LINEAR_TOLERANCE * 10.0);
}

#[test]
fn curve2_line_and_nurbs_secant_return_two_points() {
    let arch = Curve2::Nurbs(quadratic_curve2([
        Point2::new(0.0, 0.0),
        Point2::new(1.0, 1.0),
        Point2::new(2.0, 0.0),
    ]));
    let line = Curve2::Line(Line2::new(Point2::new(0.0, 0.25), Point2::new(2.0, 0.25)));

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
fn curve2_intersection_parameters_are_normalized_from_native_domains() {
    let horizontal = Curve2::Nurbs(
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
    let vertical = Curve2::Line(Line2::new(Point2::new(1.0, -1.0), Point2::new(1.0, 1.0)));

    let intersections = horizontal.intersect_curve(&vertical).unwrap();

    let CurveCurveIntersection2::Point { u_a, u_b, .. } = intersections[0] else {
        panic!("expected point intersection, got {intersections:?}");
    };
    assert!((u_a - 0.25).abs() <= LINEAR_TOLERANCE);
    assert!((u_b - 0.5).abs() <= LINEAR_TOLERANCE);
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
