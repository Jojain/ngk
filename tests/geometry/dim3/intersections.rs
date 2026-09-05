use nalgebra::Vector3;
use ngk::geometry::{
    Circle, ControlNet, ControlPolygon, Curve, Curve2, CurveCurveIntersection,
    CurveSurfaceIntersection, Cylinder, Degree, HPoint, IntersectionCoverage,
    IntersectionIncompleteReason, IntersectionOptions, KnotVector, LINEAR_TOLERANCE, NurbsCurve,
    NurbsSurface, Plane, Point3, PointCoincidence, PreparedCurve, PreparedSurface, Surface,
    SurfaceIntersectionBranchKind, SurfaceSurfaceIntersection, intersect_prepared_curve_surface,
    intersect_surfaces_with_options,
};

fn assert_point_near(actual: Point3, expected: Point3) {
    assert!(
        actual.coincides(expected, LINEAR_TOLERANCE * 10.0),
        "expected {expected:?}, got {actual:?}"
    );
}

fn point_results(results: &[CurveCurveIntersection]) -> Vec<Point3> {
    results
        .iter()
        .filter_map(|result| match result {
            CurveCurveIntersection::Point { point, .. } => Some(*point),
            CurveCurveIntersection::Overlap { .. } => None,
        })
        .collect()
}

fn overlap_count(results: &[CurveCurveIntersection]) -> usize {
    results
        .iter()
        .filter(|result| matches!(result, CurveCurveIntersection::Overlap { .. }))
        .count()
}

fn unique_points(points: impl IntoIterator<Item = Point3>) -> Vec<Point3> {
    let mut unique = Vec::new();
    for point in points {
        if !unique
            .iter()
            .any(|existing| point.coincides(*existing, LINEAR_TOLERANCE * 10.0))
        {
            unique.push(point);
        }
    }
    unique
}

fn line(a: Point3, b: Point3) -> Curve {
    Curve::line(a, b)
}

fn is_bounded_line(curve: &Curve) -> bool {
    matches!(curve, Curve::Bounded(curve) if matches!(curve.inner(), Curve::Line(_)))
}

fn is_bounded_circle(curve: &Curve) -> bool {
    matches!(curve, Curve::Bounded(curve) if matches!(curve.inner(), Curve::Circle(_)))
}

#[test]
fn line_line_intersection_returns_point() {
    let a = line(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));
    let b = line(Point3::new(0.5, -1.0, 0.0), Point3::new(0.5, 1.0, 0.0));

    let results = a.intersect_curve(&b).unwrap();

    let points = point_results(&results);
    assert_eq!(points.len(), 1, "{results:?}");
    assert_point_near(points[0], Point3::new(0.5, 0.0, 0.0));
}

#[test]
fn skew_lines_do_not_intersect() {
    let a = line(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));
    let b = line(Point3::new(0.5, -1.0, 1.0), Point3::new(0.5, 1.0, 1.0));

    let results = a.intersect_curve(&b).unwrap();

    assert!(results.is_empty(), "{results:?}");
}

#[test]
fn collinear_lines_return_overlap_interval() {
    let a = line(Point3::new(0.0, 0.0, 0.0), Point3::new(3.0, 0.0, 0.0));
    let b = line(Point3::new(1.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0));

    let results = a.intersect_curve(&b).unwrap();

    assert_eq!(results.len(), 1, "{results:?}");
    let CurveCurveIntersection::Overlap {
        interval_a,
        interval_b,
    } = results[0]
    else {
        panic!("expected overlap, got {results:?}");
    };
    assert!((interval_a.start - 1.0 / 3.0).abs() <= LINEAR_TOLERANCE * 10.0);
    assert!((interval_a.end - 2.0 / 3.0).abs() <= LINEAR_TOLERANCE * 10.0);
    assert!((interval_b.start - 0.0).abs() <= LINEAR_TOLERANCE * 10.0);
    assert!((interval_b.end - 1.0).abs() <= LINEAR_TOLERANCE * 10.0);
}

#[test]
fn tangent_line_circle_returns_single_point() {
    let circle = Curve::Circle(Circle::new(Plane::xy(), 1.0));
    let line = line(Point3::new(-1.0, 1.0, 0.0), Point3::new(1.0, 1.0, 0.0));

    let results = line.intersect_curve(&circle).unwrap();

    let points = unique_points(point_results(&results));
    assert_eq!(points.len(), 1, "{results:?}");
    assert_point_near(points[0], Point3::new(0.0, 1.0, 0.0));
}

#[test]
fn secant_line_circle_returns_two_points() {
    let circle = Curve::Circle(Circle::new(Plane::xy(), 1.0));
    let line = line(Point3::new(-2.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0));

    let results = line.intersect_curve(&circle).unwrap();

    let points = unique_points(point_results(&results));
    assert_eq!(points.len(), 2, "{results:?}");
    assert!(
        points
            .iter()
            .any(|point| point.coincides(Point3::new(-1.0, 0.0, 0.0), LINEAR_TOLERANCE * 10.0))
    );
    assert!(
        points
            .iter()
            .any(|point| point.coincides(Point3::new(1.0, 0.0, 0.0), LINEAR_TOLERANCE * 10.0))
    );
}

#[test]
fn separated_circles_do_not_intersect() {
    let a = Curve::Circle(Circle::new(Plane::xy(), 1.0));
    let b = Curve::Circle(Circle::new(
        Plane::from_xy(Point3::new(3.0, 0.0, 0.0), Vector3::x(), Vector3::y()),
        1.0,
    ));

    let results = a.intersect_curve(&b).unwrap();

    assert!(results.is_empty(), "{results:?}");
}

#[test]
fn identical_circles_return_bezier_span_overlaps() {
    let a = Curve::Circle(Circle::new(Plane::xy(), 1.0));
    let b = Curve::Circle(Circle::new(Plane::xy(), 1.0));

    let results = a.intersect_curve(&b).unwrap();

    assert!(
        overlap_count(&results) >= 4,
        "expected at least 4 overlap spans, got {results:?}"
    );
}

#[test]
fn quadratic_nurbs_curves_return_crossing_point() {
    let a = quadratic_curve([
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
    ]);
    let b = quadratic_curve([
        Point3::new(0.0, 1.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(2.0, 1.0, 0.0),
    ]);

    let results = Curve::Nurbs(a).intersect_curve(&Curve::Nurbs(b)).unwrap();

    let points = unique_points(point_results(&results));
    assert_eq!(points.len(), 1, "{results:?}");
    assert_point_near(points[0], Point3::new(1.0, 0.5, 0.0));
}

#[test]
fn line_plane_intersection_returns_curve_surface_point() {
    let curve = line(Point3::new(0.5, 0.5, -1.0), Point3::new(0.5, 0.5, 1.0));
    let surface = Surface::Plane(Plane::xy());

    let results = curve.intersect_surface(&surface).unwrap();

    assert_eq!(results.len(), 1, "{results:?}");
    let CurveSurfaceIntersection::Point { point, .. } = results[0] else {
        panic!("expected point, got {results:?}");
    };
    assert_point_near(point, Point3::new(0.5, 0.5, 0.0));
}

#[test]
fn line_on_plane_returns_curve_surface_overlap() {
    let curve = line(Point3::new(0.2, 0.2, 0.0), Point3::new(0.8, 0.8, 0.0));
    let surface = Surface::Plane(Plane::xy());

    let results = curve.intersect_surface(&surface).unwrap();

    assert_eq!(results.len(), 1, "{results:?}");
    let CurveSurfaceIntersection::Overlap { curve_interval } = results[0] else {
        panic!("expected overlap, got {results:?}");
    };
    assert!((curve_interval.start - 0.0).abs() <= LINEAR_TOLERANCE * 10.0);
    assert!((curve_interval.end - 1.0).abs() <= LINEAR_TOLERANCE * 10.0);
}

#[test]
fn perpendicular_planes_return_surface_surface_curve() {
    let a = Surface::Plane(Plane::xy());
    let b = Surface::Plane(Plane::xz());

    let results = a.intersect_surface(&b).unwrap();

    assert_eq!(results.len(), 1, "{results:?}");
    let SurfaceSurfaceIntersection::Branch(branch) = &results[0] else {
        panic!(
            "expected intersection branch, got {:?}",
            results.intersections()
        );
    };
    assert!(branch.samples.len() >= 2, "{:?}", results.intersections());
    assert!(is_bounded_line(&branch.curve_3d));
    assert!(matches!(branch.pcurve_a, Curve2::Line(_)));
    assert!(matches!(branch.pcurve_b, Curve2::Line(_)));
    assert!(
        branch
            .samples
            .iter()
            .all(|sample| sample.point.y.abs() <= LINEAR_TOLERANCE * 10.0
                && sample.point.z.abs() <= LINEAR_TOLERANCE * 10.0
                && sample.residual <= LINEAR_TOLERANCE)
    );
    for parameter in [0.0, 0.25, 0.5, 0.75, 1.0] {
        let point = branch.curve_3d.point_at(parameter);
        let uv_a = branch.pcurve_a.point_at(parameter);
        let uv_b = branch.pcurve_b.point_at(parameter);
        assert_point_near(point, a.point_at(uv_a.x, uv_a.y));
        assert_point_near(point, b.point_at(uv_b.x, uv_b.y));
    }
    assert_eq!(results.coverage(), &IntersectionCoverage::Complete);
}

#[test]
fn coincident_planes_return_overlap_candidate() {
    let a = Surface::Plane(Plane::xy());
    let b = Surface::Plane(Plane::xy());

    let results = a.intersect_surface(&b).unwrap();

    assert!(matches!(
        results.as_slice(),
        [SurfaceSurfaceIntersection::OverlapCandidate(_)]
    ));
    assert!(matches!(
        results.coverage(),
        IntersectionCoverage::Incomplete(reasons)
            if reasons.contains(
                &IntersectionIncompleteReason::CoincidentRegionResolutionNotImplemented
            )
    ));
}

#[test]
fn plane_cylinder_intersection_returns_closed_synchronized_branch() {
    let plane = square_nurbs_plane(0.5, 2.0);
    let cylinder = Surface::Cylinder(Cylinder::new(
        Point3::origin(),
        Vector3::x(),
        Vector3::z(),
        1.0,
    ));

    let results = plane.intersect_surface(&cylinder).unwrap();

    assert_eq!(results.coverage(), &IntersectionCoverage::Complete);

    let branches = results
        .intersections()
        .iter()
        .filter_map(|intersection| match intersection {
            SurfaceSurfaceIntersection::Branch(branch) => Some(branch),
            _ => None,
        })
        .collect::<Vec<_>>();
    assert_eq!(branches.len(), 1, "{:?}", results.intersections());
    let branch = branches[0];
    assert!(
        branch.closed,
        "terminal samples: {:?}",
        [branch.samples.first(), branch.samples.last()]
    );
    assert!(branch.samples.len() > 16);
    assert!(is_bounded_circle(&branch.curve_3d));
    assert!(matches!(branch.pcurve_a, Curve2::Circle(_)));
    assert!(matches!(branch.pcurve_b, Curve2::Line(_)));
    assert!(
        branch.quality.max_residual <= LINEAR_TOLERANCE,
        "{:?}",
        branch.quality
    );
    assert!(
        branch.quality.max_fit_error <= IntersectionOptions::default().fit_tolerance,
        "{:?}",
        branch.quality
    );
    assert!(branch.quality.certified);
    assert_point_near(branch.curve_3d.point_at(0.0), branch.curve_3d.point_at(1.0));
    for parameter in [0.125, 0.375, 0.625, 0.875] {
        let point = branch.curve_3d.point_at(parameter);
        let uv_plane = branch.pcurve_a.point_at(parameter);
        let uv_cylinder = branch.pcurve_b.point_at(parameter);
        let fit_tolerance = IntersectionOptions::default().fit_tolerance;
        assert!((point - plane.point_at(uv_plane.x, uv_plane.y)).norm() <= fit_tolerance);
        assert!((point - cylinder.point_at(uv_cylinder.x, uv_cylinder.y)).norm() <= fit_tolerance);
    }
    for index in 0..=128 {
        let parameter = index as f64 / 128.0;
        let point = branch.curve_3d.point_at(parameter);
        let uv_plane = branch.pcurve_a.point_at(parameter);
        let uv_cylinder = branch.pcurve_b.point_at(parameter);
        let fit_tolerance = IntersectionOptions::default().fit_tolerance;
        assert!((point - plane.point_at(uv_plane.x, uv_plane.y)).norm() <= fit_tolerance);
        assert!((point - cylinder.point_at(uv_cylinder.x, uv_cylinder.y)).norm() <= fit_tolerance);
    }
}

#[test]
fn surface_intersection_simplification_is_enabled_by_default() {
    assert!(IntersectionOptions::default().simplify_curves);
}

#[test]
fn interior_paraboloid_plane_loop_is_found_with_complete_coverage() {
    let paraboloid = square_paraboloid(0.5);
    let plane = square_nurbs_plane(0.0, 1.0);

    let results = paraboloid.intersect_surface(&plane).unwrap();

    assert_eq!(results.coverage(), &IntersectionCoverage::Complete);
    let branches = results
        .intersections()
        .iter()
        .filter_map(|intersection| match intersection {
            SurfaceSurfaceIntersection::Branch(branch) => Some(branch),
            _ => None,
        })
        .collect::<Vec<_>>();
    assert_eq!(branches.len(), 1, "{results:?}");
    assert!(branches[0].closed, "{results:?}");
    assert!(is_bounded_circle(&branches[0].curve_3d), "{results:?}");
}

#[test]
fn surface_intersection_can_keep_synchronized_nurbs_curves() {
    let a = Surface::Plane(Plane::xy());
    let b = Surface::Plane(Plane::xz());
    let options = IntersectionOptions {
        simplify_curves: false,
        ..IntersectionOptions::default()
    };

    let results = intersect_surfaces_with_options(&a, &b, options).unwrap();

    let SurfaceSurfaceIntersection::Branch(branch) = &results[0] else {
        panic!("expected an intersection branch, got {results:?}");
    };
    assert!(matches!(branch.curve_3d, Curve::Nurbs(_)));
    assert!(matches!(branch.pcurve_a, Curve2::Nurbs(_)));
    assert!(matches!(branch.pcurve_b, Curve2::Nurbs(_)));
}

#[test]
fn disjoint_surface_control_hulls_return_complete_empty_coverage() {
    let a = Surface::Plane(Plane::xy());
    let b = Surface::Plane(Plane::from_xy(
        Point3::new(0.0, 0.0, 2.0),
        Vector3::x(),
        Vector3::y(),
    ));

    let results = a.intersect_surface(&b).unwrap();

    assert!(results.is_empty());
    assert_eq!(results.coverage(), &IntersectionCoverage::Complete);
}

#[test]
fn invalid_surface_intersection_options_return_an_error() {
    let options = IntersectionOptions {
        residual_tolerance: 0.0,
        ..IntersectionOptions::default()
    };

    let error = intersect_surfaces_with_options(
        &Surface::Plane(Plane::xy()),
        &Surface::Plane(Plane::xz()),
        options,
    )
    .unwrap_err();

    assert!(matches!(
        error,
        ngk::geometry::IntersectionError::InvalidOptions
    ));
}

#[test]
fn unsupported_surface_weights_return_incomplete_coverage() {
    let points = vec![
        HPoint::from_cartesian(Point3::new(0.0, 0.0, 0.0), -1.0),
        HPoint::from_cartesian(Point3::new(1.0, 0.0, 0.0), 1.0),
        HPoint::from_cartesian(Point3::new(0.0, 1.0, 0.0), 1.0),
        HPoint::from_cartesian(Point3::new(1.0, 1.0, 0.0), 1.0),
    ];
    let knots = KnotVector::new(vec![0.0, 0.0, 1.0, 1.0]).unwrap();
    let unsupported = Surface::Nurbs(
        NurbsSurface::new(
            Degree::new(1).unwrap(),
            Degree::new(1).unwrap(),
            ControlNet::new(points, 2, 2).unwrap(),
            knots.clone(),
            knots,
        )
        .unwrap(),
    );

    let results = unsupported
        .intersect_surface(&Surface::Plane(Plane::xz()))
        .unwrap();

    assert!(results.is_empty());
    assert!(matches!(
        results.coverage(),
        IntersectionCoverage::Incomplete(reasons)
            if reasons == &[IntersectionIncompleteReason::UnsupportedControlPointWeights]
    ));
}

#[test]
fn exhausted_trace_budget_is_reported_as_incomplete() {
    let options = IntersectionOptions {
        max_trace_steps: 1,
        ..IntersectionOptions::default()
    };

    let results = intersect_surfaces_with_options(
        &Surface::Plane(Plane::xy()),
        &Surface::Plane(Plane::xz()),
        options,
    )
    .unwrap();

    assert!(matches!(
        results.coverage(),
        IntersectionCoverage::Incomplete(reasons)
            if reasons.contains(&IntersectionIncompleteReason::TraceBudgetExhausted)
    ));
}

#[test]
fn minimum_trace_step_returns_a_singular_point_and_incomplete_reason() {
    let options = IntersectionOptions {
        residual_tolerance: 1.0e-14,
        newton_max_iterations: 1,
        min_trace_step: 2.0e-2,
        max_trace_step: 2.0e-2,
        ..IntersectionOptions::default()
    };
    let plane = square_nurbs_plane(0.5, 2.0);
    let cylinder = Surface::Cylinder(Cylinder::new(
        Point3::origin(),
        Vector3::x(),
        Vector3::z(),
        1.0,
    ));

    let results = intersect_surfaces_with_options(&plane, &cylinder, options).unwrap();

    assert!(results.intersections().iter().any(|intersection| matches!(
        intersection,
        SurfaceSurfaceIntersection::Point(point)
            if point.kind == ngk::geometry::SurfaceIntersectionPointKind::Singular
    )));
    assert!(matches!(
        results.coverage(),
        IntersectionCoverage::Incomplete(reasons)
            if reasons.contains(&IntersectionIncompleteReason::MinimumTraceStepReached)
    ));
}

#[test]
fn synchronized_fit_tolerance_failure_is_reported_as_incomplete() {
    let options = IntersectionOptions {
        simplify_curves: false,
        fit_tolerance: 1.0e-12,
        ..IntersectionOptions::default()
    };
    let plane = square_nurbs_plane(0.5, 2.0);
    let cylinder = Surface::Cylinder(Cylinder::new(
        Point3::origin(),
        Vector3::x(),
        Vector3::z(),
        1.0,
    ));

    let results = intersect_surfaces_with_options(&plane, &cylinder, options).unwrap();

    assert!(matches!(
        results.coverage(),
        IntersectionCoverage::Incomplete(reasons)
            if reasons.contains(
                &IntersectionIncompleteReason::SynchronizedFitToleranceExceeded
            )
    ));
}

#[test]
fn a_plane_tangent_to_a_cylinder_yields_the_ruling_it_touches_along() {
    let tangent_plane = Surface::Plane(Plane::from_xy(
        Point3::new(1.0, 0.0, 0.0),
        Vector3::y(),
        Vector3::z(),
    ));
    let cylinder = Surface::Cylinder(Cylinder::new(
        Point3::origin(),
        Vector3::x(),
        Vector3::z(),
        1.0,
    ));

    let results = tangent_plane.intersect_surface(&cylinder).unwrap();

    assert_eq!(results.coverage(), &IntersectionCoverage::Complete);
    let branches = results
        .intersections()
        .iter()
        .filter_map(|contact| match contact {
            SurfaceSurfaceIntersection::Branch(branch) => Some(branch),
            _ => None,
        })
        .collect::<Vec<_>>();
    assert_eq!(branches.len(), 1);
    let branch = branches[0];
    assert_eq!(branch.kind, SurfaceIntersectionBranchKind::Tangent);
    assert!(branch.quality.certified);
    // The contact is the ruling the plane rests on, so every point of the
    // branch stays on both surfaces' shared line.
    for step in 0..=8 {
        let point = branch.curve_3d.point_at(f64::from(step) / 8.0);
        assert!(
            (point.x - 1.0).abs() <= LINEAR_TOLERANCE && point.y.abs() <= LINEAR_TOLERANCE,
            "tangency point {point:?} left the ruling"
        );
    }
}

fn quadratic_curve(points: [Point3; 3]) -> NurbsCurve {
    NurbsCurve::new(
        Degree::new(2).unwrap(),
        ControlPolygon::new(
            points
                .into_iter()
                .map(|point| HPoint::from_cartesian(point, 1.0))
                .collect(),
        )
        .unwrap(),
        KnotVector::new(vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0]).unwrap(),
    )
    .unwrap()
}

fn square_nurbs_plane(z: f64, half_size: f64) -> Surface {
    let points = vec![
        Point3::new(-half_size, -half_size, z),
        Point3::new(half_size, -half_size, z),
        Point3::new(-half_size, half_size, z),
        Point3::new(half_size, half_size, z),
    ]
    .into_iter()
    .map(|point| HPoint::from_cartesian(point, 1.0))
    .collect();
    let knots = KnotVector::new(vec![0.0, 0.0, 1.0, 1.0]).unwrap();
    Surface::Nurbs(
        NurbsSurface::new(
            Degree::new(1).unwrap(),
            Degree::new(1).unwrap(),
            ControlNet::new(points, 2, 2).unwrap(),
            knots.clone(),
            knots,
        )
        .unwrap(),
    )
}

/// Exact biquadratic patch `z = x^2 + y^2 - radius^2` over `[-1, 1]^2`.
fn square_paraboloid(radius: f64) -> Surface {
    let coordinates = [-1.0, 0.0, 1.0];
    let square_coefficients = [1.0, -1.0, 1.0];
    let points = (0..3)
        .flat_map(|v| {
            (0..3).map(move |u| {
                HPoint::from_cartesian(
                    Point3::new(
                        coordinates[u],
                        coordinates[v],
                        square_coefficients[u] + square_coefficients[v] - radius * radius,
                    ),
                    1.0,
                )
            })
        })
        .collect();
    let knots = KnotVector::new(vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0]).unwrap();
    Surface::Nurbs(
        NurbsSurface::new(
            Degree::new(2).unwrap(),
            Degree::new(2).unwrap(),
            ControlNet::new(points, 3, 3).unwrap(),
            knots.clone(),
            knots,
        )
        .unwrap(),
    )
}

#[test]
fn curve_crossing_a_plane_twice_returns_both_points() {
    let curve = Curve::Nurbs(quadratic_curve([
        Point3::new(0.2, 0.2, 1.0),
        Point3::new(0.5, 0.5, -3.0),
        Point3::new(0.8, 0.8, 1.0),
    ]));
    let surface = Surface::Plane(Plane::xy());

    let results = curve.intersect_surface(&surface).unwrap();

    assert_eq!(results.len(), 2, "{results:?}");
    for result in results.intersections() {
        let CurveSurfaceIntersection::Point { point, .. } = result else {
            panic!("expected points, got {results:?}");
        };
        assert!(point.z.abs() <= LINEAR_TOLERANCE * 10.0, "{point:?}");
    }
}

#[test]
fn curve_missing_a_plane_returns_complete_empty_coverage() {
    let curve = line(Point3::new(0.2, 0.2, 1.0), Point3::new(0.8, 0.8, 2.0));
    let surface = Surface::Plane(Plane::xy());

    let results = curve.intersect_surface(&surface).unwrap();

    assert!(results.is_empty(), "{results:?}");
    assert_eq!(results.coverage(), &IntersectionCoverage::Complete);
}

#[test]
fn curve_endpoint_touching_a_plane_is_reported() {
    let curve = line(Point3::new(0.5, 0.5, 0.0), Point3::new(0.5, 0.5, 1.0));
    let surface = Surface::Plane(Plane::xy());

    let results = curve.intersect_surface(&surface).unwrap();

    assert_eq!(results.len(), 1, "{results:?}");
    let CurveSurfaceIntersection::Point { point, curve_u, .. } = results[0] else {
        panic!("expected point, got {results:?}");
    };
    assert_point_near(point, Point3::new(0.5, 0.5, 0.0));
    assert!(curve_u.abs() <= LINEAR_TOLERANCE.sqrt(), "{curve_u}");
}

#[test]
fn contact_at_an_interior_knot_is_found_once() {
    // Two Bézier spans meet at u = 0.5; the crossing sits exactly on that seam,
    // so both spans propose it and the result must still be a single point.
    let curve = Curve::Nurbs(
        NurbsCurve::new(
            Degree::new(1).unwrap(),
            ControlPolygon::new(vec![
                HPoint::from_cartesian(Point3::new(0.5, 0.5, -1.0), 1.0),
                HPoint::from_cartesian(Point3::new(0.5, 0.5, 0.0), 1.0),
                HPoint::from_cartesian(Point3::new(0.5, 0.5, 1.0), 1.0),
            ])
            .unwrap(),
            KnotVector::new(vec![0.0, 0.0, 0.5, 1.0, 1.0]).unwrap(),
        )
        .unwrap(),
    );
    let surface = Surface::Plane(Plane::xy());

    let results = curve.intersect_surface(&surface).unwrap();

    assert_eq!(results.len(), 1, "{results:?}");
    assert_eq!(results.coverage(), &IntersectionCoverage::Complete);
}

#[test]
fn rational_arc_crossing_a_plane_returns_the_exact_point() {
    // A rational quarter circle in the xz plane, centred at the origin with
    // radius one, crosses the z = 0 plane at (1, 0, 0).
    let weight = std::f64::consts::FRAC_1_SQRT_2;
    let curve = Curve::Nurbs(
        NurbsCurve::new(
            Degree::new(2).unwrap(),
            ControlPolygon::new(vec![
                HPoint::from_cartesian(Point3::new(1.0, 0.0, -1.0), 1.0),
                HPoint::from_cartesian(Point3::new(1.0, 0.0, 0.0), weight),
                HPoint::from_cartesian(Point3::new(1.0, 0.0, 1.0), 1.0),
            ])
            .unwrap(),
            KnotVector::new(vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0]).unwrap(),
        )
        .unwrap(),
    );
    let surface = Surface::Plane(Plane::from_xy(
        Point3::new(0.0, -1.0, 0.0),
        Vector3::x(),
        Vector3::y(),
    ));

    let results = curve.intersect_surface(&surface).unwrap();

    assert_eq!(results.len(), 1, "{results:?}");
    let CurveSurfaceIntersection::Point { point, .. } = results[0] else {
        panic!("expected point, got {results:?}");
    };
    assert_point_near(point, Point3::new(1.0, 0.0, 0.0));
}

#[test]
fn curve_surface_contacts_survive_a_thousandfold_scale_change() {
    for scale in [1.0e-3, 1.0e3] {
        let curve = line(
            Point3::new(0.5 * scale, 0.5 * scale, -scale),
            Point3::new(0.5 * scale, 0.5 * scale, scale),
        );
        let surface = Surface::Plane(Plane::xy());
        let box_side = ngk::geometry::Interval::new(0.0, scale);
        let results = intersect_prepared_curve_surface(
            &PreparedCurve::new(&curve).unwrap(),
            &PreparedSurface::over(&surface, box_side, box_side).unwrap(),
            IntersectionOptions::default(),
        )
        .unwrap();

        assert_eq!(results.len(), 1, "scale {scale}: {results:?}");
        let CurveSurfaceIntersection::Point { point, .. } = results[0] else {
            panic!("expected point, got {results:?}");
        };
        assert!(
            point.coincides(
                Point3::new(0.5 * scale, 0.5 * scale, 0.0),
                LINEAR_TOLERANCE.sqrt() * scale
            ),
            "scale {scale}: {point:?}"
        );
    }
}

#[test]
fn non_positive_curve_weights_return_incomplete_coverage() {
    let curve = Curve::Nurbs(
        NurbsCurve::new(
            Degree::new(1).unwrap(),
            ControlPolygon::new(vec![
                HPoint::from_cartesian(Point3::new(0.5, 0.5, -1.0), 1.0),
                HPoint::from_cartesian(Point3::new(0.5, 0.5, 1.0), -1.0),
            ])
            .unwrap(),
            KnotVector::new(vec![0.0, 0.0, 1.0, 1.0]).unwrap(),
        )
        .unwrap(),
    );
    let surface = Surface::Plane(Plane::xy());

    let results = curve.intersect_surface(&surface).unwrap();

    // Control hulls do not bound negative-weight geometry, so every rejection
    // the search makes would be unsound. It reports that instead of guessing.
    assert!(matches!(
        results.coverage(),
        IntersectionCoverage::Incomplete(reasons)
            if reasons.contains(&IntersectionIncompleteReason::UnsupportedControlPointWeights)
    ));
}

#[test]
fn plane_patch_covers_the_requested_parameter_box() {
    let surface = Surface::Plane(Plane::xy());
    let curve = line(Point3::new(1.5, 1.5, -1.0), Point3::new(1.5, 1.5, 1.0));
    let options = IntersectionOptions::default();

    // An unbounded plane converts to an arbitrary unit patch, which cannot see
    // a crossing outside it.
    let default = curve.intersect_surface(&surface).unwrap();
    assert!(default.is_empty(), "{default:?}");

    // Given the trim box a face actually occupies, the same crossing is found.
    let prepared = PreparedSurface::over(
        &surface,
        ngk::geometry::Interval::new(0.0, 2.0),
        ngk::geometry::Interval::new(0.0, 2.0),
    )
    .unwrap();
    let results =
        intersect_prepared_curve_surface(&PreparedCurve::new(&curve).unwrap(), &prepared, options)
            .unwrap();

    assert_eq!(results.len(), 1, "{results:?}");
    let CurveSurfaceIntersection::Point {
        point,
        surface_u,
        surface_v,
        ..
    } = results[0]
    else {
        panic!("expected point, got {results:?}");
    };
    assert_point_near(point, Point3::new(1.5, 1.5, 0.0));
    assert!(
        (surface_u - 1.5).abs() <= LINEAR_TOLERANCE.sqrt(),
        "{surface_u}"
    );
    assert!(
        (surface_v - 1.5).abs() <= LINEAR_TOLERANCE.sqrt(),
        "{surface_v}"
    );
}

#[test]
fn boundary_seeding_finds_a_curved_nurbs_surface_pair() {
    // Neither operand is a plane, so this exercises the general tracing path
    // and the boundary seeding underneath it rather than any analytic shortcut.
    let cylinder = Surface::Cylinder(Cylinder::new(
        Point3::new(0.0, 0.0, -1.0),
        Vector3::x(),
        Vector3::z(),
        1.0,
    ));
    let sheet = square_nurbs_plane(0.0, 3.0);

    let results =
        intersect_surfaces_with_options(&cylinder, &sheet, IntersectionOptions::default()).unwrap();

    assert!(
        results
            .intersections()
            .iter()
            .any(|result| matches!(result, SurfaceSurfaceIntersection::Branch(_))),
        "{results:?}"
    );
    for result in results.intersections() {
        let SurfaceSurfaceIntersection::Branch(branch) = result else {
            continue;
        };
        for sample in &branch.samples {
            assert!(
                (sample.point.coords.xy().norm() - 1.0).abs() <= LINEAR_TOLERANCE.sqrt(),
                "{sample:?}"
            );
            assert!(
                sample.point.z.abs() <= LINEAR_TOLERANCE.sqrt(),
                "{sample:?}"
            );
        }
    }
}

#[test]
fn a_curve_crossing_a_plane_nine_times_is_fully_resolved() {
    // Guards the search budget: a legitimate multi-root case must still report
    // complete coverage rather than giving up part way through.
    let count = 10;
    let points: Vec<_> = (0..count)
        .map(|i| {
            let x = 0.05 + 0.9 * (i as f64) / (count - 1) as f64;
            let z = if i % 2 == 0 { -1.0 } else { 1.0 };
            HPoint::from_cartesian(Point3::new(x, 0.5, z), 1.0)
        })
        .collect();
    let mut knots = vec![0.0, 0.0];
    knots.extend((1..count - 1).map(|i| i as f64 / (count - 1) as f64));
    knots.extend([1.0, 1.0]);
    let curve = Curve::Nurbs(
        NurbsCurve::new(
            Degree::new(1).unwrap(),
            ControlPolygon::new(points).unwrap(),
            KnotVector::new(knots).unwrap(),
        )
        .unwrap(),
    );
    let surface = Surface::Plane(Plane::xy());

    let results = curve.intersect_surface(&surface).unwrap();

    assert_eq!(results.len(), count - 1, "{results:?}");
    assert_eq!(results.coverage(), &IntersectionCoverage::Complete);
    for result in results.intersections() {
        let CurveSurfaceIntersection::Point { point, .. } = result else {
            panic!("expected points, got {results:?}");
        };
        assert!(point.z.abs() <= LINEAR_TOLERANCE * 10.0, "{point:?}");
    }
}

#[test]
fn crossing_cylinders_return_two_interior_loops_with_complete_coverage() {
    // Both intersection curves lie strictly inside the two unit-height patches
    // and never touch a patch boundary, so they are only found once a
    // subdivision certifies that the parameter boxes hold no other loop.
    // Different radii keep the crossing transverse everywhere.
    let upright = Surface::Cylinder(Cylinder::new(
        Point3::new(0.0, 0.0, -0.5),
        Vector3::x(),
        Vector3::z(),
        0.3,
    ));
    let lying = Surface::Cylinder(Cylinder::new(
        Point3::new(-0.5, 0.0, 0.0),
        Vector3::y(),
        Vector3::x(),
        0.2,
    ));

    let results = upright.intersect_surface(&lying).unwrap();

    assert_eq!(
        results.coverage(),
        &IntersectionCoverage::Complete,
        "{results:?}"
    );
    let branches = results
        .intersections()
        .iter()
        .filter_map(|intersection| match intersection {
            SurfaceSurfaceIntersection::Branch(branch) => Some(branch),
            _ => None,
        })
        .collect::<Vec<_>>();
    assert_eq!(branches.len(), 2, "{results:?}");
    for branch in &branches {
        assert!(branch.closed, "{branch:?}");
        assert!(branch.quality.certified, "{branch:?}");
        for index in 0..=64 {
            let parameter = index as f64 / 64.0;
            let point = branch.curve_3d.point_at(parameter);
            assert!(
                (point.x * point.x + point.y * point.y - 0.09).abs() <= 1.0e-6,
                "{point:?}"
            );
            assert!(
                (point.y * point.y + point.z * point.z - 0.04).abs() <= 1.0e-6,
                "{point:?}"
            );
        }
    }
    // The two loops sit on opposite sides of the upright cylinder's axis.
    assert!(
        branches[0].curve_3d.point_at(0.0).x * branches[1].curve_3d.point_at(0.0).x < 0.0,
        "{results:?}"
    );
}
