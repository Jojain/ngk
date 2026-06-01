use nalgebra::Vector3;
use ngk::geometry::{
    Circle, ControlPolygon, Curve, CurveCurveIntersection, CurveSurfaceIntersection, Degree,
    HPoint, KnotVector, LINEAR_TOLERANCE, NurbsCurve, Plane, Point3, PointCoincidence, Surface,
    SurfaceSurfaceIntersection, axis::Axis3,
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
    Curve::line(Axis3::from_points(a, b))
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
    let SurfaceSurfaceIntersection::Curve { points } = &results[0] else {
        panic!("expected intersection curve, got {results:?}");
    };
    assert!(points.len() >= 2, "{results:?}");
    assert!(
        points
            .iter()
            .all(|point| point.y.abs() <= LINEAR_TOLERANCE * 10.0
                && point.z.abs() <= LINEAR_TOLERANCE * 10.0)
    );
}

#[test]
fn coincident_planes_return_surface_surface_region() {
    let a = Surface::Plane(Plane::xy());
    let b = Surface::Plane(Plane::xy());

    let results = a.intersect_surface(&b).unwrap();

    assert!(matches!(
        results.as_slice(),
        [SurfaceSurfaceIntersection::Region]
    ));
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
