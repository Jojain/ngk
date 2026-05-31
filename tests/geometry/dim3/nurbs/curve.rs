use std::f64::consts::FRAC_1_SQRT_2;

use nalgebra::Vector3;
use ngk::geometry::{
    Circle, ControlPolygon, Degree, HPoint, KnotVector, LINEAR_TOLERANCE, NurbsCurve, Plane, Point3,
};

fn assert_vector_near(actual: Vector3<f64>, expected: Vector3<f64>, tol: f64) {
    let error = (actual - expected).norm();
    assert!(
        error <= tol,
        "expected {expected:?}, got {actual:?}, error {error}"
    );
}

fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}

#[test]
fn quadratic_bezier_midpoint() {
    let cps = vec![
        HPoint::from_cartesian(Point3::new(0.0, 0.0, 0.0), 1.0),
        HPoint::from_cartesian(Point3::new(1.0, 1.0, 0.0), 1.0),
        HPoint::from_cartesian(Point3::new(2.0, 0.0, 0.0), 1.0),
    ];
    let cp = ControlPolygon::new(cps).unwrap();
    let knots = KnotVector::new(vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0]).unwrap();
    let curve = NurbsCurve::new(Degree::new(2).unwrap(), cp, knots).unwrap();
    let p = curve.point_at(0.5);
    assert!(approx_eq(p.x, 1.0, 1e-10), "x = {}", p.x);
    assert!(approx_eq(p.y, 0.5, 1e-10), "y = {}", p.y);
    assert!(approx_eq(p.z, 0.0, 1e-10), "z = {}", p.z);
}

#[test]
fn cubic_bezier_endpoints() {
    let pts = vec![
        Point3::new(-2.0, 0.0, 0.0),
        Point3::new(-1.0, 2.0, 0.0),
        Point3::new(1.0, -2.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
    ];
    let cp = ControlPolygon::from_cartesian(pts.clone(), &[1.0, 1.0, 1.0, 1.0]).unwrap();
    let curve = NurbsCurve::with_uniform_knots(Degree::new(3).unwrap(), cp).unwrap();

    let p0 = curve.point_at(0.0);
    assert!((p0 - pts[0]).norm() < 1e-10);

    let p1 = curve.point_at(1.0);
    assert!((p1 - pts[3]).norm() < 1e-10);
}

#[test]
fn line_nurbs_derivative_matches_line_direction() {
    let start = Point3::new(1.0, 2.0, 3.0);
    let end = Point3::new(4.0, 6.0, 8.0);
    let cp = ControlPolygon::from_cartesian(vec![start, end], &[1.0, 1.0]).unwrap();
    let curve = NurbsCurve::with_uniform_knots(Degree::new(1).unwrap(), cp).unwrap();

    assert_vector_near(curve.derivative_at(0.35, 1), end - start, 1e-12);
    assert_vector_near(curve.derivative_at(0.35, 2), Vector3::zeros(), 1e-12);
}

#[test]
fn insert_knot_preserves_shape() {
    let pts = vec![
        Point3::new(-2.0, 0.0, 0.0),
        Point3::new(-1.0, 2.0, 0.0),
        Point3::new(1.0, -2.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
    ];
    let cp = ControlPolygon::from_cartesian(pts, &[1.0, 1.0, 1.0, 1.0]).unwrap();
    let mut curve = NurbsCurve::with_uniform_knots(Degree::new(3).unwrap(), cp).unwrap();

    let orig_samples: Vec<_> = (0..=20).map(|i| curve.point_at(i as f64 / 20.0)).collect();
    let orig_cp_count = curve.control_points().len();
    let orig_knot_count = curve.knots().len();

    curve.insert_knot(0.5);

    assert_eq!(curve.control_points().len(), orig_cp_count + 1);
    assert_eq!(curve.knots().len(), orig_knot_count + 1);

    for (i, orig) in orig_samples.iter().enumerate() {
        let p = curve.point_at(i as f64 / 20.0);
        let err = (p - orig).norm();
        assert!(err < LINEAR_TOLERANCE, "sample {} deviates by {}", i, err);
    }
}

#[test]
fn insert_knot_quadratic_s1() {
    let pts = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(2.0, -1.0, 0.0),
        Point3::new(3.0, 0.0, 0.0),
    ];
    let cp = ControlPolygon::from_cartesian(pts, &[1.0, 1.0, 1.0, 1.0]).unwrap();
    let mut curve = NurbsCurve::with_uniform_knots(Degree::new(2).unwrap(), cp).unwrap();

    assert_eq!(curve.knots().len(), 7);
    assert_eq!(curve.control_points().len(), 4);

    let orig_samples: Vec<_> = (0..=20).map(|i| curve.point_at(i as f64 / 20.0)).collect();

    curve.insert_knot(0.5);

    assert_eq!(
        curve.knots().len(),
        8,
        "knots after insert: {:?}",
        curve.knots().as_slice()
    );
    assert_eq!(curve.control_points().len(), 5);

    for (i, orig) in orig_samples.iter().enumerate() {
        let p = curve.point_at(i as f64 / 20.0);
        let err = (p - orig).norm();
        assert!(err < LINEAR_TOLERANCE, "sample {} deviates by {}", i, err);
    }
}

#[test]
fn rational_circle_quarter() {
    let w = FRAC_1_SQRT_2;
    let cps = vec![
        HPoint::from_cartesian(Point3::new(1.0, 0.0, 0.0), 1.0),
        HPoint::from_cartesian(Point3::new(1.0, 1.0, 0.0), w),
        HPoint::from_cartesian(Point3::new(0.0, 1.0, 0.0), 1.0),
    ];
    let cp = ControlPolygon::new(cps).unwrap();
    let knots = KnotVector::new(vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0]).unwrap();
    let curve = NurbsCurve::new(Degree::new(2).unwrap(), cp, knots).unwrap();

    for i in 0..=16 {
        let t = i as f64 / 16.0;
        let p = curve.point_at(t);
        let r = (p.x * p.x + p.y * p.y).sqrt();
        assert!((r - 1.0).abs() < 1e-10, "r={} at t={}", r, t);
    }

    let _ = Vector3::new(1.0, 0.0, 0.0);
}

#[test]
fn rational_quarter_circle_length_matches_arc_length() {
    let w = FRAC_1_SQRT_2;
    let cps = vec![
        HPoint::from_cartesian(Point3::new(1.0, 0.0, 0.0), 1.0),
        HPoint::from_cartesian(Point3::new(1.0, 1.0, 0.0), w),
        HPoint::from_cartesian(Point3::new(0.0, 1.0, 0.0), 1.0),
    ];
    let cp = ControlPolygon::new(cps).unwrap();
    let knots = KnotVector::new(vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0]).unwrap();
    let curve = NurbsCurve::new(Degree::new(2).unwrap(), cp, knots).unwrap();
    let length = curve.length(0.0, 1.0);
    assert!((length - std::f64::consts::FRAC_PI_2).abs() < 1e-8);
}

#[test]
fn bezier_spans_returns_single_span_for_bezier_curve() {
    let pts = vec![
        Point3::new(-2.0, 0.0, 0.0),
        Point3::new(-1.0, 2.0, 0.0),
        Point3::new(1.0, -2.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
    ];
    let cp = ControlPolygon::from_cartesian(pts, &[1.0, 1.0, 1.0, 1.0]).unwrap();
    let curve = NurbsCurve::with_uniform_knots(Degree::new(3).unwrap(), cp).unwrap();

    let spans = curve.bezier_spans().unwrap();

    assert_eq!(spans.len(), 1);
    assert_eq!(spans[0].domain(), curve.domain());
}

#[test]
fn bezier_spans_splits_curve_at_interior_knots() {
    let pts = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(2.0, -1.0, 0.0),
        Point3::new(3.0, 0.0, 0.0),
    ];
    let cp = ControlPolygon::from_cartesian(pts, &[1.0, 1.0, 1.0, 1.0]).unwrap();
    let curve = NurbsCurve::with_uniform_knots(Degree::new(2).unwrap(), cp).unwrap();

    let spans = curve.bezier_spans().unwrap();

    assert_eq!(spans.len(), 2);
    for span in spans {
        let domain = span.domain();
        for i in 0..=8 {
            let u = domain.start + (domain.end - domain.start) * i as f64 / 8.0;
            let error = (span.point_at(u) - curve.point_at(u)).norm();
            assert!(error <= LINEAR_TOLERANCE, "u={u}, error={error}");
        }
    }
}

#[test]
fn bezier_spans_extracts_four_quadratic_circle_arcs() {
    let circle = Circle::new(Plane::xy(), 1.0);
    let curve = circle.to_nurbs().unwrap();

    let spans = curve.bezier_spans().unwrap();

    assert_eq!(spans.len(), 4);
    assert!(
        spans
            .iter()
            .all(|span| span.degree() == Degree::new(2).unwrap())
    );
    for span in spans {
        let domain = span.domain();
        for i in 0..=8 {
            let u = domain.start + (domain.end - domain.start) * i as f64 / 8.0;
            let point = span.point_at(u);
            let radius = (point.x * point.x + point.y * point.y).sqrt();
            assert!((radius - 1.0).abs() <= 1.0e-10);
        }
    }
}
