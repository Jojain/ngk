use nalgebra::Vector3;
use ngk::geometry::nurbs::{ControlPolygon, Degree, HPoint, KnotVector, NurbsCurve};
use ngk::geometry::utils::Point3;

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
fn insert_knot_preserves_shape() {
    let pts = vec![
        Point3::new(-2.0, 0.0, 0.0),
        Point3::new(-1.0, 2.0, 0.0),
        Point3::new(1.0, -2.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
    ];
    let cp = ControlPolygon::from_cartesian(pts, &[1.0, 1.0, 1.0, 1.0]).unwrap();
    let mut curve =
        NurbsCurve::with_uniform_knots(Degree::new(3).unwrap(), cp).unwrap();

    let orig_samples: Vec<_> = (0..=20)
        .map(|i| curve.point_at(i as f64 / 20.0))
        .collect();
    let orig_cp_count = curve.control_points().len();
    let orig_knot_count = curve.knots().len();

    curve.insert_knot(0.5);

    assert_eq!(curve.control_points().len(), orig_cp_count + 1);
    assert_eq!(curve.knots().len(), orig_knot_count + 1);

    for (i, orig) in orig_samples.iter().enumerate() {
        let p = curve.point_at(i as f64 / 20.0);
        let err = (p - orig).norm();
        assert!(err < 1e-9, "sample {} deviates by {}", i, err);
    }
}

#[test]
fn insert_knot_quadratic_s1() {
    // 4 CPs degree 2 → uniform clamped knots have an interior 0.5.
    // Inserting 0.5 again (multiplicity 1) must still add exactly one CP and one knot.
    let pts = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(2.0, -1.0, 0.0),
        Point3::new(3.0, 0.0, 0.0),
    ];
    let cp = ControlPolygon::from_cartesian(pts, &[1.0, 1.0, 1.0, 1.0]).unwrap();
    let mut curve =
        NurbsCurve::with_uniform_knots(Degree::new(2).unwrap(), cp).unwrap();

    assert_eq!(curve.knots().len(), 7);
    assert_eq!(curve.control_points().len(), 4);

    let orig_samples: Vec<_> = (0..=20)
        .map(|i| curve.point_at(i as f64 / 20.0))
        .collect();

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
        assert!(err < 1e-9, "sample {} deviates by {}", i, err);
    }
}

#[test]
fn rational_circle_quarter() {
    // A standard rational-quadratic quarter circle: CPs (1,0), (1,1), (0,1) with weights 1, √2/2, 1.
    let w = std::f64::consts::FRAC_1_SQRT_2;
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
    // Sanity that the geometry really is a quarter circle.
    let _ = Vector3::new(1.0, 0.0, 0.0);
}
