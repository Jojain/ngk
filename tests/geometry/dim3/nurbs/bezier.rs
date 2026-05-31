use std::f64::consts::FRAC_1_SQRT_2;

use ngk::geometry::{
    Bezier, ControlPolygon, Degree, HPoint, Interval, LINEAR_TOLERANCE, Point3, PointCoincidence,
};

fn assert_point_near(actual: Point3, expected: Point3) {
    assert!(
        actual.coincides(expected, LINEAR_TOLERANCE),
        "expected {expected:?}, got {actual:?}"
    );
}

#[test]
fn quadratic_bezier_midpoint() {
    let bezier = Bezier::new(
        Degree::new(2).unwrap(),
        ControlPolygon::from_cartesian(
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(2.0, 0.0, 0.0),
            ],
            &[1.0, 1.0, 1.0],
        )
        .unwrap(),
        Interval::new(0.0, 1.0),
    )
    .unwrap();

    assert_point_near(bezier.point_at(0.5), Point3::new(1.0, 0.5, 0.0));
}

#[test]
fn rational_quarter_circle_stays_on_circle() {
    let bezier = Bezier::new(
        Degree::new(2).unwrap(),
        ControlPolygon::new(vec![
            HPoint::from_cartesian(Point3::new(1.0, 0.0, 0.0), 1.0),
            HPoint::from_cartesian(Point3::new(1.0, 1.0, 0.0), FRAC_1_SQRT_2),
            HPoint::from_cartesian(Point3::new(0.0, 1.0, 0.0), 1.0),
        ])
        .unwrap(),
        Interval::new(0.0, 1.0),
    )
    .unwrap();

    for i in 0..=16 {
        let point = bezier.point_at(i as f64 / 16.0);
        let radius = (point.x * point.x + point.y * point.y).sqrt();
        assert!(
            (radius - 1.0).abs() <= 1.0e-10,
            "radius {radius} at sample {i}"
        );
    }
}

#[test]
fn bezier_bbox_contains_curve_samples() {
    let bezier = Bezier::new(
        Degree::new(3).unwrap(),
        ControlPolygon::from_cartesian(
            vec![
                Point3::new(-1.0, 0.0, 0.0),
                Point3::new(-0.5, 1.0, 0.0),
                Point3::new(0.5, -1.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
            ],
            &[1.0, 1.0, 1.0, 1.0],
        )
        .unwrap(),
        Interval::new(2.0, 4.0),
    )
    .unwrap();
    let bbox = bezier.bbox();

    for i in 0..=32 {
        let u = 2.0 + 2.0 * i as f64 / 32.0;
        assert!(bbox.contains_point(bezier.point_at(u), LINEAR_TOLERANCE));
    }
}

#[test]
fn subdivide_preserves_shape_and_domains() {
    let bezier = Bezier::new(
        Degree::new(3).unwrap(),
        ControlPolygon::from_cartesian(
            vec![
                Point3::new(-1.0, 0.0, 0.0),
                Point3::new(-0.5, 1.0, 0.0),
                Point3::new(0.5, -1.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
            ],
            &[1.0, 1.0, 1.0, 1.0],
        )
        .unwrap(),
        Interval::new(2.0, 4.0),
    )
    .unwrap();

    let (left, right) = bezier.subdivide(3.0).unwrap();

    assert_eq!(left.domain(), Interval::new(2.0, 3.0));
    assert_eq!(right.domain(), Interval::new(3.0, 4.0));
    for i in 0..=16 {
        let u = 2.0 + i as f64 / 16.0;
        assert_point_near(left.point_at(u), bezier.point_at(u));
    }
    for i in 0..=16 {
        let u = 3.0 + i as f64 / 16.0;
        assert_point_near(right.point_at(u), bezier.point_at(u));
    }
}
