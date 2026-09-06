use std::f64::consts::TAU;

use nalgebra::{Rotation3, Vector3};
use ngk::geometry::axis::Axis3;
use ngk::geometry::{
    Curve, CurveGeometry, Ellipse, Frame, Interval, LINEAR_TOLERANCE, Periodicity, Point3,
    PointCoincidence,
};

fn ellipse() -> Ellipse {
    Ellipse::new(
        Frame::from_xy(Point3::new(1.0, 2.0, 3.0), Vector3::y(), -Vector3::x()),
        4.0,
        2.0,
    )
}

fn assert_point_near(actual: Point3, expected: Point3) {
    assert!(
        actual.coincides(expected, LINEAR_TOLERANCE),
        "expected {expected:?}, got {actual:?}"
    );
}

#[test]
fn ellipse_uses_its_frame_and_recovers_parameters() {
    let ellipse = ellipse();

    assert_point_near(ellipse.point_at(0.0), Point3::new(1.0, 6.0, 3.0));
    assert_point_near(
        ellipse.point_at(std::f64::consts::FRAC_PI_2),
        Point3::new(-1.0, 2.0, 3.0),
    );
    for parameter in [0.17, 1.23, 3.81, 5.77] {
        let recovered = ellipse.param_at(ellipse.point_at(parameter));
        let error = (recovered - parameter)
            .rem_euclid(TAU)
            .min((parameter - recovered).rem_euclid(TAU));
        assert!(error <= 1.0e-10, "parameter error at {parameter}: {error}");
    }
}

#[test]
fn ellipse_curve_forwards_domain_periodicity_and_projection() {
    let ellipse = ellipse();
    let curve = Curve::Ellipse(ellipse.clone());

    assert_eq!(curve.domain(), Interval::new(0.0, TAU));
    assert_eq!(curve.periodicity(), Periodicity::Periodic(TAU));
    assert_point_near(
        curve.project(Point3::new(1.0, 8.0, 9.0)),
        ellipse.point_at(0.0),
    );
}

#[test]
fn ellipse_converts_to_an_exact_rational_quadratic_point_set() {
    let ellipse = ellipse();
    let nurbs = ellipse.to_nurbs().expect("ellipse should convert exactly");

    assert_eq!(nurbs.degree().get(), 2);
    assert!(nurbs.is_rational());
    assert_eq!(nurbs.domain(), Interval::new(0.0, TAU));
    for parameter in [0.19, 0.83, 1.91, 2.77, 4.31, 5.63] {
        let local = ellipse.frame().coordinates_of(nurbs.point_at(parameter));
        let equation =
            (local.x / ellipse.major_radius()).powi(2) + (local.y / ellipse.minor_radius()).powi(2);
        assert!(
            (equation - 1.0).abs() <= 1.0e-9,
            "NURBS point at {parameter} is not on the ellipse: {equation}"
        );
        assert!(local.z.abs() <= 1.0e-9);
    }
}

#[test]
fn ellipse_rotation_and_translation_preserve_parameterization() {
    let ellipse = ellipse();
    let axis = Axis3::new(Point3::origin(), Vector3::z());
    let angle = 0.63;
    let rotation = Rotation3::from_axis_angle(&axis.direction, angle);
    let rotated = ellipse.rotated(axis, angle).unwrap();
    let offset = Vector3::new(-2.0, 5.0, 1.5);
    let translated = ellipse.translated(offset).unwrap();

    for parameter in [0.0, 0.37, 2.4, 5.9] {
        let rotated_offset = rotation * (ellipse.point_at(parameter) - axis.origin);
        assert_point_near(rotated.point_at(parameter), axis.origin + rotated_offset);
        assert_point_near(
            translated.point_at(parameter),
            ellipse.point_at(parameter) + offset,
        );
    }
}

#[test]
fn ellipse_bbox_over_contains_an_off_axis_arc() {
    let ellipse = ellipse();
    let interval = Interval::new(0.31, 2.17);
    let bounds = ellipse
        .bbox_over(interval)
        .expect("an ellipse arc has finite exact bounds");

    for index in 0..=128 {
        let parameter = interval.start + interval.length() * index as f64 / 128.0;
        assert!(
            bounds.contains_point(ellipse.point_at(parameter), LINEAR_TOLERANCE),
            "ellipse point at {parameter} escaped its analytic bounds"
        );
    }
}
