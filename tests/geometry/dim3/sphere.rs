use std::f64::consts::{FRAC_PI_2, TAU};

use nalgebra::{Rotation3, Vector3};
use ngk::geometry::axis::Axis3;
use ngk::geometry::{
    Frame, Interval, LINEAR_TOLERANCE, Point2, Point3, PointCoincidence, Sphere, Surface,
    SurfaceGeometry, SurfacePeriodicity,
};

fn sphere() -> Sphere {
    Sphere::new(
        Frame::from_xy(Point3::new(1.0, 2.0, 3.0), Vector3::y(), -Vector3::x()),
        2.5,
    )
}

fn assert_point_near(actual: Point3, expected: Point3) {
    assert!(
        actual.coincides(expected, LINEAR_TOLERANCE),
        "expected {expected:?}, got {actual:?}"
    );
}

#[test]
fn sphere_uses_longitude_latitude_parameterization() {
    let sphere = sphere();

    assert_point_near(sphere.point_at(0.0, 0.0), Point3::new(1.0, 4.5, 3.0));
    assert_point_near(sphere.point_at(FRAC_PI_2, 0.0), Point3::new(-1.5, 2.0, 3.0));
    assert_point_near(sphere.point_at(1.37, FRAC_PI_2), Point3::new(1.0, 2.0, 5.5));
    assert_point_near(
        sphere.point_at(4.21, -FRAC_PI_2),
        Point3::new(1.0, 2.0, 0.5),
    );

    let normal = sphere.normal_at(0.73, -0.41);
    let radial = (sphere.point_at(0.73, -0.41) - sphere.frame().origin).normalize();
    assert!((*normal - radial).norm() <= 1.0e-12);
}

#[test]
fn sphere_forwards_domain_periodicity_and_pole_degeneracy() {
    let surface = Surface::Sphere(sphere());

    assert_eq!(
        surface.domain(),
        (
            Interval::new(0.0, TAU),
            Interval::new(-FRAC_PI_2, FRAC_PI_2)
        )
    );
    assert_eq!(surface.periodicity(), SurfacePeriodicity::UPeriodic(TAU));
    assert!(!surface.is_degenerate_at(0.3, 0.0));
    assert!(surface.is_degenerate_at(0.3, FRAC_PI_2));
    assert!(surface.is_degenerate_at(4.2, -FRAC_PI_2));

    assert_point_near(
        Point3::from(*surface.normal_at(0.3, FRAC_PI_2)),
        Point3::from(*sphere().frame().z_dir),
    );
}

#[test]
fn sphere_nurbs_patch_matches_off_knot_parameters_through_param_map() {
    let sphere = sphere();
    let u = Interval::new(0.23, 5.71);
    let v = Interval::new(-1.21, 1.37);
    let nurbs = sphere.to_nurbs_over(u, v).unwrap();
    let map = sphere.param_map_over(u, v);

    assert_eq!(nurbs.degree_u().get(), 2);
    assert_eq!(nurbs.degree_v().get(), 2);
    assert!(
        nurbs
            .control_points()
            .as_slice()
            .iter()
            .any(|point| (point.weight() - 1.0).abs() > 1.0e-12)
    );
    assert_eq!(nurbs.domain_u(), u);
    assert_eq!(nurbs.domain_v(), v);

    for (analytic_u, analytic_v) in [(0.37, -0.93), (1.14, 0.17), (4.89, 1.11)] {
        let mapped = map.map(Point2::new(analytic_u, analytic_v));
        assert_point_near(
            nurbs.point_at(mapped.x, mapped.y),
            sphere.point_at(analytic_u, analytic_v),
        );
        let recovered = map.inverse(mapped);
        assert!((recovered.x - analytic_u).abs() <= 1.0e-11);
        assert!((recovered.y - analytic_v).abs() <= 1.0e-11);
    }
}

#[test]
fn sphere_closest_parameter_round_trips_away_from_poles() {
    let sphere = sphere();

    for (u, v) in [(0.17, -1.1), (1.73, 0.31), (5.81, 1.23)] {
        let recovered = sphere.closest_parameter(sphere.point_at(u, v));
        let u_error = (recovered.x - u)
            .rem_euclid(TAU)
            .min((u - recovered.x).rem_euclid(TAU));
        assert!(
            u_error <= 1.0e-11,
            "longitude error at ({u}, {v}): {u_error}"
        );
        assert!((recovered.y - v).abs() <= 1.0e-11);
    }
}

#[test]
fn sphere_bbox_over_contains_a_trimmed_patch() {
    let sphere = sphere();
    let u = Interval::new(0.31, 4.77);
    let v = Interval::new(-1.19, 0.83);
    let bounds = sphere
        .bbox_over(u, v)
        .expect("a finite sphere patch has exact bounds");

    for iu in 0..=64 {
        for iv in 0..=32 {
            let parameter_u = u.start + u.length() * iu as f64 / 64.0;
            let parameter_v = v.start + v.length() * iv as f64 / 32.0;
            assert!(
                bounds.contains_point(sphere.point_at(parameter_u, parameter_v), LINEAR_TOLERANCE,),
                "sphere point ({parameter_u}, {parameter_v}) escaped its bounds"
            );
        }
    }
}

#[test]
fn sphere_rotation_and_translation_preserve_parameterization() {
    let sphere = sphere();
    let axis = Axis3::new(Point3::new(-1.0, 0.5, 0.0), Vector3::z());
    let angle = 0.63;
    let rotation = Rotation3::from_axis_angle(&axis.direction, angle);
    let rotated = sphere.rotated(axis, angle).unwrap();
    let offset = Vector3::new(-2.0, 5.0, 1.5);
    let translated = sphere.translated(offset).unwrap();

    for (u, v) in [(0.37, -0.7), (2.4, 0.13), (5.9, 1.1)] {
        let rotated_offset = rotation * (sphere.point_at(u, v) - axis.origin);
        assert_point_near(rotated.point_at(u, v), axis.origin + rotated_offset);
        assert_point_near(translated.point_at(u, v), sphere.point_at(u, v) + offset);
    }
}
