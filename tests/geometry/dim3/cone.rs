use std::f64::consts::{FRAC_PI_6, TAU};

use nalgebra::{Rotation3, Vector3};
use ngk::geometry::axis::Axis3;
use ngk::geometry::{
    Cone, Frame, Interval, LINEAR_TOLERANCE, Point2, Point3, PointCoincidence, Surface,
    SurfaceGeometry, SurfacePeriodicity,
};
use ngk::tessellate::{SurfaceOpts, tessellate_surface_patch};

fn cone() -> Cone {
    Cone::new(
        Frame::from_xy(Point3::new(1.0, 2.0, 3.0), Vector3::y(), -Vector3::x()),
        2.0,
        FRAC_PI_6,
    )
}

fn assert_point_near(actual: Point3, expected: Point3) {
    assert!(
        actual.coincides(expected, LINEAR_TOLERANCE),
        "expected {expected:?}, got {actual:?}"
    );
}

#[test]
fn cone_uses_angular_and_axial_distance_parameterization() {
    let cone = Cone::new(Frame::xyz(), 2.0, FRAC_PI_6);

    assert_point_near(cone.point_at(0.0, 0.0), Point3::new(2.0, 0.0, 0.0));
    assert_point_near(
        cone.point_at(std::f64::consts::FRAC_PI_2, 2.0),
        Point3::new(0.0, 3.0, 3.0_f64.sqrt()),
    );

    let expected_normal = Vector3::new(FRAC_PI_6.cos(), 0.0, -FRAC_PI_6.sin());
    assert!((*cone.normal_at(0.0, 0.0) - expected_normal).norm() <= 1.0e-12);
}

#[test]
fn cone_forwards_domain_periodicity_and_apex_degeneracy() {
    let cone = cone();
    let surface = Surface::Cone(cone.clone());
    let apex_v = cone.apex_parameter().expect("sloped cone has an apex");

    assert_eq!(surface.domain().0, Interval::new(0.0, TAU));
    assert!(!surface.domain().1.is_finite());
    assert_eq!(surface.periodicity(), SurfacePeriodicity::UPeriodic(TAU));
    assert!(!surface.is_degenerate_at(0.3, 0.0));
    assert!(surface.is_degenerate_at(0.3, apex_v));

    let expected = *cone.frame().x_dir * FRAC_PI_6.cos() - *cone.frame().z_dir * FRAC_PI_6.sin();
    assert!((*surface.normal_at(0.0, apex_v) - expected).norm() <= 1.0e-12);
    assert_eq!(cone.closest_parameter(cone.point_at(2.7, apex_v)).x, 0.0);
}

#[test]
fn cone_nurbs_patch_matches_off_knot_parameters_through_param_map() {
    let cone = cone();
    let u = Interval::new(0.23, 5.71);
    let v = Interval::new(-2.31, 4.17);
    let nurbs = cone.to_nurbs_over(u, v).unwrap();
    let map = cone.param_map_over(u, v);

    assert_eq!(nurbs.degree_u().get(), 2);
    assert_eq!(nurbs.degree_v().get(), 1);
    assert_eq!(nurbs.domain_u(), u);
    assert_eq!(nurbs.domain_v(), v);
    assert!(
        nurbs
            .control_points()
            .as_slice()
            .iter()
            .any(|point| (point.weight() - 1.0).abs() > 1.0e-12)
    );

    for (analytic_u, analytic_v) in [(0.37, -1.93), (1.14, 0.17), (4.89, 3.11)] {
        let mapped = map.map(Point2::new(analytic_u, analytic_v));
        assert_point_near(
            nurbs.point_at(mapped.x, mapped.y),
            cone.point_at(analytic_u, analytic_v),
        );
        let recovered = map.inverse(mapped);
        assert!((recovered.x - analytic_u).abs() <= 1.0e-11);
        assert!((recovered.y - analytic_v).abs() <= 1.0e-11);
    }
}

#[test]
fn cone_closest_parameter_round_trips_on_both_sides_of_the_apex() {
    let cone = cone();

    for (u, v) in [(0.17, -5.3), (1.73, -1.31), (5.81, 3.23)] {
        let recovered = cone.closest_parameter(cone.point_at(u, v));
        let u_error = (recovered.x - u)
            .rem_euclid(TAU)
            .min((u - recovered.x).rem_euclid(TAU));
        assert!(u_error <= 1.0e-11, "angular error at ({u}, {v}): {u_error}");
        assert!((recovered.y - v).abs() <= 1.0e-11);
    }
}

#[test]
fn cone_bbox_over_contains_a_trimmed_patch() {
    let cone = cone();
    let u = Interval::new(0.31, 4.77);
    let v = Interval::new(-5.19, 3.83);
    let bounds = cone
        .bbox_over(u, v)
        .expect("a finite cone patch has exact bounds");

    for iu in 0..=64 {
        for iv in 0..=32 {
            let parameter_u = u.start + u.length() * iu as f64 / 64.0;
            let parameter_v = v.start + v.length() * iv as f64 / 32.0;
            assert!(
                bounds.contains_point(cone.point_at(parameter_u, parameter_v), LINEAR_TOLERANCE),
                "cone point ({parameter_u}, {parameter_v}) escaped its bounds"
            );
        }
    }
}

#[test]
fn cone_rotation_and_translation_preserve_parameterization() {
    let cone = cone();
    let axis = Axis3::new(Point3::new(-1.0, 0.5, 0.0), Vector3::z());
    let angle = 0.63;
    let rotation = Rotation3::from_axis_angle(&axis.direction, angle);
    let rotated = cone.rotated(axis, angle).unwrap();
    let offset = Vector3::new(-2.0, 5.0, 1.5);
    let translated = cone.translated(offset).unwrap();

    for (u, v) in [(0.37, -2.7), (2.4, 0.13), (5.9, 3.1)] {
        let rotated_offset = rotation * (cone.point_at(u, v) - axis.origin);
        assert_point_near(rotated.point_at(u, v), axis.origin + rotated_offset);
        assert_point_near(translated.point_at(u, v), cone.point_at(u, v) + offset);
    }
}

#[test]
fn cone_tessellation_collapses_the_apex_without_zero_area_triangles() {
    let cone = cone();
    let apex_v = cone.apex_parameter().unwrap();
    let surface = Surface::Cone(cone);
    let mesh = tessellate_surface_patch(
        &surface,
        (0.0, TAU),
        (apex_v, apex_v + 3.0),
        SurfaceOpts { nu: 24, nv: 8 },
    );

    assert!(!mesh.is_empty());
    assert!(mesh.indices.chunks_exact(3).all(|triangle| {
        let [a, b, c] = [
            triangle[0] as usize,
            triangle[1] as usize,
            triangle[2] as usize,
        ];
        (mesh.positions[b] - mesh.positions[a])
            .cross(&(mesh.positions[c] - mesh.positions[a]))
            .norm()
            > LINEAR_TOLERANCE * LINEAR_TOLERANCE
    }));
}
