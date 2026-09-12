use std::collections::HashMap;
use std::f64::consts::{FRAC_PI_2, TAU};

use nalgebra::{Rotation3, Vector3};
use ngk::geometry::axis::Axis3;
use ngk::geometry::{
    Frame, Interval, LINEAR_TOLERANCE, Point2, Point3, PointCoincidence, Surface, SurfaceGeometry,
    SurfacePeriodicity, Torus,
};
use ngk::tessellate::{SurfaceOpts, tessellate_surface_patch};

fn torus() -> Torus {
    Torus::new(
        Frame::from_xy(Point3::new(1.0, 2.0, 3.0), Vector3::y(), -Vector3::x()),
        3.0,
        1.0,
    )
}

fn assert_point_near(actual: Point3, expected: Point3) {
    assert!(
        actual.coincides(expected, LINEAR_TOLERANCE),
        "expected {expected:?}, got {actual:?}"
    );
}

/// A torus walks the main circle with `u` and the tube with `v`.
#[test]
fn torus_uses_longitude_and_tube_parameterization() {
    let torus = Torus::new(Frame::xyz(), 3.0, 1.0);

    assert_point_near(torus.point_at(0.0, 0.0), Point3::new(4.0, 0.0, 0.0));
    assert_point_near(torus.point_at(FRAC_PI_2, 0.0), Point3::new(0.0, 4.0, 0.0));
    assert_point_near(torus.point_at(0.0, FRAC_PI_2), Point3::new(3.0, 0.0, 1.0));
    assert_point_near(
        torus.point_at(0.0, std::f64::consts::PI),
        Point3::new(2.0, 0.0, 0.0),
    );
}

/// The normal leaves the tube's centre circle outward, tilting with the tube.
#[test]
fn torus_normal_points_away_from_the_tube_centre_circle() {
    let torus = Torus::new(Frame::xyz(), 3.0, 1.0);

    for (u, v, expected) in [
        (0.0, 0.0, Vector3::x()),
        (0.0, FRAC_PI_2, Vector3::z()),
        (0.0, std::f64::consts::PI, -Vector3::x()),
        (FRAC_PI_2, FRAC_PI_2, Vector3::z()),
    ] {
        let normal = *torus.normal_at(u, v);
        assert!(
            (normal - expected).norm() <= 1.0e-9,
            "normal at ({u}, {v}) should be {expected:?}, got {normal:?}"
        );
    }
}

#[test]
fn torus_forwards_domain_periodicity_and_closedness() {
    let surface = Surface::Torus(torus());

    assert_eq!(surface.domain().0, Interval::new(0.0, TAU));
    assert_eq!(surface.domain().1, Interval::new(0.0, TAU));
    assert_eq!(
        surface.periodicity(),
        SurfacePeriodicity::UVPeriodic(TAU, TAU)
    );
    assert!(!surface.is_degenerate_at(0.3, 1.7));
    assert!(
        surface.is_closed(),
        "a torus has no boundary in either direction"
    );
}

#[test]
fn torus_nurbs_patch_matches_off_knot_parameters_through_param_map() {
    let torus = torus();
    let turn = Interval::new(0.0, TAU);
    let nurbs = torus.to_nurbs_over(turn, turn).unwrap();
    let map = torus.param_map_over(turn, turn);

    assert_eq!(nurbs.degree_u().get(), 2);
    assert_eq!(nurbs.degree_v().get(), 2);
    assert_eq!(nurbs.domain_u(), turn);
    assert_eq!(nurbs.domain_v(), turn);
    assert!(
        nurbs
            .control_points()
            .as_slice()
            .iter()
            .any(|point| (point.weight() - 1.0).abs() > 1.0e-12),
        "a torus is rational in both directions"
    );

    for (analytic_u, analytic_v) in [(0.37, 0.9), (1.14, 2.6), (4.89, 3.11)] {
        let mapped = map.map(Point2::new(analytic_u, analytic_v));
        assert_point_near(
            nurbs.point_at(mapped.x, mapped.y),
            torus.point_at(analytic_u, analytic_v),
        );
        let recovered = map.inverse(mapped);
        assert!((recovered.x - analytic_u).abs() <= 1.0e-11);
        assert!((recovered.y - analytic_v).abs() <= 1.0e-11);
    }
}

/// A partial box converts to a NURBS patch over exactly that box.
#[test]
fn torus_nurbs_patch_honours_a_partial_box() {
    let torus = torus();
    let u = Interval::new(0.4, 2.7);
    let v = Interval::new(1.1, 5.2);
    let nurbs = torus.to_nurbs_over(u, v).unwrap();
    let map = torus.param_map_over(u, v);

    assert_eq!(
        nurbs.domain_u(),
        u,
        "the patch must span the requested u box"
    );
    assert_eq!(
        nurbs.domain_v(),
        v,
        "the patch must span the requested v box"
    );

    for fraction_u in [0.0, 0.13, 0.5, 0.77, 1.0] {
        for fraction_v in [0.0, 0.29, 0.5, 0.81, 1.0] {
            let analytic_u = u.start + u.length() * fraction_u;
            let analytic_v = v.start + v.length() * fraction_v;
            let mapped = map.map(Point2::new(analytic_u, analytic_v));
            assert_point_near(
                nurbs.point_at(mapped.x, mapped.y),
                torus.point_at(analytic_u, analytic_v),
            );
        }
    }
}

#[test]
fn torus_closest_parameter_round_trips_on_both_sides_of_the_seam() {
    let torus = torus();

    for (u, v) in [
        (0.17, 0.83),
        (1.73, 5.31),
        (TAU - 1.0e-3, 0.4),
        (0.4, TAU - 1.0e-3),
    ] {
        let recovered = torus.closest_parameter(torus.point_at(u, v));
        let u_error = (recovered.x - u)
            .rem_euclid(TAU)
            .min((u - recovered.x).rem_euclid(TAU));
        let v_error = (recovered.y - v)
            .rem_euclid(TAU)
            .min((v - recovered.y).rem_euclid(TAU));
        assert!(
            u_error <= 1.0e-11,
            "longitude error at ({u}, {v}): {u_error}"
        );
        assert!(v_error <= 1.0e-11, "tube error at ({u}, {v}): {v_error}");
    }
}

#[test]
fn torus_bbox_over_contains_a_trimmed_patch() {
    let torus = torus();
    let u = Interval::new(0.31, 4.77);
    let v = Interval::new(0.41, 5.19);
    let bounds = torus
        .bbox_over(u, v)
        .expect("a finite torus patch has bounds");

    for iu in 0..=64 {
        for iv in 0..=32 {
            let parameter_u = u.start + u.length() * iu as f64 / 64.0;
            let parameter_v = v.start + v.length() * iv as f64 / 32.0;
            assert!(
                bounds.contains_point(torus.point_at(parameter_u, parameter_v), LINEAR_TOLERANCE),
                "torus point ({parameter_u}, {parameter_v}) escaped its bounds"
            );
        }
    }
}

#[test]
fn torus_rotation_and_translation_preserve_parameterization() {
    let torus = torus();
    let axis = Axis3::new(Point3::new(-1.0, 0.5, 0.0), Vector3::z());
    let angle = 0.63;
    let rotation = Rotation3::from_axis_angle(&axis.direction, angle);
    let rotated = torus.rotated(axis, angle).unwrap();
    let offset = Vector3::new(-2.0, 5.0, 1.5);
    let translated = torus.translated(offset).unwrap();

    for (u, v) in [(0.37, 2.7), (2.4, 0.13), (5.9, 3.1)] {
        let rotated_offset = rotation * (torus.point_at(u, v) - axis.origin);
        assert_point_near(rotated.point_at(u, v), axis.origin + rotated_offset);
        assert_point_near(translated.point_at(u, v), torus.point_at(u, v) + offset);
    }
}

/// Both parameter directions wrap, so the mesh closes only if the last column
/// *and* the last row index back onto the first.
#[test]
fn torus_tessellation_closes_both_parameter_directions() {
    let surface = Surface::Torus(torus());
    let mesh = tessellate_surface_patch(
        &surface,
        (0.0, TAU),
        (0.0, TAU),
        SurfaceOpts { nu: 24, nv: 16 },
    );

    assert!(!mesh.is_empty());
    let mut uses = HashMap::new();
    for triangle in mesh.indices.chunks_exact(3) {
        for pair in [
            (triangle[0], triangle[1]),
            (triangle[1], triangle[2]),
            (triangle[2], triangle[0]),
        ] {
            let edge = (pair.0.min(pair.1), pair.0.max(pair.1));
            *uses.entry(edge).or_insert(0) += 1;
        }
    }
    assert!(
        uses.values().all(|count| *count == 2),
        "every mesh edge of a closed torus is shared by exactly two triangles"
    );
    assert!(
        mesh.positions.iter().all(|point| {
            let uv = surface.param_at(*point).unwrap();
            (surface.point_at(uv.x, uv.y) - point).norm() <= LINEAR_TOLERANCE
        }),
        "every mesh vertex sits on the torus"
    );
}
