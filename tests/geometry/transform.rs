use std::f64::consts::{FRAC_PI_3, FRAC_PI_6, TAU};

use nalgebra::{Rotation3, Vector3};
use ngk::geometry::axis::Axis3;
use ngk::geometry::{
    BBox, Circle, Cone, Curve, CurveGeometry, Cylinder, Ellipse, Frame, LINEAR_TOLERANCE, Line,
    NurbsCurve, Plane, Point3, PointCoincidence, Rigid, RuledSurface, Sphere, Surface,
    SurfaceOfRevolution, Torus,
};
use radians::Rad64;

fn tilted_axis() -> Axis3 {
    Axis3::new(Point3::new(-1.0, 0.5, 2.0), Vector3::new(1.0, -2.0, 3.0))
}

/// A motion with both a rotation and a translation, so nothing passes by
/// accidentally being a pure translation.
fn motion() -> Rigid {
    Rigid::rotation(tilted_axis(), Rad64::new(0.63))
        .compose(Rigid::translation(Vector3::new(-2.0, 5.0, 1.5)))
}

fn tilted_frame() -> Frame {
    Frame::from_xy(
        Point3::new(1.0, 2.0, 3.0),
        Vector3::new(1.0, 1.0, 0.0),
        Vector3::new(-1.0, 1.0, 1.0),
    )
}

fn assert_point_near(actual: Point3, expected: Point3) {
    assert!(
        actual.coincides(expected, LINEAR_TOLERANCE),
        "expected {expected:?}, got {actual:?}"
    );
}

/// Every curve variant, so the checks below cover the whole enum.
fn every_curve() -> Vec<Curve> {
    vec![
        Curve::line(Point3::new(1.0, 0.0, 0.0), Point3::new(2.0, 1.0, 1.0)),
        Curve::circle(Plane::from_frame(tilted_frame()), 2.0),
        Curve::Ellipse(Ellipse::new(tilted_frame(), 4.0, 2.0)),
        Curve::Nurbs(
            NurbsCurve::interpolate(&[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 2.0, 0.5),
                Point3::new(3.0, 1.0, -1.0),
                Point3::new(4.0, 3.0, 2.0),
            ])
            .expect("four samples should interpolate"),
        ),
    ]
}

/// Every surface variant, so the checks below cover the whole enum.
fn every_surface() -> Vec<Surface> {
    vec![
        Surface::Plane(Plane::from_frame(tilted_frame())),
        Surface::Cylinder(Cylinder::new(
            Point3::new(1.0, 2.0, 3.0),
            Vector3::y(),
            -Vector3::x(),
            2.0,
        )),
        Surface::Sphere(Sphere::new(tilted_frame(), 2.5)),
        Surface::Cone(Cone::new(tilted_frame(), 2.0, FRAC_PI_6)),
        Surface::Torus(Torus::new(tilted_frame(), 3.0, 1.0)),
        Surface::Ruled(RuledSurface::new(
            Curve::line(Point3::new(1.0, 0.0, 0.0), Point3::new(2.0, 1.0, 1.0)),
            Vector3::new(0.0, 0.0, 2.0),
        )),
        Surface::Revolution(SurfaceOfRevolution::new(
            Curve::line(Point3::new(1.0, 0.0, 0.0), Point3::new(2.0, 0.0, 1.0)),
            Axis3::z(),
        )),
        Surface::Nurbs(
            Surface::Cylinder(Cylinder::new(
                Point3::new(1.0, 2.0, 3.0),
                Vector3::y(),
                -Vector3::x(),
                2.0,
            ))
            .to_nurbs()
            .expect("a cylinder converts to NURBS"),
        ),
    ]
}

/// The name of a variant, so a failure says which arm broke.
fn surface_variant(surface: &Surface) -> &'static str {
    match surface {
        Surface::Plane(_) => "Plane",
        Surface::Cylinder(_) => "Cylinder",
        Surface::Sphere(_) => "Sphere",
        Surface::Cone(_) => "Cone",
        Surface::Torus(_) => "Torus",
        Surface::Ruled(_) => "Ruled",
        Surface::Revolution(_) => "Revolution",
        Surface::Nurbs(_) => "Nurbs",
    }
}

fn curve_variant(curve: &Curve) -> &'static str {
    match curve {
        Curve::Line(_) => "Line",
        Curve::Circle(_) => "Circle",
        Curve::Ellipse(_) => "Ellipse",
        Curve::Nurbs(_) => "Nurbs",
    }
}

#[test]
fn identity_moves_nothing() {
    let identity = Rigid::identity();
    let point = Point3::new(1.0, -2.0, 3.0);

    assert_point_near(identity.apply(point), point);
    assert_eq!(identity.apply_vector(Vector3::x()), Vector3::x());
}

#[test]
fn rotation_turns_about_the_axis_it_is_given() {
    let axis = Axis3::new(Point3::new(0.0, 0.0, 5.0), Vector3::z());
    let rotated = Rigid::rotation(axis, Rad64::QUARTER_TURN).apply(Point3::new(2.0, 0.0, 5.0));

    assert_point_near(rotated, Point3::new(0.0, 2.0, 5.0));
}

#[test]
fn rotation_about_an_offset_axis_leaves_that_axis_fixed() {
    let axis = tilted_axis();
    let moved = Rigid::rotation(axis, Rad64::new(1.1)).apply(axis.origin);

    assert_point_near(moved, axis.origin);
}

#[test]
fn compose_applies_the_left_motion_first() {
    let quarter = Rigid::rotation(Axis3::z(), Rad64::QUARTER_TURN);
    let lift = Rigid::translation(Vector3::new(10.0, 0.0, 0.0));
    let point = Point3::new(1.0, 0.0, 0.0);

    // Rotate, then translate: the translation is not itself turned.
    assert_point_near(
        quarter.compose(lift).apply(point),
        Point3::new(10.0, 1.0, 0.0),
    );
    // Translate, then rotate: the offset comes along into the turn.
    assert_point_near(
        lift.compose(quarter).apply(point),
        Point3::new(0.0, 11.0, 0.0),
    );
}

#[test]
fn inverse_round_trips_every_point() {
    let r = motion();
    let back = r.inverse();

    for point in [
        Point3::origin(),
        Point3::new(3.0, -1.0, 4.0),
        Point3::new(-7.0, 2.5, 0.25),
    ] {
        assert_point_near(back.apply(r.apply(point)), point);
    }
}

#[test]
fn between_frames_carries_one_frame_onto_the_other() {
    let from = Frame::xyz();
    let to = tilted_frame();
    let moved = from.moved(&Rigid::between_frames(&from, &to));

    assert_point_near(moved.origin, to.origin);
    for (actual, expected) in [
        (moved.x_dir, to.x_dir),
        (moved.y_dir, to.y_dir),
        (moved.z_dir, to.z_dir),
    ] {
        assert!(
            (actual.into_inner() - expected.into_inner()).norm() <= LINEAR_TOLERANCE,
            "expected {expected:?}, got {actual:?}"
        );
    }
}

/// The claim the quaternion representation is there to make good on: a chain
/// long enough to matter must not take a frame out of square.
#[test]
fn composing_a_rotation_sixty_times_keeps_a_frame_orthonormal() {
    let step = Rigid::rotation(tilted_axis(), Rad64::new(TAU / 60.0));
    let mut whole = Rigid::identity();
    for _ in 0..60 {
        whole = whole.compose(step);
    }

    let frame = tilted_frame().moved(&whole);
    let axes = [frame.x_dir, frame.y_dir, frame.z_dir];
    for axis in axes {
        assert!(
            (axis.norm() - 1.0).abs() <= 1.0e-12,
            "axis {axis:?} lost unit length after sixty compositions"
        );
    }
    for (first, second) in [(0, 1), (1, 2), (2, 0)] {
        let dot = axes[first].dot(&axes[second]);
        assert!(
            dot.abs() <= 1.0e-12,
            "axes {first} and {second} went out of square after sixty compositions, dot {dot}"
        );
    }

    // A full turn in sixty steps is the identity, so the frame comes home.
    assert_point_near(frame.origin, tilted_frame().origin);
}

#[test]
fn every_curve_variant_survives_a_rigid_motion_as_itself() {
    let r = motion();
    for curve in every_curve() {
        let moved = curve.moved(&r);
        assert_eq!(
            curve_variant(&curve),
            curve_variant(&moved),
            "a rigid motion must not degrade {}",
            curve_variant(&curve)
        );
    }
}

#[test]
fn every_surface_variant_survives_a_rigid_motion_as_itself() {
    let r = motion();
    for surface in every_surface() {
        let moved = surface.moved(&r);
        assert_eq!(
            surface_variant(&surface),
            surface_variant(&moved),
            "a rigid motion must not degrade {}",
            surface_variant(&surface)
        );
    }
}

/// The contract `moved` rests on: the image's parameter means what the
/// source's did, so no stored interval has to be recomputed.
#[test]
fn every_curve_variant_keeps_its_parameterization() {
    let r = motion();
    for curve in every_curve() {
        let moved = curve.moved(&r);
        let domain = curve.domain().or_extent(1.0);
        for step in 0..=8 {
            let t = domain.start + domain.length() * f64::from(step) / 8.0;
            let expected = r.apply(curve.point_at(t));
            assert!(
                moved.point_at(t).coincides(expected, LINEAR_TOLERANCE),
                "{} re-parameterised at {t}: expected {expected:?}, got {:?}",
                curve_variant(&curve),
                moved.point_at(t)
            );
        }
    }
}

#[test]
fn every_surface_variant_keeps_its_parameterization() {
    let r = motion();
    for surface in every_surface() {
        let moved = surface.moved(&r);
        let (u, v) = surface.domain();
        let u = u.or_extent(1.0);
        let v = v.or_extent(1.0);
        for iu in 0..=4 {
            for iv in 0..=4 {
                let pu = u.start + u.length() * f64::from(iu) / 4.0;
                let pv = v.start + v.length() * f64::from(iv) / 4.0;
                let expected = r.apply(surface.point_at(pu, pv));
                assert!(
                    moved.point_at(pu, pv).coincides(expected, LINEAR_TOLERANCE),
                    "{} re-parameterised at ({pu}, {pv}): expected {expected:?}, got {:?}",
                    surface_variant(&surface),
                    moved.point_at(pu, pv)
                );
            }
        }
    }
}

/// A rigid motion cannot change a length, so a line's affine parameter — which
/// is a length — has to come through untouched.
#[test]
fn a_moved_line_keeps_its_affine_scale() {
    let line = Line::through(Point3::new(1.0, 0.0, 0.0), Point3::new(4.0, 4.0, 0.0));
    let moved = line.moved(&motion());

    assert!((moved.length(0.0, 1.0) - line.length(0.0, 1.0)).abs() <= LINEAR_TOLERANCE);
}

#[test]
fn a_moved_circle_keeps_its_radius() {
    let circle = Circle::new(Plane::from_frame(tilted_frame()), 2.0);
    let moved = circle.moved(&motion());

    assert!((moved.radius() - circle.radius()).abs() <= LINEAR_TOLERANCE);
}

#[test]
fn an_axis_moves_origin_and_direction_together() {
    let r = motion();
    let axis = tilted_axis();
    let moved = axis.moved(&r);

    assert_point_near(moved.origin, r.apply(axis.origin));
    assert!(
        (moved.direction.into_inner() - r.apply_vector(axis.direction.into_inner())).norm()
            <= LINEAR_TOLERANCE
    );
}

#[test]
fn axis_constructors_run_through_the_origin() {
    for (axis, direction) in [
        (Axis3::x(), Vector3::x()),
        (Axis3::y(), Vector3::y()),
        (Axis3::z(), Vector3::z()),
    ] {
        assert_point_near(axis.origin, Point3::origin());
        assert_eq!(axis.direction.into_inner(), direction);
    }
}

/// A rigid motion of an oriented box is the same box in a moved frame, so no
/// extent may change and no refit may happen.
#[test]
fn a_moved_bbox_keeps_its_extents_and_moves_its_centre() {
    let r = motion();
    let bounds = BBox::from_points_in_frame(
        Frame::xyz(),
        [Point3::new(-1.0, -2.0, -3.0), Point3::new(1.0, 2.0, 3.0)],
    );
    let moved = bounds.moved(&r);

    assert_eq!(moved.size(), bounds.size());
    assert_point_near(
        moved.center().expect("a non-empty box has a centre"),
        r.apply(bounds.center().expect("a non-empty box has a centre")),
    );
    assert!(BBox::empty().moved(&r).is_empty());
}

/// Delegating the rotation to nalgebra is only correct if the sense agrees
/// with the rest of the crate, which builds rotations with `Rotation3`.
#[test]
fn rigid_rotation_agrees_with_the_matrix_it_replaces() {
    let axis = tilted_axis();
    let angle = Rad64::new(FRAC_PI_3);
    let matrix = Rotation3::from_axis_angle(&axis.direction, FRAC_PI_3);
    let r = Rigid::rotation(axis, angle);

    for point in [Point3::origin(), Point3::new(3.0, -1.0, 4.0)] {
        assert_point_near(r.apply(point), axis.origin + matrix * (point - axis.origin));
    }
}
