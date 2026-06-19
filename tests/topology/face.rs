use ngk::builders::faces::{add_annulus, add_circle};
use ngk::geometry::{LINEAR_TOLERANCE, Plane, Point3, PointCoincidence};
use ngk::modeling::faces;
use ngk::topology::Orientation;
use ngk::topology::gmap::{Dim, GMap};
use ngk::topology::payload::StandardPayload;

#[test]
fn face_point_at_evaluates_its_support_surface() {
    let shape = faces::rectangle(Plane::xy(), 2.0, 3.0).expect("face should build");
    let point = shape.face().point_at(0.5, 1.25);

    assert!(point.coincides(&Point3::new(0.5, 1.25, 0.0), LINEAR_TOLERANCE));
}

#[test]
fn face_point_at_is_defined_inside_a_trimmed_hole() {
    let shape = faces::annulus(Plane::xy(), 2.0, 1.0).expect("face should build");
    let point = shape.face().point_at(0.0, 0.0);

    assert!(point.coincides(&Point3::origin(), LINEAR_TOLERANCE));
}

#[test]
fn face_darts_resolve_orientation_relative_to_stored_loop_seed() {
    let mut g = GMap::<StandardPayload>::new();
    let face_key = add_circle(&mut g, Plane::xy(), 1.0).expect("circle face should build");
    let default_dart = g
        .face_attr(face_key)
        .expect("face should be registered")
        .outer_loop;
    let reversed_dart = g.alpha(Dim::Zero, default_dart);

    assert_eq!(
        g.face_orientation_at_dart(face_key, default_dart),
        Orientation::Same
    );
    assert_eq!(
        g.face_orientation_at_dart(face_key, reversed_dart),
        Orientation::Reversed
    );

    let default_face =
        ngk::topology::face::Face::from_dart(&g, default_dart).expect("face should resolve");
    let reversed_face =
        ngk::topology::face::Face::from_dart(&g, reversed_dart).expect("face should resolve");

    assert_eq!(default_face.key(), face_key);
    assert_eq!(default_face.orientation, Orientation::Same);
    assert_eq!(reversed_face.key(), face_key);
    assert_eq!(reversed_face.orientation, Orientation::Reversed);
}

#[test]
fn every_stored_face_loop_seed_has_the_default_orientation() {
    let mut g = GMap::<StandardPayload>::new();
    let face_key = add_annulus(&mut g, Plane::xy(), 2.0, 1.0).expect("annulus face should build");
    let face = g.face_attr(face_key).expect("face should be registered");

    assert_eq!(
        g.face_orientation_at_dart(face_key, face.outer_loop),
        Orientation::Same
    );
    assert_eq!(
        g.face_orientation_at_dart(face_key, face.inner_loops[0]),
        Orientation::Same
    );
    assert_eq!(
        g.face_orientation_at_dart(face_key, g.alpha(Dim::Zero, face.inner_loops[0])),
        Orientation::Reversed
    );
}
