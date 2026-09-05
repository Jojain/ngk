use nalgebra::Vector3;
use ngk::builders::boolean::{BooleanOperand, BooleanOptions, compute_boolean_intersections};
use ngk::builders::faces::{add_face, add_rectangle};
use ngk::builders::profiles::add_polyline;
use ngk::geometry::{Plane, Point3};
use ngk::topology::gmap::GMap;

/// A U-shaped face in the z = 0 plane, opening towards +y.
fn u_shaped_face(map: &mut GMap<ngk::StandardPayload>) -> ngk::topology::shape_keys::FaceKey {
    let points = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(3.0, 0.0, 0.0),
        Point3::new(3.0, 3.0, 0.0),
        Point3::new(2.0, 3.0, 0.0),
        Point3::new(2.0, 1.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(1.0, 3.0, 0.0),
        Point3::new(0.0, 3.0, 0.0),
        Point3::new(0.0, 0.0, 0.0),
    ];
    let profile = add_polyline(map, &points).expect("u profile");
    add_face(map, profile).expect("u face")
}

#[test]
fn a_section_leaving_and_re_entering_a_face_is_clipped_into_two_spans() {
    let mut map = GMap::<ngk::StandardPayload>::new();
    let first = u_shaped_face(&mut map);
    // A vertical face crossing the U at y = 2, where the notch splits the material.
    let second = add_rectangle(
        &mut map,
        Plane::from_xy(Point3::new(-1.0, 2.0, -1.0), Vector3::x(), Vector3::z()),
        5.0,
        2.0,
    )
    .expect("tool face");

    let plan = compute_boolean_intersections(
        &map,
        BooleanOperand::Face(first),
        BooleanOperand::Face(second),
        BooleanOptions::default(),
    )
    .expect("crossing faces must produce a network");

    assert_eq!(
        plan.network.spans().len(),
        2,
        "the notch must not be bridged by a single span"
    );
    for span in plan.network.spans() {
        let start = span.curve.point_at(0.0);
        let end = span.curve.point_at(1.0);
        let inside_notch = |point: Point3| point.x > 1.0 + 1.0e-9 && point.x < 2.0 - 1.0e-9;
        assert!(
            !inside_notch(span.curve.point_at(0.5)),
            "a retained span must stay inside the face, got {:?}",
            span.curve.point_at(0.5)
        );
        assert!((start.y - 2.0).abs() < 1.0e-9 && (end.y - 2.0).abs() < 1.0e-9);
    }
}
