use ngk::builders::faces::add_rectangle;
use ngk::geometry::{Plane, Surface};
use ngk::topology::gmap::GMap;
use ngk::topology::payload::StandardPayload;

#[test]
fn add_rectangle_creates_single_planar_face_with_pcurves() {
    let mut g = GMap::<StandardPayload>::new();
    let face_key = add_rectangle(&mut g, Plane::xy(), 2.0, 3.0).expect("face should build");
    let face = g.face(face_key).expect("face key should be registered");

    assert_eq!(g.iter_faces().count(), 1);
    assert!(matches!(face.surface, Surface::Plane(_)));
    assert_eq!(face.pcurves.len(), 4);
}
