use ngk::builders::errors::{FaceCreationError, PolylineError};
use ngk::builders::faces::{add_annulus, add_circle, add_rectangle};
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

#[test]
fn add_rectangle_reports_profile_creation_errors() {
    let mut g = GMap::<StandardPayload>::new();

    let result = add_rectangle(&mut g, Plane::xy(), 0.0, 3.0);

    assert_eq!(
        result,
        Err(FaceCreationError::ProfileCreationFailed(
            PolylineError::InvalidRectangleSize {
                axis: "x",
                value: 0.0
            }
        ))
    );
}

#[test]
fn add_circle_creates_single_planar_face_with_circular_pcurve() {
    let mut g = GMap::<StandardPayload>::new();
    let face_key = add_circle(&mut g, Plane::xy(), 2.0).expect("circle face should build");
    let face = g.face(face_key).expect("face key should be registered");

    assert_eq!(g.iter_faces().count(), 1);
    assert_eq!(g.iter_edges().count(), 1);
    assert!(matches!(face.surface, Surface::Plane(_)));
    assert_eq!(face.inner_loops.len(), 0);
    assert_eq!(face.pcurves.len(), 1);
}

#[test]
fn add_annulus_creates_planar_face_with_inner_circular_loop() {
    let mut g = GMap::<StandardPayload>::new();
    let face_key = add_annulus(&mut g, Plane::xy(), 2.0, 1.0).expect("annulus face should build");
    let face = g.face(face_key).expect("face key should be registered");

    assert_eq!(g.iter_faces().count(), 1);
    assert_eq!(g.iter_edges().count(), 2);
    assert!(matches!(face.surface, Surface::Plane(_)));
    assert_eq!(face.inner_loops.len(), 1);
    assert_eq!(face.pcurves.len(), 2);
}
