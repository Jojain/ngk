use nalgebra::Vector3;
use ngk::builders::faces::{add_face, add_polygon};
use ngk::builders::solids::translate_face;
use ngk::geometry::{LINEAR_TOLERANCE, Point3, Surface};
use ngk::topology::gmap::GMap;
use ngk::topology::payload::StandardPayload;

#[test]
fn translate_face_copies_face_into_translated_map() {
    let mut source = GMap::<StandardPayload>::new();
    let profile_key = add_polygon(
        &mut source,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ],
    );
    let face_key = add_face(&mut source, profile_key).expect("face should build");
    let face = source.face_unchecked(face_key);

    let translated = translate_face(&face, Vector3::new(0.0, 0.0, 2.0)).unwrap();

    assert_eq!(translated.map().dart_count(), 8);
    assert_eq!(translated.map().iter_faces().count(), 1);
    assert!(
        translated
            .map()
            .iter_vertices()
            .all(|(_, attr)| (attr.point.z - 2.0).abs() <= LINEAR_TOLERANCE)
    );
    assert!(
        source
            .iter_vertices()
            .all(|(_, attr)| attr.point.z.abs() <= LINEAR_TOLERANCE)
    );

    match translated.face().surface() {
        Surface::Plane(plane) => {
            assert!((plane.origin().z - 2.0).abs() <= LINEAR_TOLERANCE);
        }
        _ => panic!("test face should remain planar"),
    }
}
