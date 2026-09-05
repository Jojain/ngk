use nalgebra::Vector3;
use ngk::builders::faces::{add_face, add_polygon};
use ngk::builders::solids::{add_extruded_face, translate_face};
use ngk::geometry::{LINEAR_TOLERANCE, Plane, Point3, Surface};
use ngk::modeling::faces;
use ngk::topology::gmap::GMap;
use ngk::topology::payload::StandardPayload;
use ngk::topology::validation::{validate_gmap, validate_solid_orientation};

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

#[test]
fn extruded_face_with_a_hole_forms_one_complete_closed_shell() {
    let outer = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(3.0, 0.0, 0.0),
        Point3::new(3.0, 3.0, 0.0),
        Point3::new(0.0, 3.0, 0.0),
    ];
    let hole = [
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(1.0, 2.0, 0.0),
        Point3::new(2.0, 2.0, 0.0),
        Point3::new(2.0, 1.0, 0.0),
    ];
    let profile =
        faces::polygon_with_holes(Plane::xy(), &outer, &[&hole]).expect("holed face should build");
    let (mut g, face) = profile.into_map();

    let solid_key = add_extruded_face(&mut g, face, Vector3::new(0.0, 0.0, 3.0))
        .expect("holed face should extrude");

    validate_gmap(&g).expect("extrusion should produce a valid map");
    validate_solid_orientation(&g, solid_key)
        .expect("extrusion should produce an outward-oriented solid");
    let solid = g.solid_unchecked(solid_key);
    let shell = solid.outer_shell();
    assert_eq!(solid.shells().len(), 1);
    assert_eq!(solid.faces().len(), 10);
    assert_eq!(shell.faces().len(), 10);
    assert_eq!(
        solid
            .faces()
            .iter()
            .filter(|face| !face.inner_loops().is_empty())
            .count(),
        2
    );
    assert_eq!(
        shell.vertices().len() as isize - shell.edges().len() as isize
            + shell.faces().len() as isize,
        2
    );
    for face in solid.faces() {
        assert_eq!(g.sheet_key(face.dart()), Some(shell.key()));
        assert_eq!(g.solid_key(face.dart()), Some(solid_key));
    }
}
