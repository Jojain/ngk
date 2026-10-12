use std::f64::consts::PI;

use nalgebra::Vector3;
use ngk::builders::edges::add_edge;
use ngk::builders::faces::{add_face, add_polygon};
use ngk::builders::solids::{add_extruded_face, translate_face};
use ngk::geometry::{Curve, Curve2, LINEAR_TOLERANCE, Plane, Point3, Surface, TrimmedCurve2};
use ngk::model::Model;
use ngk::modeling::faces;
use ngk::topology::ModelEditError;
use ngk::topology::payload::StandardPayload;
use ngk::topology::shape::{FaceTag, Shape};
use ngk::topology::validation::{validate_gmap, validate_solid_orientation};
use ngk::viz::debug_viewer::show;

#[test]
fn translate_face_copies_face_into_translated_map() {
    let mut source = Model::<StandardPayload>::new();
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

    assert_eq!(translated.model().dart_count(), 8);
    assert_eq!(translated.model().iter_faces().count(), 1);
    assert!(
        translated
            .model()
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
fn extrusion_result_views_are_revision_checked() {
    let mut model = Model::<StandardPayload>::new();
    let profile = add_polygon(
        &mut model,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ],
    );
    let face = add_face(&mut model, profile).expect("face should build");
    let result = add_extruded_face(&mut model, face, Vector3::z()).expect("face should extrude");

    let view = result.view(&model).expect("a fresh result should resolve");
    assert_eq!(view.laterals.len(), 4);
    assert_eq!(view.start_cap.key(), result.start_cap);
    assert_eq!(view.end_cap.key(), result.end_cap);

    add_edge(
        &mut model,
        Point3::new(4.0, 0.0, 0.0),
        Point3::new(5.0, 0.0, 0.0),
        Curve::line(Point3::new(4.0, 0.0, 0.0), Point3::new(5.0, 0.0, 0.0)),
    )
    .expect("independent edit should commit");
    assert!(result.view(&model).is_err());
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
    let (mut g, face) = profile.into_model();

    let solid_key = add_extruded_face(&mut g, face, Vector3::new(0.0, 0.0, 3.0))
        .expect("holed face should extrude")
        .solid;

    validate_gmap(g.topology()).expect("extrusion should produce a valid map");
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

#[test]
fn an_extruded_shell_faces_outward_whichever_way_the_face_was_wound() {
    // Which cap has to be turned depends on the sign, and so does whether the
    // laterals come out facing in. A face wound against the run used to
    // extrude an inside-out shell: the caps were corrected and the laterals
    // were left alone.
    for winding in [
        [
            Point3::new(-1.0, -1.0, 0.0),
            Point3::new(1.0, -1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ],
        [
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(1.0, -1.0, 0.0),
            Point3::new(-1.0, -1.0, 0.0),
        ],
    ] {
        let mut model = Model::<StandardPayload>::new();
        let profile = add_polygon(&mut model, &winding);
        let face = add_face(&mut model, profile).expect("face should build");
        let extrusion = add_extruded_face(&mut model, face, Vector3::new(0.0, 0.0, 3.0))
            .expect("extrusion should build");

        validate_gmap(model.topology()).expect("gmap should stay valid");
        validate_solid_orientation(&model, extrusion.solid)
            .expect("every face of the shell should point out of the material");
    }
}

/// Rewrites every curve bounding `face` as the NURBS that represents it.
///
/// A closed circle becomes a closed NURBS edge with no vertex, the boundary a
/// face Boolean or a STEP import can hand over.
fn with_nurbs_boundary(face: Shape<FaceTag>) -> Shape<FaceTag> {
    let (mut model, key) = face.into_model();
    let edges = model
        .iter_edges()
        .map(|(edge, attr)| {
            (
                edge,
                Curve::Nurbs(attr.curve.to_nurbs().expect("edge nurbs")),
            )
        })
        .collect::<Vec<_>>();
    let pcurves = model
        .face_attr_unchecked(key)
        .pcurves
        .iter()
        .map(|(&dart, pcurve)| {
            let nurbs = pcurve.to_nurbs().expect("pcurve nurbs");
            let domain = nurbs.domain();
            (dart, TrimmedCurve2::new(Curve2::Nurbs(nurbs), domain))
        })
        .collect::<Vec<_>>();
    model
        .transaction(|edit| {
            for (edge, curve) in edges {
                edit.edge_attr_mut_unchecked(edge).curve = curve;
            }
            for (dart, pcurve) in pcurves {
                edit.face_attr_mut_unchecked(key)
                    .pcurves
                    .insert(dart, pcurve);
            }
            Ok::<_, ModelEditError>(())
        })
        .expect("rewrite the boundary as NURBS");
    Shape::new(model, key)
}

fn extrudes_to(face: Shape<FaceTag>, height: f64, expected: f64) {
    let (mut model, key) = face.into_model();
    let extrusion = add_extruded_face(&mut model, key, Vector3::new(0.0, 0.0, height))
        .expect("a closed NURBS loop with no vertex extrudes");
    validate_gmap(model.topology()).expect("gmap should stay valid");
    validate_solid_orientation(&model, extrusion.solid).expect("the shell faces outward");
    let volume = model
        .solid_unchecked(extrusion.solid)
        .volume()
        .expect("volume");
    assert!(
        (volume - expected).abs() < 5e-3 * expected,
        "expected a volume of {expected}, got {volume}"
    );
}

#[test]
fn a_disk_bounded_by_a_closed_nurbs_edge_extrudes() {
    let disk = with_nurbs_boundary(faces::circle(Plane::xy(), 5.0).expect("disk"));
    extrudes_to(disk, 8.0, 25.0 * PI * 8.0);
}

#[test]
fn a_hole_bounded_by_a_closed_nurbs_edge_extrudes() {
    let rectangle = faces::rectangle(
        Plane::new(Point3::new(-20.0, -15.0, 0.0), Vector3::x(), Vector3::z()),
        40.0,
        30.0,
    )
    .expect("rectangle");
    let hole = faces::circle(Plane::xy(), 5.0).expect("hole");
    let holed = faces::cut(rectangle, [&hole]).expect("cut");
    let (model, keys) = holed.into_model();
    let holed = with_nurbs_boundary(Shape::new(model, keys[0]));
    extrudes_to(holed, 8.0, (1200.0 - 25.0 * PI) * 8.0);
}
