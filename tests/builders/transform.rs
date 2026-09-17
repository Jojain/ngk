use std::collections::HashMap;

use nalgebra::Vector3;
use ngk::builders::transform::rigid;
use ngk::geometry::axis::Axis3;
use ngk::geometry::{LINEAR_TOLERANCE, Point3, PointCoincidence, Rigid, Surface, TrimmedCurve2};
use ngk::model::Model;
use ngk::modeling::solids::{block, cylinder};
use ngk::topology::attributes::LoopKind;
use ngk::topology::gmap::Dart;
use ngk::topology::payload::StandardPayload;
use ngk::topology::shape_keys::{EdgeKey, FaceKey, VertexKey};
use ngk::topology::validation::{validate_gmap, validate_solid_orientation};
use radians::Rad64;

/// A motion with both a rotation and a translation, about an axis that is
/// neither a model axis nor through the origin.
fn motion() -> Rigid {
    Rigid::rotation(
        Axis3::new(Point3::new(-1.0, 0.5, 2.0), Vector3::new(1.0, -2.0, 3.0)),
        Rad64::new(0.63),
    )
    .compose(Rigid::translation(Vector3::new(-2.0, 5.0, 1.5)))
}

/// Every stored pcurve, keyed the way the model keys them.
fn pcurves(model: &Model<StandardPayload>) -> Vec<(FaceKey, HashMap<Dart, TrimmedCurve2>)> {
    let mut stored = model
        .iter_faces()
        .map(|(key, attr)| (key, attr.pcurves.clone()))
        .collect::<Vec<_>>();
    stored.sort_by_key(|(key, _)| *key);
    stored
}

/// One face's boundary loops, each as its walked darts and its role.
type FaceLoops = (FaceKey, Vec<(Vec<Dart>, LoopKind)>);

/// Every face's loops, as the dart order and role a traversal reads.
fn loops(model: &Model<StandardPayload>) -> Vec<FaceLoops> {
    let mut stored = model
        .iter_faces()
        .map(|(key, attr)| {
            let face = attr.face(model);
            (
                key,
                face.loops()
                    .iter()
                    .map(|boundary| (boundary.darts().collect::<Vec<_>>(), boundary.kind()))
                    .collect::<Vec<_>>(),
            )
        })
        .collect::<Vec<_>>();
    stored.sort_by_key(|(key, _)| *key);
    stored
}

fn vertex_keys(model: &Model<StandardPayload>) -> Vec<VertexKey> {
    let mut keys = model
        .iter_vertices()
        .map(|(key, _)| key)
        .collect::<Vec<_>>();
    keys.sort();
    keys
}

fn edge_keys(model: &Model<StandardPayload>) -> Vec<EdgeKey> {
    let mut keys = model.iter_edges().map(|(key, _)| key).collect::<Vec<_>>();
    keys.sort();
    keys
}

fn face_keys(model: &Model<StandardPayload>) -> Vec<FaceKey> {
    let mut keys = model.iter_faces().map(|(key, _)| key).collect::<Vec<_>>();
    keys.sort();
    keys
}

#[test]
fn rigid_moves_every_vertex_point() {
    let r = motion();
    let shape = block(1.0, 2.0, 3.0).expect("block primitive should build");
    let before = shape
        .model()
        .iter_vertices()
        .map(|(key, attr)| (key, attr.point))
        .collect::<HashMap<_, _>>();

    let (mut model, _) = shape.into_model();
    rigid(&mut model, &r);

    for (key, attr) in model.iter_vertices() {
        let expected = r.apply(before[&key]);
        assert!(
            attr.point.coincides(expected, LINEAR_TOLERANCE),
            "vertex {key:?} expected at {expected:?}, found at {:?}",
            attr.point
        );
    }
}

/// The claim the whole rigid design rests on: a motion that preserves every
/// parameterization cannot move anything stored in a parameter space. Not
/// "close enough" — bit-identical.
#[test]
fn rigid_leaves_every_pcurve_bit_identical() {
    let shape = cylinder(2.0, 5.0).expect("cylinder primitive should build");
    let (mut model, _) = shape.into_model();
    let before = pcurves(&model);

    rigid(&mut model, &motion());

    assert!(
        before.iter().any(|(_, curves)| !curves.is_empty()),
        "the fixture must store pcurves for this to be testing anything"
    );
    assert!(
        pcurves(&model) == before,
        "a rigid motion must leave stored parameter-space data untouched"
    );
}

#[test]
fn rigid_leaves_every_loop_definition_untouched() {
    let shape = cylinder(2.0, 5.0).expect("cylinder primitive should build");
    let (mut model, _) = shape.into_model();
    let before = loops(&model);

    rigid(&mut model, &motion());

    assert_eq!(
        loops(&model),
        before,
        "a rigid motion must not re-seed or re-role a boundary loop"
    );
}

#[test]
fn rigid_moves_no_key_and_no_dart() {
    let shape = block(1.0, 2.0, 3.0).expect("block primitive should build");
    let (mut model, _) = shape.into_model();
    let darts = model.dart_count();
    let (vertices, edges, faces) = (vertex_keys(&model), edge_keys(&model), face_keys(&model));

    rigid(&mut model, &motion());

    assert_eq!(model.dart_count(), darts);
    assert_eq!(vertex_keys(&model), vertices);
    assert_eq!(edge_keys(&model), edges);
    assert_eq!(face_keys(&model), faces);
}

#[test]
fn rigid_keeps_every_analytic_support_as_itself() {
    let shape = cylinder(2.0, 5.0).expect("cylinder primitive should build");
    let (mut model, _) = shape.into_model();
    let planes = model
        .iter_faces()
        .filter(|(_, attr)| matches!(attr.surface, Surface::Plane(_)))
        .count();
    let cylinders = model
        .iter_faces()
        .filter(|(_, attr)| matches!(attr.surface, Surface::Cylinder(_)))
        .count();

    rigid(&mut model, &motion());

    assert_eq!(
        model
            .iter_faces()
            .filter(|(_, attr)| matches!(attr.surface, Surface::Plane(_)))
            .count(),
        planes
    );
    assert_eq!(
        model
            .iter_faces()
            .filter(|(_, attr)| matches!(attr.surface, Surface::Cylinder(_)))
            .count(),
        cylinders,
    );
}

#[test]
fn rigid_then_its_inverse_restores_every_point() {
    let r = motion();
    let shape = block(1.0, 2.0, 3.0).expect("block primitive should build");
    let (mut model, _) = shape.into_model();
    let before = model
        .iter_vertices()
        .map(|(key, attr)| (key, attr.point))
        .collect::<HashMap<_, _>>();

    rigid(&mut model, &r);
    rigid(&mut model, &r.inverse());

    for (key, attr) in model.iter_vertices() {
        assert!(
            attr.point.coincides(before[&key], LINEAR_TOLERANCE),
            "vertex {key:?} did not come home: {:?} vs {:?}",
            attr.point,
            before[&key]
        );
    }
}

#[test]
fn a_moved_solid_is_still_valid_and_still_oriented_outward() {
    let shape = block(1.0, 2.0, 3.0).expect("block primitive should build");
    let (mut model, solid_key) = shape.into_model();

    rigid(&mut model, &motion());

    validate_gmap(model.topology()).expect("a rigid motion must not disturb the map");
    validate_solid_orientation(&model, solid_key)
        .expect("a rigid motion preserves handedness, so the shell stays outward");
}
