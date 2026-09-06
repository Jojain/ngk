use ngk::builders::faces::split_face_edge;
use ngk::geometry::{Curve, Point3};
use ngk::healing::{HealingOptions, HealingScope, remove_redundant_cells};
use ngk::modeling::solids;
use ngk::tessellate::TessellateOpts;
use ngk::tessellate::face::tessellate_face_key;
use ngk::topology::StandardPayload;
use ngk::topology::gmap::GMap;
use ngk::topology::shape_keys::{EdgeKey, FaceKey};

/// Returns a face of the map together with one of its boundary edges.
fn any_boundary_edge(g: &GMap<StandardPayload>) -> (FaceKey, EdgeKey) {
    let face = g.iter_faces().next().expect("map should have a face").0;
    let edge = g
        .face(face)
        .expect("face should be registered")
        .edges()
        .first()
        .expect("face should have edges")
        .key();
    (face, edge)
}

#[test]
fn splitting_an_edge_then_healing_restores_a_single_edge() {
    let (mut map, _) = solids::block(2.0, 2.0, 2.0).expect("block").into_map();
    let edges = map.iter_edges().count();
    let vertices = map.iter_vertices().count();

    let (face, edge) = any_boundary_edge(&map);
    split_face_edge(&mut map, face, edge, 0.5).expect("splitting a block edge should succeed");
    assert_eq!(map.iter_edges().count(), edges + 1);
    assert_eq!(map.iter_vertices().count(), vertices + 1);

    let report = remove_redundant_cells(&mut map, HealingOptions::default())
        .expect("healing should succeed");

    assert_eq!(
        report.removed_vertices.len(),
        1,
        "exactly the inserted vertex should go, skips were {:?}",
        report.skipped
    );
    assert_eq!(map.iter_edges().count(), edges);
    assert_eq!(map.iter_vertices().count(), vertices);
    assert_eq!(map.iter_faces().count(), 6);
}

#[test]
fn a_fused_edge_spans_its_two_original_endpoints() {
    let (mut map, _) = solids::block(2.0, 3.0, 4.0).expect("block").into_map();
    let (face, edge) = any_boundary_edge(&map);
    let original = map.edge_attr_unchecked(edge).curve.clone();
    let start = original.point_at(0.0);
    let end = original.point_at(1.0);

    split_face_edge(&mut map, face, edge, 0.25).expect("splitting a block edge should succeed");
    remove_redundant_cells(&mut map, HealingOptions::default()).expect("healing should succeed");

    let fused = map
        .iter_edges()
        .map(|(_, attr)| &attr.curve)
        .find(|curve| endpoints_match(curve, start, end))
        .expect("a fused edge spanning the original endpoints should exist");
    assert!(
        (fused.length(0.0, 1.0) - original.length(0.0, 1.0)).abs() <= 1.0e-6,
        "the fused edge should keep the original length"
    );
}

fn endpoints_match(curve: &Curve, start: Point3, end: Point3) -> bool {
    let matches = |a: Point3, b: Point3| (a - b).norm() <= 1.0e-9;
    (matches(curve.point_at(0.0), start) && matches(curve.point_at(1.0), end))
        || (matches(curve.point_at(0.0), end) && matches(curve.point_at(1.0), start))
}

#[test]
fn healing_preserves_shell_euler_characteristic() {
    let (mut map, solid) = solids::block(1.0, 1.0, 1.0).expect("block").into_map();
    let euler = |g: &GMap<StandardPayload>| {
        let shell = &g.solid_unchecked(solid).shells()[0];
        shell.vertices().len() as isize - shell.edges().len() as isize
            + shell.faces().len() as isize
    };
    let before = euler(&map);

    let (face, edge) = any_boundary_edge(&map);
    split_face_edge(&mut map, face, edge, 0.5).expect("splitting a block edge should succeed");
    assert_eq!(euler(&map), before, "splitting must not change the shell");

    remove_redundant_cells(&mut map, HealingOptions::default()).expect("healing should succeed");
    assert_eq!(euler(&map), before, "healing must not change the shell");
}

#[test]
fn a_corner_vertex_between_two_directions_is_preserved() {
    let (mut map, _) = solids::block(2.0, 2.0, 2.0).expect("block").into_map();
    let report = remove_redundant_cells(&mut map, HealingOptions::default())
        .expect("healing should succeed");

    assert!(
        report.is_empty(),
        "a block has no redundant topology, but healing removed {report:?}"
    );
    assert_eq!(map.iter_vertices().count(), 8);
    assert_eq!(map.iter_edges().count(), 12);
    assert_eq!(map.iter_faces().count(), 6);
}

#[test]
fn the_seam_vertex_of_a_closed_edge_is_preserved() {
    let (mut map, _) = solids::cylinder(1.0, 2.0).expect("cylinder").into_map();
    let vertices = map.iter_vertices().count();
    let edges = map.iter_edges().count();

    remove_redundant_cells(&mut map, HealingOptions::default()).expect("healing should succeed");

    assert_eq!(
        map.iter_vertices().count(),
        vertices,
        "removing a seam vertex would leave a vertexless closed edge"
    );
    assert_eq!(map.iter_edges().count(), edges);
}

#[test]
fn a_healed_face_still_tessellates() {
    let (mut map, _) = solids::block(2.0, 2.0, 2.0).expect("block").into_map();
    let (face, edge) = any_boundary_edge(&map);
    split_face_edge(&mut map, face, edge, 0.5).expect("splitting a block edge should succeed");
    remove_redundant_cells(&mut map, HealingOptions::default()).expect("healing should succeed");

    for (key, _) in map.iter_faces() {
        let mesh = tessellate_face_key(&map, key, TessellateOpts::default())
            .expect("every healed face should tessellate");
        assert!(
            !mesh.positions.is_empty(),
            "face {key:?} should emit vertices"
        );
    }
}

#[test]
fn an_empty_scope_heals_nothing() {
    let (mut map, _) = solids::block(2.0, 2.0, 2.0).expect("block").into_map();
    let (face, edge) = any_boundary_edge(&map);
    split_face_edge(&mut map, face, edge, 0.5).expect("splitting a block edge should succeed");
    let edges = map.iter_edges().count();

    let report = remove_redundant_cells(
        &mut map,
        HealingOptions::for_scope(HealingScope::Cells {
            vertices: Vec::new(),
            edges: Vec::new(),
        }),
    )
    .expect("healing should succeed");

    assert!(report.is_empty());
    assert_eq!(map.iter_edges().count(), edges);
}
