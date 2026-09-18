use ngk::builders::faces::{add_circle, split_face_edge};
use ngk::geometry::{Curve, NativeParam, Point3};
use ngk::healing::{HealedCell, HealingOptions, HealingScope, SkipReason, remove_redundant_cells};
use ngk::model::Model;
use ngk::modeling::solids;
use ngk::tessellate::TessellateOpts;
use ngk::tessellate::face::tessellate_face_key;
use ngk::topology::StandardPayload;
use ngk::topology::edge::Edge;
use ngk::topology::shape_keys::{EdgeKey, FaceKey};

/// Returns a face of the map together with one of its boundary edges.
fn any_boundary_edge(g: &Model<StandardPayload>) -> (FaceKey, EdgeKey) {
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
    let (mut map, _) = solids::block(2.0, 2.0, 2.0).expect("block").into_model();
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
    let (mut map, _) = solids::block(2.0, 3.0, 4.0).expect("block").into_model();
    let (face, edge) = any_boundary_edge(&map);
    let original = map.edge_attr_unchecked(edge).curve.clone();
    let start = original.point_at(NativeParam::new(0.0));
    let end = original.point_at(NativeParam::new(1.0));

    split_face_edge(&mut map, face, edge, 0.25).expect("splitting a block edge should succeed");
    remove_redundant_cells(&mut map, HealingOptions::default()).expect("healing should succeed");

    let fused = map
        .iter_edges()
        .map(|(_, attr)| &attr.curve)
        .find(|curve| endpoints_match(curve, start, end))
        .expect("a fused edge spanning the original endpoints should exist");
    assert!(
        (fused.length(NativeParam::new(0.0), NativeParam::new(1.0))
            - original.length(NativeParam::new(0.0), NativeParam::new(1.0)))
        .abs()
            <= 1.0e-6,
        "the fused edge should keep the original length"
    );
}

fn endpoints_match(curve: &Curve, start: Point3, end: Point3) -> bool {
    let matches = |a: Point3, b: Point3| (a - b).norm() <= 1.0e-9;
    (matches(curve.point_at(NativeParam::new(0.0)), start)
        && matches(curve.point_at(NativeParam::new(1.0)), end))
        || (matches(curve.point_at(NativeParam::new(0.0)), end)
            && matches(curve.point_at(NativeParam::new(1.0)), start))
}

#[test]
fn healing_preserves_shell_euler_characteristic() {
    let (mut map, solid) = solids::block(1.0, 1.0, 1.0).expect("block").into_model();
    let euler = |g: &Model<StandardPayload>| {
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
    let (mut map, _) = solids::block(2.0, 2.0, 2.0).expect("block").into_model();
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

/// A marked edge keeps the one corner it has.
///
/// A 0-removal fuses the two edges meeting at a vertex, and a marked edge's
/// corner joins that edge to itself -- there is no pair to fuse, so the pass
/// declines it as `NotBetweenTwoCells`. The corner survives because no operation
/// applies, not because a guard forbids one.
///
/// It takes a cut to reach this shape: a rim built fresh is unmarked, since the
/// place a circle closes is classified inside the edge.
#[test]
fn the_lone_vertex_of_a_closed_edge_is_preserved() {
    let mut map = Model::<StandardPayload>::new();
    let face = add_circle(&mut map, ngk::geometry::Plane::xy(), 1.0).expect("a disc");
    let rim = map
        .face_unchecked(face)
        .edges()
        .first()
        .expect("the disc has a rim")
        .key();
    split_face_edge(&mut map, face, rim, 0.5).expect("the rim takes a corner");
    assert_eq!(map.iter_edges().count(), 1, "marking separates nothing");
    assert_eq!(map.iter_vertices().count(), 1, "and leaves one corner");

    let report = remove_redundant_cells(&mut map, HealingOptions::default())
        .expect("healing should succeed");

    assert!(
        report.skipped.iter().any(|skip| matches!(
            (&skip.cell, &skip.reason),
            (HealedCell::Vertex(_), SkipReason::NotBetweenTwoCells)
        )),
        "the corner joins one edge to itself, so there is no pair to fuse"
    );
    assert_eq!(map.iter_edges().count(), 1);
    assert_eq!(map.iter_vertices().count(), 1, "the corner stays");
}

#[test]
fn a_healed_face_still_tessellates() {
    let (mut map, _) = solids::block(2.0, 2.0, 2.0).expect("block").into_model();
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
    let (mut map, _) = solids::block(2.0, 2.0, 2.0).expect("block").into_model();
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

/// Two arcs that close on each other fuse back into one closed edge.
///
/// This used to be refused as `WouldCloseEdge`: the fused edge's two ends are the
/// same vertex, and an edge whose span was derived from its endpoints could say
/// nothing about which arc it was. A closed edge *is* its support now, so the
/// fusion is expressible and healing performs it — a disc split across its rim
/// comes back with the single circular edge it started with.
///
/// Fusing a boundary must also leave the face it bounds facing the way it did:
/// the loop is rebuilt from a curve fitted through sampled points, and a fit
/// that came back running the other way would reverse the loop and with it the
/// face. The normal is checked either side of the fusion for that.
#[test]
fn two_arcs_that_close_on_each_other_fuse_into_one_closed_edge() {
    let mut map = Model::<StandardPayload>::new();
    let face = add_circle(&mut map, ngk::geometry::Plane::xy(), 1.0).expect("a disc");
    let normal_before = map.face_unchecked(face).normal_at(0.0, 0.0);
    let rim = map
        .face_unchecked(face)
        .edges()
        .first()
        .expect("the disc has a rim")
        .key();
    // Two cuts, because one only marks: a circle needs two corners before it
    // is two arcs.
    split_face_edge(&mut map, face, rim, 0.5).expect("the first cut marks the rim");
    split_face_edge(&mut map, face, rim, 3.0).expect("the second cut separates it");
    assert_eq!(map.iter_edges().count(), 2, "the rim starts split in two");
    assert_eq!(map.iter_vertices().count(), 2);

    remove_redundant_cells(&mut map, HealingOptions::default()).expect("healing should succeed");

    assert_eq!(
        map.face_unchecked(face).normal_at(0.0, 0.0),
        normal_before,
        "the fused rim must run the way the split arcs did"
    );

    assert_eq!(
        map.iter_edges().count(),
        1,
        "the two arcs are one circle and must fuse"
    );
    let (edge, _) = map.iter_edges().next().expect("the fused rim");
    let view = map.edge_unchecked(edge);
    assert!(
        matches!(view, Edge::Marked(_)),
        "the fused rim closes on itself, keeping the corner the fusion left"
    );
    let span = view
        .parameter_interval()
        .expect("a closed edge still spans");
    assert!(
        (span.end - span.start).abs() > std::f64::consts::PI,
        "a closed edge spans its support's whole period, got {span:?}"
    );
    assert_eq!(map.iter_faces().count(), 1, "the disc is still one face");
}
