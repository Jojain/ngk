use ngk::builders::chamfer::chamfer;
use ngk::builders::errors::ChamferError;
use ngk::builders::profiles::add_polyline;
use ngk::geometry::Point3;
use ngk::modeling::solids::block;
use ngk::topology::StandardPayload;
use ngk::topology::gmap::GMap;
use ngk::topology::validation::{validate_solid_manifold, validate_solid_orientation};

#[test]
fn failed_chamfer_builder_preserves_the_source_profile() {
    let mut g = GMap::<StandardPayload>::new();
    let profile = add_polyline(
        &mut g,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ],
    )
    .expect("profile should build");
    let corner = g.profile_unchecked(profile).edges()[0].end().dart;
    let before_darts = g.dart_count();
    let before_edges = g.iter_edges().count();
    let before_vertices = g.iter_vertices().count();

    let result = chamfer(&mut g, corner, -1.0);

    assert!(
        matches!(result, Err(ChamferError::InvalidDistance { .. })),
        "unexpected result: {result:?}"
    );
    assert_eq!(g.dart_count(), before_darts);
    assert_eq!(g.iter_edges().count(), before_edges);
    assert_eq!(g.iter_vertices().count(), before_vertices);
}

#[test]
fn profile_chamfer_mutates_in_place_without_returning_a_topology_handle() {
    let mut g = GMap::<StandardPayload>::new();
    let profile = add_polyline(
        &mut g,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ],
    )
    .expect("profile should build");
    let corner = g.profile_unchecked(profile).edges()[0].end().dart;

    let result: Result<(), ChamferError> = chamfer(&mut g, corner, 0.25);

    result.expect("profile corner should chamfer");
    assert_eq!(g.iter_edges().count(), 3);
    assert_eq!(g.iter_vertices().count(), 4);
}

#[test]
fn solid_edge_chamfer_replaces_a_block_edge_with_a_planar_face() {
    let mut shape = block(2.0, 3.0, 4.0).expect("block should build");
    let solid = shape.key();
    let edge = shape
        .solid()
        .edges()
        .into_iter()
        .find(|edge| {
            let start = *edge
                .start()
                .point()
                .expect("edge start should be geometric");
            let end = *edge.end().point().expect("edge end should be geometric");
            (start.x - end.x).abs() < 1.0e-9
                && (start.y - end.y).abs() < 1.0e-9
                && (start.z - end.z).abs() > 3.9
        })
        .expect("block should have a vertical edge")
        .key();

    let result: Result<(), ChamferError> = chamfer(shape.map_mut(), edge, 0.25);

    result.expect("straight block edge should chamfer");
    assert_eq!(shape.solid().faces().len(), 7);
    assert_eq!(shape.solid().edges().len(), 15);
    assert_eq!(shape.solid().vertices().len(), 10);
    validate_solid_manifold(shape.map(), solid).expect("chamfered block should remain manifold");
    validate_solid_orientation(shape.map(), solid)
        .expect("chamfered block faces should remain outward");
}

#[test]
fn solid_vertex_chamfer_replaces_a_block_corner_with_a_planar_face() {
    let mut shape = block(2.0, 3.0, 4.0).expect("block should build");
    let solid = shape.key();
    let vertex = shape
        .solid()
        .vertices()
        .into_iter()
        .find(|vertex| {
            let point = *vertex.point().expect("block vertex should be geometric");
            point.coords.norm() < 1.0e-9
        })
        .expect("block should have an origin vertex")
        .key();

    let result: Result<(), ChamferError> = chamfer(shape.map_mut(), vertex, 0.25);

    result.expect("trihedral block vertex should chamfer");
    assert_eq!(shape.solid().faces().len(), 7);
    assert_eq!(shape.solid().edges().len(), 15);
    assert_eq!(shape.solid().vertices().len(), 10);
    validate_solid_manifold(shape.map(), solid).expect("chamfered block should remain manifold");
    validate_solid_orientation(shape.map(), solid)
        .expect("chamfered block faces should remain outward");
}

#[test]
fn profile_target_chamfers_every_corner_in_place() {
    let mut g = GMap::<StandardPayload>::new();
    let profile =
        ngk::builders::profiles::add_rectangle(&mut g, ngk::geometry::Plane::xy(), 2.0, 1.0)
            .expect("rectangle should build");

    chamfer(&mut g, profile, 0.1).expect("closed line profile should chamfer");

    assert_eq!(g.iter_edges().count(), 8);
    assert_eq!(g.iter_vertices().count(), 8);
}

#[test]
fn several_disjoint_solid_edges_can_be_chamfered_in_one_transaction() {
    let mut shape = block(2.0, 3.0, 4.0).expect("block should build");
    let solid = shape.key();
    let edges = shape
        .solid()
        .edges()
        .into_iter()
        .filter(|edge| {
            let start = *edge
                .start()
                .point()
                .expect("edge start should be geometric");
            let end = *edge.end().point().expect("edge end should be geometric");
            (start.x - end.x).abs() < 1.0e-9
                && (start.y - end.y).abs() < 1.0e-9
                && (start.z - end.z).abs() > 3.9
                && ((start.x < 1.0e-9 && start.y < 1.0e-9) || (start.x > 1.9 && start.y > 2.9))
        })
        .map(|edge| edge.key())
        .collect::<Vec<_>>();
    assert_eq!(edges.len(), 2);

    chamfer(shape.map_mut(), edges, 0.2).expect("disjoint block edges should chamfer");

    assert_eq!(shape.solid().faces().len(), 8);
    assert_eq!(shape.solid().edges().len(), 18);
    assert_eq!(shape.solid().vertices().len(), 12);
    validate_solid_manifold(shape.map(), solid).expect("multi-chamfer should remain manifold");
    validate_solid_orientation(shape.map(), solid)
        .expect("multi-chamfer faces should remain outward");
}
