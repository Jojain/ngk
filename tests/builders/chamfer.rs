use std::collections::HashSet;

use nalgebra::Vector3;
use ngk::builders::chamfer::chamfer;
use ngk::builders::edges::add_edge;
use ngk::builders::errors::ChamferError;
use ngk::builders::faces::add_face;
use ngk::builders::profiles::{add_polyline, append_edge};
use ngk::builders::solids::add_extruded_face;
use ngk::geometry::{Curve, NurbsCurve, Point3, Surface};
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
    let corner = g.profile_unchecked(profile).edges()[0].end().key();
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
    let corner = g.profile_unchecked(profile).edges()[0].end().key();

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

#[test]
fn solid_face_profile_chamfer_replaces_the_complete_rim_with_a_bevel_ring() {
    let mut shape = block(2.0, 3.0, 4.0).expect("block should build");
    let solid = shape.key();
    let top_profile = shape
        .solid()
        .faces()
        .into_iter()
        .find(|face| {
            face.vertices().iter().all(|vertex| {
                vertex
                    .point()
                    .is_some_and(|point| (point.z - 4.0).abs() < 1.0e-9)
            })
        })
        .expect("block should have a top face")
        .outer_loop()
        .key();

    chamfer(shape.map_mut(), top_profile, 0.25)
        .expect("complete top profile should chamfer as one solid operation");

    assert_eq!(shape.solid().faces().len(), 10);
    assert_eq!(shape.solid().edges().len(), 20);
    assert_eq!(shape.solid().vertices().len(), 12);
    validate_solid_manifold(shape.map(), solid)
        .expect("profile-chamfered block should remain manifold");
    validate_solid_orientation(shape.map(), solid)
        .expect("profile-chamfered block faces should remain outward");
}

#[test]
fn solid_edge_chamfer_supports_an_extruded_nurbs_profile_edge() {
    let mut g = GMap::<StandardPayload>::new();
    let profile = add_polyline(
        &mut g,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 0.0),
            Point3::new(4.0, 3.0, 0.0),
        ],
    )
    .expect("open profile should build");
    let wavy_edge = add_edge(
        &mut g,
        Point3::new(4.0, 3.0, 0.0),
        Point3::new(0.0, 3.0, 0.0),
        Curve::Nurbs(
            NurbsCurve::interpolate(&[
                Point3::new(4.0, 3.0, 0.0),
                Point3::new(3.0, 2.65, 0.0),
                Point3::new(2.0, 3.35, 0.0),
                Point3::new(1.0, 2.65, 0.0),
                Point3::new(0.0, 3.0, 0.0),
            ])
            .expect("wave samples should interpolate"),
        ),
    )
    .expect("wavy edge should build");
    append_edge(&mut g, profile, wavy_edge).expect("wavy edge should append");
    let closing_edge = add_edge(
        &mut g,
        Point3::new(0.0, 3.0, 0.0),
        Point3::new(0.0, 0.0, 0.0),
        Curve::line(Point3::new(0.0, 3.0, 0.0), Point3::new(0.0, 0.0, 0.0)),
    )
    .expect("closing edge should build");
    append_edge(&mut g, profile, closing_edge).expect("profile should close");
    let face = add_face(&mut g, profile).expect("wavy planar face should build");
    let solid = add_extruded_face(&mut g, face, Vector3::new(0.0, 0.0, 2.0))
        .expect("wavy face should extrude");
    let top_wavy_edge = g
        .solid_unchecked(solid)
        .edges()
        .into_iter()
        .find(|edge| {
            matches!(edge.curve(), Some(Curve::Nurbs(_)))
                && edge
                    .start()
                    .point()
                    .is_some_and(|point| (point.z - 2.0).abs() < 1.0e-9)
        })
        .expect("extrusion should contain a translated wavy edge")
        .key();
    let original_faces = g.iter_faces().map(|(key, _)| key).collect::<HashSet<_>>();

    chamfer(&mut g, top_wavy_edge, 0.2).expect("wavy solid edge should chamfer");

    assert_eq!(g.solid_unchecked(solid).faces().len(), 7);
    let chamfer_face = g
        .solid_unchecked(solid)
        .faces()
        .into_iter()
        .find(|face| {
            !original_faces.contains(&face.key())
                && matches!(
                    face.surface(),
                    Surface::Ruled(surface) if surface.direction().y.abs() > 1.0e-9
                )
        })
        .expect("chamfer should insert a new face");
    assert!(
        matches!(chamfer_face.surface(), Surface::Ruled(_)),
        "the curved chamfer should have a ruled support surface"
    );
    assert_eq!(
        chamfer_face
            .edges()
            .iter()
            .filter(|edge| matches!(edge.curve(), Some(Curve::Nurbs(_))))
            .count(),
        2,
        "the chamfer should retain both curved NURBS boundaries"
    );
    validate_solid_manifold(&g, solid).expect("curved chamfer should remain manifold");
    validate_solid_orientation(&g, solid).expect("curved chamfer faces should remain outward");
}
