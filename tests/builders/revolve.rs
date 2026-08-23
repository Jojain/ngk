use std::collections::HashSet;

use nalgebra::Vector3;
use ngk::viz::debug_viewer::show;
use radians::Rad64;

use ngk::builders::edges::add_edge;
use ngk::builders::faces::{add_face, add_polygon};
use ngk::builders::revolve::{add_revolved_edge, add_revolved_face};
use ngk::geometry::axis::Axis3;
use ngk::geometry::{Curve, LINEAR_TOLERANCE, Point3, PointCoincidence, Surface};
use ngk::tessellate::{TessellateOpts, tessellate_face_key};
use ngk::topology::gmap::GMap;
use ngk::topology::payload::StandardPayload;

#[test]
fn revolve_edge_partial_turn_creates_four_edge_face() {
    let mut g = GMap::<StandardPayload>::new();
    let edge_key = add_edge(
        &mut g,
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
        Curve::line(Point3::new(1.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)),
    )
    .expect("edge should build");

    let face_key = add_revolved_edge(
        &mut g,
        edge_key,
        Axis3::new(Point3::origin(), Vector3::z()),
        Rad64::QUARTER_TURN,
    )
    .unwrap();
    let face = g.face_unchecked(face_key);
    let boundary_edges = face.outer_loop().edges();
    let boundary_vertices = face.outer_loop().vertices();
    let boundary_edge_keys = boundary_edges
        .iter()
        .map(|edge| edge.key())
        .collect::<HashSet<_>>();
    let boundary_vertex_keys = boundary_vertices
        .iter()
        .map(|vertex| vertex.key())
        .collect::<HashSet<_>>();
    show(&g);
    assert_eq!(
        (
            g.iter_faces().count(),
            g.iter_edges().count(),
            g.iter_vertices().count()
        ),
        (1, 4, 4)
    );
    assert_eq!(
        (
            face.loops().len(),
            boundary_edges.len(),
            boundary_vertices.len()
        ),
        (1, 4, 4)
    );
    assert_eq!(
        (boundary_edge_keys.len(), boundary_vertex_keys.len()),
        (4, 4)
    );
    assert_eq!(
        boundary_edges
            .iter()
            .filter(|edge| edge.key() == edge_key)
            .count(),
        1,
        "the source edge should occur once in the boundary"
    );
}

#[test]
fn revolve_edge_partial_turn_uses_quarter_circle_sides() {
    let mut g = GMap::<StandardPayload>::new();
    let edge_key = add_edge(
        &mut g,
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
        Curve::line(Point3::new(1.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)),
    )
    .expect("edge should build");

    let face_key = add_revolved_edge(
        &mut g,
        edge_key,
        Axis3::new(Point3::origin(), Vector3::z()),
        Rad64::QUARTER_TURN,
    )
    .unwrap();

    let side_arc = g
        .face_unchecked(face_key)
        .edges()
        .into_iter()
        .find(|edge| {
            let start = *edge
                .start()
                .point()
                .expect("side arc start should have geometry");
            let end = *edge
                .end()
                .point()
                .expect("side arc end should have geometry");
            let original = Point3::new(1.0, 0.0, 0.0);
            let rotated = Point3::new(0.0, 1.0, 0.0);
            matches!(edge.curve(), Some(Curve::Circle(_)))
                && ((start.coincides(original, LINEAR_TOLERANCE)
                    && end.coincides(rotated, LINEAR_TOLERANCE))
                    || (start.coincides(rotated, LINEAR_TOLERANCE)
                        && end.coincides(original, LINEAR_TOLERANCE)))
        })
        .expect("revolve should create a circular side arc");
    let curve = side_arc.curve().expect("side arc should have geometry");
    let t0 = curve.param_at(Point3::new(1.0, 0.0, 0.0));
    let t1 = curve.param_at(Point3::new(0.0, 1.0, 0.0));
    let midpoint = curve.point_at(0.5 * (t0 + t1));

    assert!(
        midpoint.coincides(
            Point3::new(
                std::f64::consts::FRAC_1_SQRT_2,
                std::f64::consts::FRAC_1_SQRT_2,
                0.0
            ),
            LINEAR_TOLERANCE
        ),
        "side arc midpoint should stay on the same quarter-turn as the revolved face, got {midpoint:?}"
    );
}

#[test]
fn revolve_edge_full_turn_creates_inner_loop() {
    let mut g = GMap::<StandardPayload>::new();
    let edge_key = add_edge(
        &mut g,
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
        Curve::line(Point3::new(1.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)),
    )
    .expect("edge should build");

    let face_key = add_revolved_edge(
        &mut g,
        edge_key,
        Axis3::new(Point3::origin(), Vector3::z()),
        Rad64::FULL_TURN,
    )
    .unwrap();
    let face = g.face_unchecked(face_key);
    let boundary_edges = face.edges();
    let boundary_vertices = face.vertices();
    let boundary_edge_keys = boundary_edges
        .iter()
        .map(|edge| edge.key())
        .collect::<HashSet<_>>();
    let boundary_vertex_keys = boundary_vertices
        .iter()
        .map(|vertex| vertex.key())
        .collect::<HashSet<_>>();
    show(&g);
    assert_eq!(
        (
            g.iter_faces().count(),
            g.iter_edges().count(),
            g.iter_vertices().count()
        ),
        (1, 2, 2)
    );
    assert_eq!(
        (
            face.loops().len(),
            boundary_edges.len(),
            boundary_vertices.len()
        ),
        (2, 2, 2)
    );
    assert_eq!(
        (boundary_edge_keys.len(), boundary_vertex_keys.len()),
        (2, 2)
    );
}

#[test]
fn revolve_edge_full_turn_without_inner_loop() {
    let mut g = GMap::<StandardPayload>::new();
    let edge_key = add_edge(
        &mut g,
        Point3::origin(),
        Point3::new(2.0, 0.0, 0.0),
        Curve::line(Point3::origin(), Point3::new(2.0, 0.0, 0.0)),
    )
    .expect("edge should build");

    let face_key = add_revolved_edge(
        &mut g,
        edge_key,
        Axis3::new(Point3::origin(), Vector3::z()),
        Rad64::FULL_TURN,
    )
    .unwrap();
    let face = g.face_unchecked(face_key);
    let boundary_edges = face.edges();
    let boundary_vertices = face.vertices();
    let boundary_edge_keys = boundary_edges
        .iter()
        .map(|edge| edge.key())
        .collect::<HashSet<_>>();
    let boundary_vertex_keys = boundary_vertices
        .iter()
        .map(|vertex| vertex.key())
        .collect::<HashSet<_>>();

    assert_eq!(
        (
            g.iter_faces().count(),
            g.iter_edges().count(),
            g.iter_vertices().count()
        ),
        (1, 1, 1)
    );
    assert_eq!(
        (
            face.loops().len(),
            boundary_edges.len(),
            boundary_vertices.len()
        ),
        (1, 1, 1)
    );
    assert_eq!(
        (boundary_edge_keys.len(), boundary_vertex_keys.len()),
        (1, 1)
    );
}

#[test]
fn revolved_face_adds_surface_of_revolution_faces() {
    let mut g = GMap::<StandardPayload>::new();
    let profile_key = add_polygon(
        &mut g,
        &[
            Point3::new(0.75, 0.0, -0.85),
            Point3::new(1.85, 0.0, -0.05),
            Point3::new(0.85, 0.0, 0.9),
        ],
    );
    let source_face = add_face(&mut g, profile_key).unwrap();

    add_revolved_face(
        &mut g,
        source_face,
        Axis3::new(Point3::origin(), Vector3::z()),
        Rad64::QUARTER_TURN,
    )
    .unwrap();

    let revolved_faces = g
        .iter_faces()
        .filter(|(_, attr)| matches!(attr.surface, Surface::Revolution(_)))
        .collect::<Vec<_>>();

    assert_eq!(revolved_faces.len(), 3);
    for (face_key, attr) in revolved_faces {
        assert_eq!(attr.pcurves.len(), 4);
        let mesh = tessellate_face_key(&g, face_key, TessellateOpts::default())
            .expect("revolved face should tessellate from its pcurves");
        assert!(!mesh.is_empty());
    }
    assert_eq!(g.iter_solids().count(), 1);
}
