use std::collections::HashSet;

use ngk::builders::edges::add_line;
use ngk::builders::errors::{FaceCreationError, PolylineError};
use ngk::builders::faces::{
    FaceEdgeSplitError, FaceImprint, FaceImprintGraph, add_annulus, add_circle, add_face,
    add_polygon, add_rectangle, split_face_by_imprints, split_face_edge,
};
use ngk::geometry::{
    Curve2, LINEAR_TOLERANCE, Line2, Plane, Point2, Point3, PointCoincidence, Polyline2, Surface,
};
use ngk::modeling::solids::block;
use ngk::topology::gmap::GMap;
use ngk::topology::gmap::{Cell0, Cell2, Dim};
use ngk::topology::payload::StandardPayload;
use ngk::topology::profile::Profile;
use ngk::topology::shape::{FaceTag, FacetTag, Shape};
use ngk::topology::shape_keys::{EdgeKey, FaceKey};
use ngk::viz::ocp_vscode::show;

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

#[test]
fn split_face_edge_updates_boundary_and_pcurves() {
    let mut g = GMap::<StandardPayload>::new();
    let face_key = add_rectangle(&mut g, Plane::xy(), 2.0, 1.0).expect("face should build");
    let edge = first_outer_edge_key(&g, face_key);
    let parameter = edge_mid_parameter(&g, edge);

    let split = split_face_edge(&mut g, face_key, edge, parameter).expect("face edge should split");
    let face = g.face(face_key).expect("face should remain registered");
    let loop_edges = face.face(&g).outer_loop().edges();

    assert_eq!(g.iter_edges().count(), 5);
    assert_eq!(loop_edges.len(), 5);
    assert_eq!(face.pcurves.len(), 5);
    assert!(
        loop_edges
            .iter()
            .all(|edge| face.pcurves.contains_key(&edge.dart))
    );
    assert!(
        g.vertex(split.vertex)
            .expect("split vertex should exist")
            .point
            .coincides(Point3::new(1.0, 0.0, 0.0), LINEAR_TOLERANCE)
    );
}

#[test]
fn split_face_edge_rejects_edges_outside_the_face() {
    let mut g = GMap::<StandardPayload>::new();
    let face_key = add_rectangle(&mut g, Plane::xy(), 2.0, 1.0).expect("face should build");
    let (_, edge) = add_line(
        &mut g,
        Point3::new(10.0, 0.0, 0.0),
        Point3::new(11.0, 0.0, 0.0),
    )
    .expect("outside edge should build");

    assert!(matches!(
        split_face_edge(&mut g, face_key, edge, 0.5),
        Err(FaceEdgeSplitError::EdgeNotOnFace { .. })
    ));
}

#[test]
fn split_face_edge_uses_existing_pcurve_for_non_planar_surface_variant() {
    let mut g = GMap::<StandardPayload>::new();
    let face_key = add_rectangle(&mut g, Plane::xy(), 2.0, 1.0).expect("face should build");
    let surface = g
        .face(face_key)
        .expect("face should exist")
        .surface
        .to_nurbs()
        .expect("face surface should convert to nurbs");
    g.face_mut(face_key).expect("face should exist").surface = Surface::Nurbs(surface);
    let edge = first_outer_edge_key(&g, face_key);

    split_face_edge(&mut g, face_key, edge, 0.5)
        .expect("face edge split should use existing pcurve");

    let face = g.face(face_key).expect("face should remain registered");
    assert_eq!(face.face(&g).outer_loop().edges().len(), 5);
    assert_eq!(face.pcurves.len(), 5);
}

#[test]
fn split_face_edge_updates_both_faces_of_a_shared_solid_edge() {
    let mut solid = block(2.0, 2.0, 2.0).expect("block should build");
    let face_key = solid
        .map()
        .iter_faces()
        .next()
        .expect("block should have faces")
        .0;
    let edge = first_outer_edge_key(solid.map(), face_key);
    let adjacent_faces = incident_face_keys(solid.map(), edge);
    let parameter = edge_mid_parameter(solid.map(), edge);

    assert_eq!(
        adjacent_faces.len(),
        2,
        "block edge should be shared by exactly two faces"
    );

    split_face_edge(solid.map_mut(), face_key, edge, parameter)
        .expect("shared solid edge should split");

    assert_eq!(solid.map().cells(Dim::One).count(), 13);
    assert_eq!(solid.map().cells(Dim::Zero).count(), 9);
    for face in adjacent_faces {
        let attr = solid.map().face(face).expect("adjacent face should remain");
        let edges = attr.face(solid.map()).outer_loop().edges();
        assert_eq!(edges.len(), 5);
        assert_eq!(attr.pcurves.len(), 5);
        assert!(
            edges
                .iter()
                .all(|edge| attr.pcurves.contains_key(&edge.dart)),
            "each split boundary edge should keep a pcurve on face {face:?}"
        );
    }
}

#[test]
fn split_face_by_imprints_splits_rectangle_with_boundary_chord() {
    let mut g = GMap::<StandardPayload>::new();
    let face_key = add_rectangle(&mut g, Plane::xy(), 2.0, 2.0).expect("face should build");
    let imprint = FaceImprint {
        points: vec![Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 2.0, 0.0)],
        pcurve: Curve2::Line(Line2::new(Point2::new(0.0, 0.0), Point2::new(2.0, 2.0))),
    };

    let splits = split_face_by_imprints(&mut g, face_key, &[imprint])
        .expect("face imprint split should run");

    assert!(g.face(face_key).is_none());
    assert_eq!(splits.len(), 1);
    assert_eq!(g.iter_faces().count(), 2);
    assert_eq!(g.iter_edges().count(), 5);
    assert_eq!(g.cells(Dim::Zero).count(), 4);
    assert_eq!(splits[0].section_edges.len(), 1);
    assert!(g.edge(splits[0].section_edges[0]).is_some());

    for face in [splits[0].first, splits[0].second] {
        let attr = g.face(face).expect("split face should exist");
        let edges = attr.face(&g).outer_loop().edges();
        assert_eq!(edges.len(), 3);
        assert_eq!(attr.pcurves.len(), 3);
        assert!(
            edges
                .iter()
                .all(|edge| attr.pcurves.contains_key(&edge.dart)),
            "each split face edge should have a pcurve"
        );
    }
}

#[test]
fn split_face_by_imprints_deduplicates_reversed_boundary_chords() {
    let mut g = GMap::<StandardPayload>::new();
    let face_key = add_rectangle(&mut g, Plane::xy(), 2.0, 2.0).expect("face should build");
    let first = FaceImprint {
        points: vec![Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 2.0, 0.0)],
        pcurve: Curve2::Line(Line2::new(Point2::new(0.0, 0.0), Point2::new(2.0, 2.0))),
    };
    let second = FaceImprint {
        points: vec![Point3::new(2.0, 2.0, 0.0), Point3::new(0.0, 0.0, 0.0)],
        pcurve: Curve2::Line(Line2::new(Point2::new(2.0, 2.0), Point2::new(0.0, 0.0))),
    };

    let splits = split_face_by_imprints(&mut g, face_key, &[first, second])
        .expect("face imprint split should run");

    assert_eq!(splits.len(), 1);
    assert_eq!(splits[0].section_edges.len(), 1);
    assert!(g.edge(splits[0].section_edges[0]).is_some());
    assert_eq!(g.iter_faces().count(), 2);
    assert_eq!(g.iter_edges().count(), 5);
}

#[test]
fn split_face_by_imprints_splits_boundary_edge_at_imprint_endpoint() {
    let mut g = GMap::<StandardPayload>::new();
    let face_key = add_rectangle(&mut g, Plane::xy(), 2.0, 2.0).expect("face should build");
    let imprint = FaceImprint {
        points: vec![Point3::new(1.0, 0.0, 0.0), Point3::new(2.0, 2.0, 0.0)],
        pcurve: Curve2::Line(Line2::new(Point2::new(1.0, 0.0), Point2::new(2.0, 2.0))),
    };

    let splits = split_face_by_imprints(&mut g, face_key, &[imprint])
        .expect("face imprint split should split boundary endpoint first");

    assert_eq!(splits.len(), 1);
    assert_eq!(g.iter_faces().count(), 2);
    assert_eq!(g.iter_edges().count(), 6);
    assert_eq!(g.cells(Dim::Zero).count(), 5);

    for (_, attr) in g.iter_faces() {
        let edges = attr.face(&g).outer_loop().edges();
        assert_eq!(attr.pcurves.len(), edges.len());
        assert!(
            edges
                .iter()
                .all(|edge| attr.pcurves.contains_key(&edge.dart))
        );
    }
}

#[test]
fn split_face_by_imprints_applies_multiple_non_crossing_chords() {
    let mut g = GMap::<StandardPayload>::new();
    let points = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
        Point3::new(3.0, 1.0, 0.0),
        Point3::new(1.5, 2.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    ];
    let loop_dart = add_polygon(&mut g, &points);
    let face_key = add_face(&mut g, loop_dart).expect("polygon face should build");
    let imprints = vec![
        FaceImprint {
            points: vec![points[0], points[2]],
            pcurve: Curve2::Line(Line2::new(Point2::new(0.0, 0.0), Point2::new(3.0, 1.0))),
        },
        FaceImprint {
            points: vec![points[0], points[3]],
            pcurve: Curve2::Line(Line2::new(Point2::new(0.0, 0.0), Point2::new(1.5, 2.0))),
        },
    ];

    let splits = split_face_by_imprints(&mut g, face_key, &imprints)
        .expect("non-crossing fan chords should split");

    assert_eq!(splits.len(), 2);
    assert_eq!(g.iter_faces().count(), 3);
    assert_eq!(g.iter_edges().count(), 7);

    for (_, attr) in g.iter_faces() {
        let edges = attr.face(&g).outer_loop().edges();
        assert_eq!(edges.len(), 3);
        assert_eq!(attr.pcurves.len(), 3);
        assert!(
            edges
                .iter()
                .all(|edge| attr.pcurves.contains_key(&edge.dart))
        );
    }
}

#[test]
fn split_face_by_imprints_ignores_crossing_chords_after_first_split() {
    let mut g = GMap::<StandardPayload>::new();
    let face_key = add_rectangle(&mut g, Plane::xy(), 2.0, 2.0).expect("face should build");
    let imprints = [
        FaceImprint {
            points: vec![Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 2.0, 0.0)],
            pcurve: Curve2::Line(Line2::new(Point2::new(0.0, 0.0), Point2::new(2.0, 2.0))),
        },
        FaceImprint {
            points: vec![Point3::new(2.0, 0.0, 0.0), Point3::new(0.0, 2.0, 0.0)],
            pcurve: Curve2::Line(Line2::new(Point2::new(2.0, 0.0), Point2::new(0.0, 2.0))),
        },
    ];

    let splits = split_face_by_imprints(&mut g, face_key, &imprints)
        .expect("crossing imprints should leave a valid partial split");

    assert_eq!(splits.len(), 1);
    assert_eq!(g.iter_faces().count(), 2);
    assert_eq!(g.iter_edges().count(), 5);
}

#[test]
fn split_face_by_imprints_adds_closed_interior_loop() {
    let mut g = GMap::<StandardPayload>::new();
    let face_key = add_rectangle(&mut g, Plane::xy(), 4.0, 4.0).expect("face should build");
    let imprint = FaceImprint {
        points: Vec::new(),
        pcurve: Curve2::Polyline(Polyline2::new(vec![
            Point2::new(1.0, 1.0),
            Point2::new(3.0, 1.0),
            Point2::new(3.0, 3.0),
            Point2::new(1.0, 3.0),
            Point2::new(1.0, 1.0),
        ])),
    };

    let splits = split_face_by_imprints(&mut g, face_key, &[imprint])
        .expect("closed interior imprint should split the face into regions");

    assert_eq!(splits.len(), 1);
    assert_eq!(splits[0].first, face_key);
    assert_eq!(splits[0].section_edges.len(), 4);
    assert!(
        splits[0]
            .section_edges
            .iter()
            .all(|edge| g.edge(*edge).is_some())
    );
    assert_eq!(g.iter_faces().count(), 2);
    assert_eq!(g.iter_edges().count(), 8);
    assert_eq!(g.cells(Dim::Zero).count(), 8);

    let face = g.face(face_key).expect("original face should remain");
    assert_eq!(face.inner_loops.len(), 1);
    assert_eq!(face.pcurves.len(), 8);

    let shape_face = face.face(&g);
    let hole = shape_face.inner_loops()[0].clone();
    assert_eq!(hole.edges().len(), 4);
    assert!(
        hole.edges()
            .iter()
            .all(|edge| face.pcurves.contains_key(&edge.dart))
    );

    let island = g
        .face(splits[0].second)
        .expect("interior island face should exist");
    assert!(island.inner_loops.is_empty());
    assert_eq!(island.face(&g).outer_loop().edges().len(), 4);
    assert_eq!(island.pcurves.len(), 4);
}

#[test]
fn face_imprint_graph_splits_crossing_segments_at_interior_vertex() {
    let graph = FaceImprintGraph::from_imprints(&[
        FaceImprint {
            points: Vec::new(),
            pcurve: Curve2::Line(Line2::new(Point2::new(0.0, 0.0), Point2::new(2.0, 2.0))),
        },
        FaceImprint {
            points: Vec::new(),
            pcurve: Curve2::Line(Line2::new(Point2::new(2.0, 0.0), Point2::new(0.0, 2.0))),
        },
    ]);

    assert_eq!(graph.vertices().len(), 5);
    assert_eq!(graph.edges().len(), 4);
    assert_eq!(graph.branch_vertices().len(), 1);
    let branch = graph.branch_vertices()[0];
    assert_eq!(graph.vertex_degree(branch), 4);
    assert!((graph.vertices()[branch].uv - Point2::new(1.0, 1.0)).norm() <= LINEAR_TOLERANCE);
}

#[test]
fn face_imprint_graph_splits_t_junction_segments() {
    let graph = FaceImprintGraph::from_imprints(&[
        FaceImprint {
            points: Vec::new(),
            pcurve: Curve2::Line(Line2::new(Point2::new(0.0, 0.0), Point2::new(2.0, 0.0))),
        },
        FaceImprint {
            points: Vec::new(),
            pcurve: Curve2::Line(Line2::new(Point2::new(1.0, 1.0), Point2::new(1.0, 0.0))),
        },
    ]);

    assert_eq!(graph.vertices().len(), 4);
    assert_eq!(graph.edges().len(), 3);
    assert_eq!(graph.branch_vertices().len(), 1);
    let branch = graph.branch_vertices()[0];
    assert_eq!(graph.vertex_degree(branch), 3);
    assert!((graph.vertices()[branch].uv - Point2::new(1.0, 0.0)).norm() <= LINEAR_TOLERANCE);
}

#[test]
fn face_imprint_graph_detects_closed_loop_components() {
    let graph = FaceImprintGraph::from_imprints(&[FaceImprint {
        points: Vec::new(),
        pcurve: Curve2::Polyline(Polyline2::new(vec![
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
            Point2::new(1.0, 1.0),
            Point2::new(0.0, 1.0),
            Point2::new(0.0, 0.0),
        ])),
    }]);

    assert_eq!(graph.vertices().len(), 4);
    assert_eq!(graph.edges().len(), 4);
    assert_eq!(graph.closed_component_count(), 1);
    assert!(graph.branch_vertices().is_empty());
}

fn first_outer_edge_key(g: &GMap<StandardPayload>, face_key: FaceKey) -> EdgeKey {
    let face = g.face(face_key).expect("face should exist").face(g);
    let dart = Profile::new(g, face.outer_loop().dart).edges()[0].dart;
    edge_key_for_dart(g, dart)
}

fn incident_face_keys(g: &GMap<StandardPayload>, edge: EdgeKey) -> Vec<FaceKey> {
    let edge_dart = g.edge(edge).expect("edge should exist").dart;
    let mut seen = HashSet::new();
    let mut faces = g
        .orbit(edge_dart, g.orbit_indices(Dim::One))
        .filter_map(|dart| g.attribute::<Cell2>(dart).copied())
        .filter(|face| seen.insert(*face))
        .collect::<Vec<_>>();
    faces.sort_by_key(|face| format!("{face:?}"));
    faces
}

fn edge_mid_parameter(g: &GMap<StandardPayload>, edge: EdgeKey) -> f64 {
    let attr = g.edge(edge).expect("edge should exist");
    let start = g
        .attribute::<Cell0>(attr.dart)
        .expect("edge start should have a vertex")
        .point;
    let end_dart = g.alpha(Dim::Zero, attr.dart);
    let end = g
        .attribute::<Cell0>(end_dart)
        .expect("edge end should have a vertex")
        .point;
    let interval = attr.curve.parameters_between(start, end);
    0.5 * (interval.start + interval.end)
}

fn edge_key_for_dart(g: &GMap<StandardPayload>, dart: ngk::topology::Dart) -> EdgeKey {
    let representative = g.cell_representative(dart, Dim::One);
    g.iter_edges()
        .find_map(|(key, edge)| (edge.dart == representative).then_some(key))
        .expect("edge key should exist for dart")
}
