use std::collections::HashSet;

use nalgebra::Vector3;
use ngk::builders::edges::add_circle as add_circle_edge;
use ngk::builders::edges::add_line;
use ngk::builders::errors::{FaceCreationError, PolylineError};
use ngk::builders::faces::{
    FaceEdgeSplitError, FaceImprint, FaceImprintGraph, add_annulus, add_circle, add_face,
    add_polygon, add_rectangle, split_face_by_imprints, split_face_edge,
};
use ngk::builders::profiles::add_polyline;
use ngk::builders::sheets::add_extruded_profile;
use ngk::geometry::{
    Curve, Curve2, LINEAR_TOLERANCE, Line2, NurbsCurve2, Plane, Point2, Point3, PointCoincidence,
    Surface,
};
use ngk::topology::TopologyEditError;
use ngk::topology::gmap::GMap;
use ngk::topology::gmap::{Cell0, Dim};
use ngk::topology::payload::StandardPayload;
use ngk::topology::shape_keys::{EdgeKey, FaceKey};
use ngk::viz::debug_viewer::show;
use ngk::viz::debug_viewer::show_gmap;

#[test]
fn add_rectangle_creates_single_planar_face_with_pcurves() {
    let mut g = GMap::<StandardPayload>::new();
    let face_key = add_rectangle(&mut g, Plane::xy(), 2.0, 3.0).expect("face should build");
    let face = g.face_attr_unchecked(face_key);

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
    let face = g.face_attr_unchecked(face_key);

    assert_eq!(g.iter_faces().count(), 1);
    assert_eq!(g.iter_edges().count(), 1);
    assert!(matches!(face.surface, Surface::Plane(_)));
    assert_eq!(face.inner_loops.len(), 0);
    assert_eq!(face.pcurves.len(), 1);

    let shape_face = face.face(&g);
    let edges = shape_face.outer_loop().edges();
    let edge = &edges[0];
    let pcurve = shape_face
        .pcurve(edge.dart())
        .expect("circle edge should have a pcurve");
    assert!(matches!(pcurve, Curve2::Nurbs(_)));
    for fraction in [0.0, 0.125, 0.25, 0.5, 0.875, 1.0] {
        let uv = pcurve.point_at(fraction);
        let surface_point = shape_face.point_at(uv.x, uv.y);
        let edge_point = edge
            .curve()
            .expect("circle edge should have geometry")
            .point_at(std::f64::consts::TAU * fraction);
        assert!(surface_point.coincides(edge_point, LINEAR_TOLERANCE));
    }
}

#[test]
fn add_annulus_creates_planar_face_with_inner_circular_loop() {
    let mut g = GMap::<StandardPayload>::new();
    let face_key = add_annulus(&mut g, Plane::xy(), 2.0, 1.0).expect("annulus face should build");
    let face = g.face_attr_unchecked(face_key);

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
    let face = g.face_attr_unchecked(face_key);
    let shape_face = face.face(&g);
    let loop_edges = shape_face.outer_loop().edges();

    assert_eq!(g.iter_edges().count(), 5);
    assert_eq!(loop_edges.len(), 5);
    assert_eq!(face.pcurves.len(), 5);
    assert!(
        loop_edges
            .iter()
            .all(|edge| shape_face.pcurve(edge.dart()).is_some())
    );
    assert!(
        g.vertex_attr_unchecked(split.vertex)
            .point
            .coincides(Point3::new(1.0, 0.0, 0.0), LINEAR_TOLERANCE)
    );
}

#[test]
fn split_face_edge_rejects_edges_outside_the_face() {
    let mut g = GMap::<StandardPayload>::new();
    let face_key = add_rectangle(&mut g, Plane::xy(), 2.0, 1.0).expect("face should build");
    let edge = add_line(
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
        .face_attr_unchecked(face_key)
        .surface
        .to_nurbs()
        .expect("face surface should convert to nurbs");
    g.transaction(|edit| {
        edit.face_attr_mut_unchecked(face_key).surface = Surface::Nurbs(surface);
        Ok::<_, TopologyEditError>(())
    })
    .unwrap();
    let edge = first_outer_edge_key(&g, face_key);

    split_face_edge(&mut g, face_key, edge, 0.5)
        .expect("face edge split should use existing pcurve");

    let face = g.face_attr_unchecked(face_key);
    assert_eq!(face.face(&g).outer_loop().edges().len(), 5);
    assert_eq!(face.pcurves.len(), 5);
}

#[test]
fn split_face_edge_splits_shared_edge_of_two_extruded_faces() {
    let mut g = GMap::<StandardPayload>::new();
    let points = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
    ];
    let profile = add_polyline(&mut g, &points).expect("two-edge profile should build");
    add_extruded_profile(&mut g, profile, Vector3::new(0.0, 0.0, 2.0))
        .expect("profile should extrude into two faces");

    let edge = edge_between_points(&g, Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 2.0));
    let adjacent_faces = incident_face_keys(&g, edge);
    let parameter = 0.75;
    let edge_count = g.iter_edges().count();
    let vertex_count = g.iter_vertices().count();
    assert_eq!(
        adjacent_faces.len(),
        2,
        "the middle sweep edge should be shared by both extruded faces"
    );
    assert_eq!(g.iter_faces().count(), 2);

    let split = split_face_edge(&mut g, adjacent_faces[0], edge, parameter)
        .expect("shared solid edge should split");

    assert_eq!(g.iter_edges().count(), edge_count + 1);
    assert_eq!(g.iter_vertices().count(), vertex_count + 1);
    assert!(
        g.vertex_attr_unchecked(split.vertex)
            .point
            .coincides(Point3::new(1.0, 0.0, 1.5), LINEAR_TOLERANCE)
    );
    assert_eq!(incident_face_keys(&g, split.first), adjacent_faces);
    assert_eq!(incident_face_keys(&g, split.second), adjacent_faces);

    for facekey in adjacent_faces {
        let face = g.face_unchecked(facekey);
        let edges = face.outer_loop().edges();
        assert_eq!(edges.len(), 5);
        assert_eq!(g.face_attr_unchecked(facekey).pcurves.len(), 5);
        assert!(
            edges.iter().all(|edge| face.pcurve(edge.dart()).is_some()),
            "each split boundary edge should keep a pcurve on face {facekey:?}"
        );
    }
}

#[test]
fn split_face_by_imprints_splits_rectangle_with_boundary_chord() {
    let mut g = GMap::<StandardPayload>::new();
    let face_key = add_rectangle(&mut g, Plane::xy(), 2.0, 2.0).expect("face should build");
    let imprint = Curve2::Line(Line2::new(Point2::new(0.0, 0.0), Point2::new(2.0, 2.0)));

    let splits = split_face_by_imprints(&mut g, face_key, &[planar_imprint(imprint)])
        .expect("face imprint split should run");

    assert_eq!(splits[0].first, face_key);
    assert!(g.face_attr(face_key).is_some());
    assert_eq!(splits.len(), 1);
    assert_eq!(g.iter_faces().count(), 2);
    assert_eq!(g.iter_edges().count(), 5);
    assert_eq!(g.iter_vertices().count(), 4);
    assert_eq!(splits[0].sections.len(), 1);
    assert!(g.edge_attr(splits[0].sections[0].edge).is_some());

    for face in [splits[0].first, splits[0].second] {
        let attr = g.face_attr_unchecked(face);
        let shape_face = attr.face(&g);
        let edges = shape_face.outer_loop().edges();
        assert_eq!(edges.len(), 3);
        assert_eq!(attr.pcurves.len(), 3);
        assert!(
            edges
                .iter()
                .all(|edge| shape_face.pcurve(edge.dart()).is_some()),
            "each split face edge should have a pcurve"
        );
    }
}

#[test]
fn split_face_by_imprints_deduplicates_reversed_boundary_chords() {
    let mut g = GMap::<StandardPayload>::new();
    let face_key = add_rectangle(&mut g, Plane::xy(), 2.0, 2.0).expect("face should build");
    let first = Curve2::Line(Line2::new(Point2::new(0.0, 0.0), Point2::new(2.0, 2.0)));
    let second = Curve2::Line(Line2::new(Point2::new(2.0, 2.0), Point2::new(0.0, 0.0)));

    let splits = split_face_by_imprints(
        &mut g,
        face_key,
        &[planar_imprint(first), planar_imprint(second)],
    )
    .expect("face imprint split should run");

    assert_eq!(splits.len(), 1);
    assert_eq!(splits[0].sections.len(), 1);
    assert!(g.edge_attr(splits[0].sections[0].edge).is_some());
    assert_eq!(g.iter_faces().count(), 2);
    assert_eq!(g.iter_edges().count(), 5);
}

#[test]
fn split_face_by_imprints_splits_boundary_edge_at_imprint_endpoint() {
    let mut g = GMap::<StandardPayload>::new();
    let face_key = add_rectangle(&mut g, Plane::xy(), 2.0, 2.0).expect("face should build");
    let imprint = Curve2::Line(Line2::new(Point2::new(1.0, 0.0), Point2::new(2.0, 2.0)));

    let splits = split_face_by_imprints(&mut g, face_key, &[planar_imprint(imprint)])
        .expect("face imprint split should split boundary endpoint first");

    assert_eq!(splits.len(), 1);
    assert_eq!(g.iter_faces().count(), 2);
    assert_eq!(g.iter_edges().count(), 6);
    assert_eq!(g.cells(Dim::Zero).count(), 5);

    for (_, attr) in g.iter_faces() {
        let shape_face = attr.face(&g);
        let edges = shape_face.outer_loop().edges();
        assert_eq!(attr.pcurves.len(), edges.len());
        assert!(
            edges
                .iter()
                .all(|edge| shape_face.pcurve(edge.dart()).is_some())
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
    let profile_key = add_polygon(&mut g, &points);
    let face_key = add_face(&mut g, profile_key).expect("polygon face should build");
    let imprints = vec![
        planar_imprint(Curve2::Line(Line2::new(
            Point2::new(0.0, 0.0),
            Point2::new(3.0, 1.0),
        ))),
        planar_imprint(Curve2::Line(Line2::new(
            Point2::new(0.0, 0.0),
            Point2::new(1.5, 2.0),
        ))),
    ];

    let splits = split_face_by_imprints(&mut g, face_key, &imprints)
        .expect("non-crossing fan chords should split");

    assert_eq!(splits.len(), 2);
    assert_eq!(g.iter_faces().count(), 3);
    assert_eq!(g.iter_edges().count(), 7);

    for (_, attr) in g.iter_faces() {
        let shape_face = attr.face(&g);
        let edges = shape_face.outer_loop().edges();
        assert_eq!(edges.len(), 3);
        assert_eq!(attr.pcurves.len(), 3);
        assert!(
            edges
                .iter()
                .all(|edge| shape_face.pcurve(edge.dart()).is_some())
        );
    }
}

#[test]
fn split_face_by_imprints_ignores_crossing_chords_after_first_split() {
    let mut g = GMap::<StandardPayload>::new();
    let face_key = add_rectangle(&mut g, Plane::xy(), 2.0, 2.0).expect("face should build");
    let imprints = [
        planar_imprint(Curve2::Line(Line2::new(
            Point2::new(0.0, 0.0),
            Point2::new(2.0, 2.0),
        ))),
        planar_imprint(Curve2::Line(Line2::new(
            Point2::new(2.0, 0.0),
            Point2::new(0.0, 2.0),
        ))),
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
    let points = [
        Point2::new(1.0, 1.0),
        Point2::new(3.0, 1.0),
        Point2::new(3.0, 3.0),
        Point2::new(1.0, 3.0),
        Point2::new(1.0, 1.0),
    ];
    let imprints = points
        .windows(2)
        .map(|pair| planar_imprint(Curve2::Line(Line2::new(pair[0], pair[1]))))
        .collect::<Vec<_>>();

    let splits = split_face_by_imprints(&mut g, face_key, &imprints)
        .expect("closed interior imprint should split the face into regions");

    assert_eq!(splits.len(), 1);
    assert_eq!(splits[0].first, face_key);
    assert_eq!(splits[0].sections.len(), 4);
    assert!(
        splits[0]
            .sections
            .iter()
            .all(|section| g.edge_attr(section.edge).is_some())
    );
    assert_eq!(g.iter_faces().count(), 2);
    assert_eq!(g.iter_edges().count(), 8);
    assert_eq!(g.cells(Dim::Zero).count(), 8);

    let face = g.face_attr_unchecked(face_key);
    assert_eq!(face.inner_loops.len(), 1);
    assert_eq!(face.pcurves.len(), 8);

    let shape_face = face.face(&g);
    let hole = shape_face.inner_loops()[0].clone();
    assert_eq!(hole.edges().len(), 4);
    assert!(
        hole.edges()
            .iter()
            .all(|edge| shape_face.pcurve(edge.dart()).is_some())
    );

    let island = g.face_attr_unchecked(splits[0].second);
    assert!(island.inner_loops.is_empty());
    assert_eq!(island.face(&g).outer_loop().edges().len(), 4);
    assert_eq!(island.pcurves.len(), 4);
}

#[test]
fn face_imprint_graph_splits_crossing_segments_at_interior_vertex() {
    let graph = FaceImprintGraph::from_curves(&[
        Curve2::Line(Line2::new(Point2::new(0.0, 0.0), Point2::new(2.0, 2.0))),
        Curve2::Line(Line2::new(Point2::new(2.0, 0.0), Point2::new(0.0, 2.0))),
    ])
    .expect("imprint graph should build");

    assert_eq!(graph.vertices().len(), 5);
    assert_eq!(graph.edges().len(), 4);
    assert_eq!(graph.branch_vertices().len(), 1);
    assert_eq!(
        graph
            .edges()
            .iter()
            .filter(|edge| edge.source_curve == 0)
            .count(),
        2
    );
    assert_eq!(
        graph
            .edges()
            .iter()
            .filter(|edge| edge.source_curve == 1)
            .count(),
        2
    );
    assert!(graph.edges().iter().all(|edge| {
        (edge.interval.start - 0.0).abs() <= LINEAR_TOLERANCE
            && (edge.interval.end - 0.5).abs() <= LINEAR_TOLERANCE
            || (edge.interval.start - 0.5).abs() <= LINEAR_TOLERANCE
                && (edge.interval.end - 1.0).abs() <= LINEAR_TOLERANCE
    }));
    let branch = graph.branch_vertices()[0];
    assert_eq!(graph.vertex_degree(branch), 4);
    assert!((graph.vertices()[branch] - Point2::new(1.0, 1.0)).norm() <= LINEAR_TOLERANCE);
}

#[test]
fn face_imprint_graph_splits_t_junction_segments() {
    let graph = FaceImprintGraph::from_curves(&[
        Curve2::Line(Line2::new(Point2::new(0.0, 0.0), Point2::new(2.0, 0.0))),
        Curve2::Line(Line2::new(Point2::new(1.0, 1.0), Point2::new(1.0, 0.0))),
    ])
    .expect("imprint graph should build");

    assert_eq!(graph.vertices().len(), 4);
    assert_eq!(graph.edges().len(), 3);
    assert_eq!(graph.branch_vertices().len(), 1);
    let branch = graph.branch_vertices()[0];
    assert_eq!(graph.vertex_degree(branch), 3);
    assert!((graph.vertices()[branch] - Point2::new(1.0, 0.0)).norm() <= LINEAR_TOLERANCE);
}

#[test]
fn face_imprint_graph_detects_closed_loop_components() {
    let points = [
        Point2::new(0.0, 0.0),
        Point2::new(1.0, 0.0),
        Point2::new(1.0, 1.0),
        Point2::new(0.0, 1.0),
        Point2::new(0.0, 0.0),
    ];
    let curves = points
        .windows(2)
        .map(|pair| Curve2::Line(Line2::new(pair[0], pair[1])))
        .collect::<Vec<_>>();
    let graph = FaceImprintGraph::from_curves(&curves).expect("imprint graph should build");

    assert_eq!(graph.vertices().len(), 4);
    assert_eq!(graph.edges().len(), 4);
    assert_eq!(graph.closed_component_count(), 1);
    assert!(graph.branch_vertices().is_empty());
}

#[test]
fn split_face_by_imprints_preserves_curved_section_edge_geometry() {
    let mut g = GMap::<StandardPayload>::new();
    let face_key = add_rectangle(&mut g, Plane::xy(), 4.0, 4.0).expect("face should build");
    let pcurve = Curve2::Nurbs(
        NurbsCurve2::interpolate(&[
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 2.0),
            Point2::new(3.0, 2.0),
            Point2::new(4.0, 4.0),
        ])
        .unwrap(),
    );

    let splits = split_face_by_imprints(&mut g, face_key, &[planar_imprint(pcurve)])
        .expect("curved face imprint should split");
    let edge = g.edge_attr_unchecked(splits[0].sections[0].edge);

    assert!(matches!(edge.curve, Curve::Nurbs(_)));
}

#[test]
fn split_face_preserves_curved_loop() {
    let mut g = GMap::<StandardPayload>::new();
    let face_key = add_rectangle(&mut g, Plane::xy(), 6.0, 6.0).expect("face should build");
    let pcurves = [
        [
            Point2::new(1.0, 1.0),
            Point2::new(3.0, 1.0 - 1.0e-8),
            Point2::new(5.0, 1.0),
        ],
        [
            Point2::new(5.0, 1.0),
            Point2::new(5.0 + 1.0e-8, 3.0),
            Point2::new(5.0, 5.0),
        ],
        [
            Point2::new(5.0, 5.0),
            Point2::new(3.0, 5.0 + 1.0e-8),
            Point2::new(1.0, 5.0),
        ],
        [
            Point2::new(1.0, 5.0),
            Point2::new(1.0 - 1.0e-8, 3.0),
            Point2::new(1.0, 1.0),
        ],
    ]
    .map(|points| Curve2::Nurbs(NurbsCurve2::interpolate(&points).unwrap()));
    let imprints = pcurves.into_iter().map(planar_imprint).collect::<Vec<_>>();

    let splits =
        split_face_by_imprints(&mut g, face_key, &imprints).expect("curved loop should split");

    show(&g);
    assert_eq!(splits.len(), 1);
    assert_eq!(splits[0].sections.len(), 4);
    assert!(
        splits[0]
            .sections
            .iter()
            .all(|edge| { matches!(g.edge_attr_unchecked(edge.edge).curve, Curve::Nurbs(_)) })
    );
}

#[test]
fn split_face_by_imprints_preserves_closed_nurbs_as_single_curved_edge() {
    let mut g = GMap::<StandardPayload>::new();
    let face_key = add_rectangle(&mut g, Plane::xy(), 4.0, 4.0).expect("face should build");
    let pcurve = Curve2::Nurbs(
        NurbsCurve2::interpolate(&[
            Point2::new(1.0, 1.0),
            Point2::new(3.0, 1.0),
            Point2::new(3.0, 3.0),
            Point2::new(1.0, 3.0),
            Point2::new(1.0, 1.0),
        ])
        .unwrap(),
    );

    let splits = split_face_by_imprints(&mut g, face_key, &[planar_imprint(pcurve)])
        .expect("closed curved imprint should split");

    assert_eq!(splits.len(), 1);
    assert_eq!(splits[0].sections.len(), 1);
    assert!(matches!(
        g.edge_attr_unchecked(splits[0].sections[0].edge).curve,
        Curve::Nurbs(_)
    ));
    assert_eq!(
        g.face_attr_unchecked(face_key).face(&g).inner_loops()[0]
            .edges()
            .len(),
        1
    );
}

fn planar_imprint(pcurve: Curve2) -> FaceImprint {
    let points = pcurve
        .sample(32)
        .into_iter()
        .map(|point| Point3::new(point.x, point.y, 0.0))
        .collect::<Vec<_>>();
    let curve = match &pcurve {
        Curve2::Line(_) => Curve::line(points[0], *points.last().unwrap()),
        Curve2::Circle(_) | Curve2::Nurbs(_) => Curve::Nurbs(
            ngk::geometry::NurbsCurve::interpolate(&points)
                .expect("sampled planar pcurve should interpolate in 3D"),
        ),
    };
    FaceImprint::new(curve, pcurve)
}

fn first_outer_edge_key(g: &GMap<StandardPayload>, face_key: FaceKey) -> EdgeKey {
    let face = g.face_attr_unchecked(face_key).face(g);
    face.outer_loop().edges()[0].key()
}

fn incident_face_keys(g: &GMap<StandardPayload>, edge: EdgeKey) -> Vec<FaceKey> {
    let mut faces = g
        .edge_unchecked(edge)
        .faces()
        .into_iter()
        .map(|face| face.key())
        .collect::<Vec<_>>();
    faces.sort_by_key(|face| format!("{face:?}"));
    faces
}

fn edge_mid_parameter(g: &GMap<StandardPayload>, edge: EdgeKey) -> f64 {
    let attr = g.edge_attr_unchecked(edge);
    let start = g.attribute_unchecked::<Cell0>(attr.dart).point;
    let end_dart = g.alpha(Dim::Zero, attr.dart);
    let end = g.attribute_unchecked::<Cell0>(end_dart).point;
    let interval = attr.curve.parameters_between(start, end);
    0.5 * (interval.start + interval.end)
}

fn edge_between_points(g: &GMap<StandardPayload>, first: Point3, second: Point3) -> EdgeKey {
    g.iter_edges()
        .find_map(|(key, edge)| {
            let start = g.attribute::<Cell0>(edge.dart)?.point;
            let end = g.attribute::<Cell0>(g.alpha(Dim::Zero, edge.dart))?.point;
            ((start.coincides(first, LINEAR_TOLERANCE) && end.coincides(second, LINEAR_TOLERANCE))
                || (start.coincides(second, LINEAR_TOLERANCE)
                    && end.coincides(first, LINEAR_TOLERANCE)))
            .then_some(key)
        })
        .expect("edge should connect the requested points")
}

#[test]
fn imprint_sections_retain_source_indices_and_directed_intervals() {
    let mut g = GMap::<StandardPayload>::new();
    let face = add_rectangle(&mut g, Plane::xy(), 4.0, 4.0).unwrap();
    let points = [
        Point2::new(1.0, 1.0),
        Point2::new(3.0, 1.0),
        Point2::new(3.0, 3.0),
        Point2::new(1.0, 3.0),
        Point2::new(1.0, 1.0),
    ];
    let imprints = points
        .windows(2)
        .map(|pair| planar_imprint(Curve2::Line(Line2::new(pair[0], pair[1]))))
        .collect::<Vec<_>>();
    let splits = split_face_by_imprints(&mut g, face, &imprints).unwrap();
    let sections = &splits[0].sections;
    assert_eq!(sections.len(), 4);
    let mut indices = sections
        .iter()
        .map(|section| section.imprint)
        .collect::<Vec<_>>();
    indices.sort_unstable();
    assert_eq!(indices, vec![0, 1, 2, 3]);
    for section in sections {
        let edge = g.edge_unchecked(section.edge);
        let source = &imprints[section.imprint].curve;
        assert!(
            edge.start()
                .point()
                .unwrap()
                .coincides(source.point_at(section.interval.start), LINEAR_TOLERANCE)
        );
        assert!(
            edge.end()
                .point()
                .unwrap()
                .coincides(source.point_at(section.interval.end), LINEAR_TOLERANCE)
        );
    }
}
