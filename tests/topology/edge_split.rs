use ngk::builders::edges::{EdgeSplitError, add_line, split_edge};
use ngk::builders::profiles::{add_polyline, add_rectangle};
use ngk::geometry::{LINEAR_TOLERANCE, Point3, PointCoincidence};
use ngk::modeling::faces;
use ngk::topology::closed::Closeable;
use ngk::topology::gmap::GMap;
use ngk::topology::payload::{Payload, StandardPayload};
use ngk::topology::profile::Profile;
use ngk::topology::shape_keys::EdgeKey;

#[derive(Clone, Default)]
struct EdgePayload;

impl Payload for EdgePayload {
    type V = ();
    type E = String;
    type Profile = ();
    type F = ();
    type Sheet = ();
    type S = ();
}

#[test]
fn split_profile_edge_handles_isolated_edge() {
    let mut g = GMap::<StandardPayload>::new();
    let start = Point3::new(0.0, 0.0, 0.0);
    let end = Point3::new(1.0, 0.0, 0.0);
    let edge = add_line(&mut g, start, end).expect("line edge should build");

    let split = split_edge(&mut g, edge, 0.25).expect("isolated edge should split");

    assert_eq!(g.iter_edges().count(), 2);
    assert_eq!(g.iter_vertices().count(), 3);

    let midpoint = Point3::new(0.25, 0.0, 0.0);
    let split_vertex = g.vertex_attr_unchecked(split.vertex).vertex(&g);
    assert!(
        split_vertex
            .point()
            .unwrap()
            .coincides(midpoint, LINEAR_TOLERANCE)
    );

    let first = g.edge_unchecked(split.first);
    let second = g.edge_unchecked(split.second);
    assert!(
        first
            .start()
            .point()
            .unwrap()
            .coincides(start, LINEAR_TOLERANCE)
    );
    assert!(
        first
            .end()
            .point()
            .unwrap()
            .coincides(midpoint, LINEAR_TOLERANCE)
    );

    assert!(
        second
            .start()
            .point()
            .expect("second split edge start should have geometry")
            .coincides(midpoint, LINEAR_TOLERANCE),
        "the second split edge should start at the split point"
    );
    assert!(
        second
            .end()
            .point()
            .expect("second split edge end should have geometry")
            .coincides(end, LINEAR_TOLERANCE),
        "the second split edge should preserve the original end point"
    );
}

#[test]
fn split_isolated_edge_keeps_edge_profile_free() {
    let mut g = GMap::<StandardPayload>::new();
    let edge = add_line(
        &mut g,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
    )
    .expect("line edge should build");

    let split = split_edge(&mut g, edge, 0.5).expect("isolated edge should split");

    assert!(Profile::from_dart(&g, g.edge_attr_unchecked(split.first).dart).is_none());
    assert!(Profile::from_dart(&g, g.edge_attr_unchecked(split.second).dart).is_none());
}

#[test]
fn split_edge_initializes_split_edge_payload_from_source() {
    let mut g = GMap::<EdgePayload>::new();
    let edge = add_line(
        &mut g,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
    )
    .expect("line edge should build");
    g.edge_attr_mut_unchecked(edge).data = "source".to_owned();

    let split = split_edge(&mut g, edge, 0.5).expect("edge should split");

    assert_eq!(g.edge_attr_unchecked(split.first).data, "source");
    assert_eq!(g.edge_attr_unchecked(split.second).data, "source");
}

#[test]
fn split_profile_edge_rejects_boundary_parameters() {
    let mut g = GMap::<StandardPayload>::new();
    let edge = add_line(
        &mut g,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
    )
    .expect("line edge should build");

    assert!(matches!(
        split_edge(&mut g, edge, 0.0),
        Err(EdgeSplitError::DegenerateSplit { .. })
    ));
    assert!(matches!(
        split_edge(&mut g, edge, 1.0),
        Err(EdgeSplitError::DegenerateSplit { .. })
    ));
}

#[test]
fn split_profile_edge_rejects_face_boundary_edges() {
    let mut face = faces::rectangle(ngk::geometry::Plane::xy(), 1.0, 1.0)
        .expect("rectangle face should build");
    let edge = face
        .map()
        .iter_edges()
        .next()
        .expect("rectangle should contain edges")
        .0;

    assert!(matches!(
        split_edge(face.map_mut(), edge, 0.5),
        Err(EdgeSplitError::EdgeBelongsToFace { .. })
    ));
}

#[test]
fn split_profile_edge_preserves_open_profile_order() {
    let mut g = GMap::<StandardPayload>::new();
    let p0 = Point3::new(0.0, 0.0, 0.0);
    let p1 = Point3::new(1.0, 0.0, 0.0);
    let p2 = Point3::new(2.0, 0.0, 0.0);
    let profile_key = add_polyline(&mut g, &[p0, p1, p2]).expect("profile should build");
    let first_edge = g.profile_unchecked(profile_key).edges()[0].key();

    let split = split_edge(&mut g, first_edge, 0.5).expect("profile edge should split");
    let profile = g.profile_unchecked(profile_key);
    let midpoint = Point3::new(0.5, 0.0, 0.0);

    assert_eq!(profile.edges().len(), 3);
    assert!(
        g.vertex_attr_unchecked(split.vertex)
            .point
            .coincides(midpoint, LINEAR_TOLERANCE)
    );
}

#[test]
fn split_profile_edge_preserves_closed_profile() {
    let mut g = GMap::<StandardPayload>::new();
    let profile_key = add_rectangle(&mut g, ngk::geometry::Plane::xy(), 1.0, 1.0)
        .expect("rectangle profile should build");
    let first_edge_dart = g.profile_unchecked(profile_key).edges()[0].dart();
    let first_edge = edge_key_for_dart(&g, first_edge_dart);

    split_edge(&mut g, first_edge, 0.5).expect("closed profile edge should split");
    let profile = g.profile_unchecked(profile_key);

    assert!(profile.is_closed());
    assert_eq!(profile.edges().len(), 5);
}

fn edge_key_for_dart(g: &GMap<StandardPayload>, dart: ngk::topology::Dart) -> EdgeKey {
    g.cell_key_unchecked::<ngk::topology::gmap::Cell1>(dart)
}
