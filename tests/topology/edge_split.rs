use ngk::builders::edges::{EdgeSplitError, add_line, split_edge};
use ngk::builders::profiles::{add_edge_to_profile, add_rectangle};
use ngk::geometry::{LINEAR_TOLERANCE, Point3, PointCoincidence};
use ngk::modeling::faces;
use ngk::topology::closed::Closeable;
use ngk::topology::gmap::{Dim, GMap};
use ngk::topology::payload::StandardPayload;
use ngk::topology::profile::Profile;
use ngk::topology::shape_keys::EdgeKey;

#[test]
fn split_profile_edge_handles_isolated_edge() {
    let mut g = GMap::<StandardPayload>::new();
    let start = Point3::new(0.0, 0.0, 0.0);
    let end = Point3::new(1.0, 0.0, 0.0);
    let (_, edge) = add_line(&mut g, start, end).expect("line edge should build");

    let split = split_edge(&mut g, edge, 0.25).expect("isolated edge should split");

    assert_eq!(g.iter_edges().count(), 2);
    assert_eq!(g.iter_vertices().count(), 3);

    let midpoint = Point3::new(0.25, 0.0, 0.0);
    let split_vertex = g
        .vertex_attr(split.vertex)
        .expect("split vertex should be stored")
        .vertex(&g);
    assert!(
        split_vertex
            .point()
            .unwrap()
            .coincides(midpoint, LINEAR_TOLERANCE)
    );

    let first = g
        .edge(split.first)
        .expect("first edge should exist");
    let second = g
        .edge(split.second)
        .expect("second edge should exist");
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

    let second_vertices = second
        .vertices()
        .into_iter()
        .map(|vertex| {
            *vertex
                .point()
                .expect("split edge vertex should have a point")
        })
        .collect::<Vec<_>>();
    assert!(
        second_vertices
            .iter()
            .any(|point| point.coincides(midpoint, LINEAR_TOLERANCE))
    );
    assert!(
        second_vertices
            .iter()
            .any(|point| point.coincides(end, LINEAR_TOLERANCE))
    );
}

#[test]
fn split_profile_edge_turns_isolated_edge_into_open_profile() {
    let mut g = GMap::<StandardPayload>::new();
    let (_, edge) = add_line(
        &mut g,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
    )
    .expect("line edge should build");

    let split = split_edge(&mut g, edge, 0.5).expect("isolated edge should split");
    let first_dart = g
        .edge_attr(split.first)
        .expect("first edge should exist")
        .dart;
    let profile = Profile::new(&g, first_dart);

    assert_eq!(profile.edges().len(), 2);
}

#[test]
fn split_profile_edge_rejects_boundary_parameters() {
    let mut g = GMap::<StandardPayload>::new();
    let (_, edge) = add_line(
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
    let (first_dart, first_edge) = add_line(&mut g, p0, p1).expect("first edge should build");
    let (second_dart, _) = add_line(&mut g, p1, p2).expect("second edge should build");
    add_edge_to_profile(&mut g, first_dart, second_dart)
        .expect("edges should connect into a profile");

    let split = split_edge(&mut g, first_edge, 0.5).expect("profile edge should split");
    let profile = Profile::new(&g, first_dart);
    let midpoint = Point3::new(0.5, 0.0, 0.0);

    assert_eq!(profile.edges().len(), 3);
    assert!(
        g.vertex_attr(split.vertex)
            .expect("split vertex should exist")
            .point
            .coincides(midpoint, LINEAR_TOLERANCE)
    );
}

#[test]
fn split_profile_edge_preserves_closed_profile() {
    let mut g = GMap::<StandardPayload>::new();
    let profile_dart = add_rectangle(&mut g, ngk::geometry::Plane::xy(), 1.0, 1.0)
        .expect("rectangle profile should build");
    let first_edge_dart = Profile::new(&g, profile_dart).edges()[0].dart();
    let first_edge = edge_key_for_dart(&g, first_edge_dart);

    split_edge(&mut g, first_edge, 0.5).expect("closed profile edge should split");
    let profile = Profile::new(&g, profile_dart);

    assert!(profile.is_closed());
    assert_eq!(profile.edges().len(), 5);
}

fn edge_key_for_dart(g: &GMap<StandardPayload>, dart: ngk::topology::Dart) -> EdgeKey {
    g.cell_key::<ngk::topology::gmap::Cell1>(dart)
        .expect("edge key should exist for dart")
}
