use ngk::builders::edges::{EdgeSplit, EdgeSplitError, add_line, split_edge};
use ngk::builders::faces::{add_circle as add_disc, split_face_edge};
use ngk::builders::profiles::{add_polyline, add_rectangle};
use ngk::geometry::{LINEAR_TOLERANCE, Plane, Point3, PointCoincidence};
use ngk::model::Model;
use ngk::modeling::faces;
use ngk::topology::ModelEditError;
use ngk::topology::closed::Closeable;
use ngk::topology::edge::Edge;
use ngk::topology::gmap::Dim;
use ngk::topology::payload::{Payload, StandardPayload};
use ngk::topology::profile::Profile;
use ngk::topology::shape_keys::EdgeKey;
use ngk::topology::shape_keys::FaceKey;

/// Returns the single rim edge of a disc.
fn rim_of(g: &Model<StandardPayload>, face: FaceKey) -> EdgeKey {
    g.face_unchecked(face)
        .edges()
        .first()
        .expect("a disc has a rim")
        .key()
}

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
    let mut g = Model::<StandardPayload>::new();
    let start = Point3::new(0.0, 0.0, 0.0);
    let end = Point3::new(1.0, 0.0, 0.0);
    let edge = add_line(&mut g, start, end).expect("line edge should build");

    let split = split_edge(&mut g, edge, 0.25).expect("isolated edge should split");

    assert_eq!(g.iter_edges().count(), 2);
    assert_eq!(g.iter_vertices().count(), 3);

    let midpoint = Point3::new(0.25, 0.0, 0.0);
    let split_vertex = g.vertex_attr_unchecked(split.vertex()).vertex(&g);
    assert!(
        split_vertex
            .point()
            .unwrap()
            .coincides(midpoint, LINEAR_TOLERANCE)
    );

    let EdgeSplit::Separated { first, second, .. } = split else {
        panic!("cutting a bounded edge separates it, got {split:?}");
    };
    let first = g.edge_unchecked(first);
    let second = g.edge_unchecked(second);
    assert!(
        first
            .bounded_unchecked()
            .start()
            .point()
            .unwrap()
            .coincides(start, LINEAR_TOLERANCE)
    );
    assert!(
        first
            .bounded_unchecked()
            .end()
            .point()
            .unwrap()
            .coincides(midpoint, LINEAR_TOLERANCE)
    );

    assert!(
        second
            .bounded_unchecked()
            .start()
            .point()
            .expect("second split edge start should have geometry")
            .coincides(midpoint, LINEAR_TOLERANCE),
        "the second split edge should start at the split point"
    );
    assert!(
        second
            .bounded_unchecked()
            .end()
            .point()
            .expect("second split edge end should have geometry")
            .coincides(end, LINEAR_TOLERANCE),
        "the second split edge should preserve the original end point"
    );
}

#[test]
fn split_isolated_edge_keeps_edge_profile_free() {
    let mut g = Model::<StandardPayload>::new();
    let edge = add_line(
        &mut g,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
    )
    .expect("line edge should build");

    let split = split_edge(&mut g, edge, 0.5).expect("isolated edge should split");

    for key in split.edges() {
        assert!(Profile::from_dart(&g, g.edge_attr_unchecked(key).dart).is_none());
    }
}

#[test]
fn split_edge_initializes_split_edge_payload_from_source() {
    let mut g = Model::<EdgePayload>::new();
    let edge = add_line(
        &mut g,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
    )
    .expect("line edge should build");
    g.transaction(|edit| {
        edit.edge_attr_mut_unchecked(edge).data = "source".to_owned();
        Ok::<_, ModelEditError>(())
    })
    .unwrap();

    let split = split_edge(&mut g, edge, 0.5).expect("edge should split");

    for key in split.edges() {
        assert_eq!(g.edge_attr_unchecked(key).data, "source");
    }
}

#[test]
fn split_profile_edge_rejects_boundary_parameters() {
    let mut g = Model::<StandardPayload>::new();
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
        .model()
        .iter_edges()
        .next()
        .expect("rectangle should contain edges")
        .0;

    assert!(matches!(
        split_edge(face.model_mut(), edge, 0.5),
        Err(EdgeSplitError::EdgeBelongsToFace { .. })
    ));
}

#[test]
fn split_profile_edge_preserves_open_profile_order() {
    let mut g = Model::<StandardPayload>::new();
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
        g.vertex_attr_unchecked(split.vertex())
            .point
            .coincides(midpoint, LINEAR_TOLERANCE)
    );
}

#[test]
fn split_profile_edge_preserves_closed_profile() {
    let mut g = Model::<StandardPayload>::new();
    let profile_key = add_rectangle(&mut g, ngk::geometry::Plane::xy(), 1.0, 1.0)
        .expect("rectangle profile should build");
    let first_edge_dart = g.profile_unchecked(profile_key).edges()[0].dart();
    let first_edge = edge_key_for_dart(&g, first_edge_dart);

    split_edge(&mut g, first_edge, 0.5).expect("closed profile edge should split");
    let profile = g.profile_unchecked(profile_key);

    assert!(profile.is_closed());
    assert_eq!(profile.edges().len(), 5);
}

fn edge_key_for_dart(g: &Model<StandardPayload>, dart: ngk::topology::Dart) -> EdgeKey {
    g.cell_key_unchecked::<ngk::model::Cell1>(dart)
}

/// Cutting an unmarked edge marks it, and creates no edge.
///
/// An unmarked circle already holds the 0-cell its corner would sit on -- its
/// two ends meet there -- so the cut materializes that cell rather than adding
/// anything. One cut does not separate a circle: there is nothing yet to
/// separate it from.
#[test]
fn cutting_an_unmarked_edge_marks_it() {
    let mut g = Model::<StandardPayload>::new();
    let face = add_disc(&mut g, Plane::xy(), 1.0).expect("a disc builds");
    let rim = rim_of(&g, face);
    let darts = g.cells(Dim::One).count();

    let split = split_face_edge(&mut g, face, rim, 1.0).expect("the rim takes a corner");

    let EdgeSplit::Marked { edge, vertex } = split else {
        panic!("cutting an unmarked edge marks it, got {split:?}");
    };
    assert_eq!(edge, rim, "the edge keeps the key it had");
    assert_eq!(g.iter_edges().count(), 1, "nothing was created");
    assert_eq!(g.iter_vertices().count(), 1, "the corner asked for");
    assert_eq!(
        g.cells(Dim::One).count(),
        darts,
        "marking is a relabel, so the raw map gains no 1-cell",
    );

    let view = g.edge_unchecked(edge);
    assert!(
        matches!(view, Edge::Closed(_)),
        "a closed edge with a deliberate corner is still closed",
    );
    assert_eq!(
        view.closed()
            .and_then(|closed| closed.vertex())
            .map(|v| v.key()),
        Some(vertex),
        "and the corner it carries is the one the cut asked for",
    );
}

/// A marked edge spans a whole period starting at its corner.
///
/// The corner is where the edge now begins and ends, so the span is derived
/// from it rather than from wherever the support's own domain happens to start.
#[test]
fn a_marked_edge_spans_a_period_from_its_corner() {
    let mut g = Model::<StandardPayload>::new();
    let face = add_disc(&mut g, Plane::xy(), 1.0).expect("a disc builds");
    let rim = rim_of(&g, face);

    split_face_edge(&mut g, face, rim, 1.0).expect("the rim takes a corner");

    let span = g
        .edge_unchecked(rim)
        .parameter_interval()
        .expect("a marked edge still spans")
        .ordered();
    assert!(
        (span.start - 1.0).abs() <= LINEAR_TOLERANCE,
        "the span begins at the corner, got {span:?}",
    );
    assert!(
        (span.end - span.start - std::f64::consts::TAU).abs() <= LINEAR_TOLERANCE,
        "and runs a whole period, got {span:?}",
    );
}

/// Cutting a marked edge separates it, which is the second cut on a circle.
#[test]
fn cutting_a_marked_edge_separates_it() {
    let mut g = Model::<StandardPayload>::new();
    let face = add_disc(&mut g, Plane::xy(), 1.0).expect("a disc builds");
    let rim = rim_of(&g, face);
    split_face_edge(&mut g, face, rim, 1.0).expect("the first cut marks");

    let split = split_face_edge(&mut g, face, rim, 3.0).expect("the second cut separates");

    let EdgeSplit::Separated { first, second, .. } = split else {
        panic!("cutting a marked edge separates it, got {split:?}");
    };
    assert_eq!(first, rim, "the original key stays on the first piece");
    assert_eq!(g.iter_edges().count(), 2, "two arcs where one circle was");
    assert_eq!(g.iter_vertices().count(), 2, "meeting at two corners");
    for key in [first, second] {
        assert!(
            matches!(g.edge_unchecked(key), Edge::Bounded(_)),
            "each arc runs between two distinct corners",
        );
    }
}
