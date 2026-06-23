use ngk::geometry::{Curve, Point3};
use ngk::topology::Dart;
use ngk::topology::attributes::{EdgeAttr, VertexAttr};
use ngk::topology::gmap::{
    Cell1, Dim, GMap, PreservePayload, TopologyCommitError, TopologyEditError,
    TopologyTransactionError,
};
use ngk::topology::payload::Payload;

#[derive(Clone, Default)]
struct TestPayload;

impl Payload for TestPayload {
    type V = ();
    type E = String;
    type Profile = ();
    type F = ();
    type Sheet = ();
    type S = ();
}

#[test]
fn failed_edit_closure_rolls_back_the_complete_map() {
    let mut g = GMap::<TestPayload>::new();
    let (first, second) = g
        .edit_preserving(|edit| {
            let first = edit.add_dart();
            let second = edit.add_dart();
            edit.add_vertex(VertexAttr::new(first, Point3::origin(), ()));
            Ok::<_, TopologyEditError>((first, second))
        })
        .unwrap();
    let original_dart_count = g.dart_count();

    let result = g.edit_preserving(|edit| {
        let added = edit.add_dart();
        edit.link(Dim::Zero, first, added)?;
        assert_eq!(edit.dart_count(), original_dart_count + 1);
        Err::<(), _>(TopologyEditError::SameDart { dart: added })
    });

    assert!(matches!(
        result,
        Err(TopologyTransactionError::Operation(_))
    ));
    assert_eq!(g.dart_count(), original_dart_count);
    assert!(g.is_free(first, Dim::Zero));
    assert_eq!(second.id(), 1);
}

#[test]
fn committing_topology_edit_reindexes_cells_after_their_orbits_change() {
    let mut g = GMap::<TestPayload>::new();
    let (first, second) = g
        .edit_preserving(|edit| {
            let first = edit.add_dart();
            let first_end = edit.add_dart();
            let second = edit.add_dart();
            let second_end = edit.add_dart();
            edit.link(Dim::Zero, first, first_end).unwrap();
            edit.link(Dim::Zero, second, second_end).unwrap();
            edit.add_edge(EdgeAttr::new(
                first,
                Curve::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0)),
                "first".to_owned(),
            ));
            edit.add_edge(EdgeAttr::new(
                second,
                Curve::line(Point3::new(1.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.0)),
                "second".to_owned(),
            ));
            edit.sew(Dim::Two, first, second_end)
                .expect("matching edges should sew");
            Ok::<_, TopologyEditError>((first, second))
        })
        .expect("edit should commit");

    let edge = g
        .cell_key::<Cell1>(second)
        .expect("the merged edge should remain indexed");
    assert_eq!(g.cell_key::<Cell1>(first), Some(edge));
    assert_eq!(g.iter_edges().count(), 2);
}

#[test]
fn topology_edit_does_not_merge_edge_payloads_implicitly() {
    let mut g = GMap::<TestPayload>::new();
    let result = g.edit_preserving(|edit| {
        let first = edit.add_dart();
        let first_end = edit.add_dart();
        let second = edit.add_dart();
        let second_end = edit.add_dart();
        edit.link(Dim::Zero, first, first_end).unwrap();
        edit.link(Dim::Zero, second, second_end).unwrap();
        edit.add_edge(EdgeAttr::new(
            first,
            Curve::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0)),
            "left".to_owned(),
        ));
        edit.add_edge(EdgeAttr::new(
            second,
            Curve::line(Point3::new(1.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.0)),
            "right".to_owned(),
        ));
        edit.sew(Dim::Two, first, second_end).unwrap();
        Ok::<_, TopologyEditError>(())
    });
    result.expect("edit should commit");

    let payloads = g
        .iter_edges()
        .map(|(_, edge)| edge.data.as_str())
        .collect::<Vec<_>>();
    assert_eq!(payloads, ["left", "right"]);
}

#[test]
fn invalid_topology_commit_rolls_back_the_complete_map() {
    let mut g = GMap::<TestPayload>::new();
    let mut staged_darts = None;
    let result = g.edit(&mut PreservePayload, |edit| {
        let darts: [Dart; 4] = std::array::from_fn(|_| edit.add_dart());
        staged_darts = Some(darts);
        edit.link(Dim::Zero, darts[0], darts[1]).unwrap();
        edit.link(Dim::Zero, darts[2], darts[3]).unwrap();
        edit.link(Dim::Two, darts[0], darts[2]).unwrap();
        Ok::<_, TopologyEditError>(())
    });

    assert!(matches!(
        result,
        Err(TopologyTransactionError::Commit(
            TopologyCommitError::InvalidTopology(_)
        ))
    ));
    assert!(staged_darts.is_some());
    assert_eq!(g.dart_count(), 0);
}

#[test]
fn topology_edit_keeps_duplicate_domain_keys_until_explicit_merge_api_exists() {
    let mut g = GMap::<TestPayload>::new();
    let (first, second_end) = g
        .edit_preserving(|edit| {
            let first = edit.add_dart();
            let first_end = edit.add_dart();
            let second = edit.add_dart();
            let second_end = edit.add_dart();
            edit.link(Dim::Zero, first, first_end).unwrap();
            edit.link(Dim::Zero, second, second_end).unwrap();
            edit.add_edge(EdgeAttr::new(
                first,
                Curve::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0)),
                "left".to_owned(),
            ));
            edit.add_edge(EdgeAttr::new(
                second,
                Curve::line(Point3::new(1.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.0)),
                "right".to_owned(),
            ));
            Ok::<_, TopologyEditError>((first, second_end))
        })
        .unwrap();

    let result = g.edit_preserving(|edit| {
        edit.sew(Dim::Two, first, second_end).unwrap();
        Ok::<_, TopologyEditError>(())
    });

    result.expect("edit should commit without implicit payload policy callbacks");
    assert_eq!(g.iter_edges().count(), 2);
    assert!(!g.is_free(first, Dim::Two));
    assert!(!g.is_free(second_end, Dim::Two));
}

#[test]
fn topology_edit_does_not_infer_split_payloads() {
    let mut g = GMap::<TestPayload>::new();
    let (start, end, source) = g
        .edit_preserving(|edit| {
            let start = edit.add_dart();
            let end = edit.add_dart();
            edit.link(Dim::Zero, start, end).unwrap();
            let source = edit.add_edge(EdgeAttr::new(
                start,
                Curve::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0)),
                "source".to_owned(),
            ));
            Ok::<_, TopologyEditError>((start, end, source))
        })
        .unwrap();

    let created = g
        .edit_preserving(|edit| {
            let first_mid = edit.add_dart();
            let second_mid = edit.add_dart();
            edit.unlink(Dim::Zero, start).unwrap();
            edit.link(Dim::Zero, start, first_mid).unwrap();
            edit.link(Dim::Zero, second_mid, end).unwrap();
            edit.link(Dim::One, first_mid, second_mid).unwrap();
            let created = edit.add_edge(EdgeAttr::new(
                second_mid,
                Curve::line(Point3::new(0.5, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)),
                "builder".to_owned(),
            ));
            Ok::<_, TopologyEditError>(created)
        })
        .unwrap();

    assert_eq!(g.edge_attr_unchecked(source).data, "source");
    assert_eq!(g.edge_attr_unchecked(created).data, "builder");
}
