use std::convert::Infallible;

use ngk::geometry::{Curve, Point3};
use ngk::topology::Dart;
use ngk::topology::attributes::{EdgeAttr, VertexAttr};
use ngk::topology::gmap::{Cell1, Dim, EditPolicy, GMap, PreservePayload, TopologyEditError};
use ngk::topology::payload::Payload;
use ngk::topology::shape_keys::EdgeKey;

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
        .edit(|edit| {
            let first = edit.add_dart();
            let second = edit.add_dart();
            edit.add_vertex(VertexAttr::new(first, Point3::origin(), ()));
            Ok((first, second))
        })
        .unwrap();
    let original_dart_count = g.dart_count();

    let result = g.edit(|edit| {
        let added = edit.add_dart();
        edit.link(Dim::Zero, first, added)?;
        assert_eq!(edit.dart_count(), original_dart_count + 1);
        Err::<(), _>(TopologyEditError::SameDart { dart: added })
    });

    assert!(matches!(result, Err(TopologyEditError::SameDart { .. })));
    assert_eq!(g.dart_count(), original_dart_count);
    assert!(g.is_free(first, Dim::Zero));
    assert_eq!(second.id(), 1);
}

#[test]
fn committing_topology_edit_reindexes_cells_after_explicit_merge() {
    let mut g = GMap::<TestPayload>::new();
    let (first, second) = g
        .edit(|edit| {
            let first = edit.add_dart();
            let first_end = edit.add_dart();
            let second = edit.add_dart();
            let second_end = edit.add_dart();
            edit.link(Dim::Zero, first, first_end).unwrap();
            edit.link(Dim::Zero, second, second_end).unwrap();
            let first_edge = edit.add_edge(EdgeAttr::new(
                first,
                Curve::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0)),
                "first".to_owned(),
            ));
            let second_edge = edit.add_edge(EdgeAttr::new(
                second,
                Curve::line(Point3::new(1.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.0)),
                "second".to_owned(),
            ));
            edit.sew(Dim::Two, first, second_end)
                .expect("matching edges should sew");
            edit.merge_edges_into(first_edge, second_edge);
            Ok((first, second))
        })
        .expect("edit should commit");

    let edge = g
        .cell_key::<Cell1>(second)
        .expect("the merged edge should remain indexed");
    assert_eq!(g.cell_key::<Cell1>(first), Some(edge));
    assert_eq!(g.iter_edges().count(), 1);
}

#[test]
fn topology_edit_rejects_duplicate_edge_keys_without_explicit_merge() {
    let mut g = GMap::<TestPayload>::new();
    let result = g.edit(|edit| {
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
        Ok(())
    });
    assert!(matches!(
        result,
        Err(TopologyEditError::DuplicateCellAttribute { entity: "edge", .. })
    ));
    assert_eq!(g.dart_count(), 0);
}

#[test]
fn invalid_topology_commit_rolls_back_the_complete_map() {
    let mut g = GMap::<TestPayload>::new();
    let mut staged_darts = None;
    let result = g.edit_with_policy(&mut PreservePayload, |edit| {
        let darts: [Dart; 4] = std::array::from_fn(|_| edit.add_dart());
        staged_darts = Some(darts);
        edit.link(Dim::Zero, darts[0], darts[1]).unwrap();
        edit.link(Dim::Zero, darts[2], darts[3]).unwrap();
        edit.link(Dim::Two, darts[0], darts[2]).unwrap();
        Ok(())
    });

    assert!(matches!(result, Err(TopologyEditError::InvalidTopology(_))));
    assert!(staged_darts.is_some());
    assert_eq!(g.dart_count(), 0);
}

#[test]
fn explicit_edge_merge_uses_the_policy_and_removes_the_consumed_key() {
    let mut g = GMap::<TestPayload>::new();
    let (first, second_end, first_edge, second_edge) = g
        .edit(|edit| {
            let first = edit.add_dart();
            let first_end = edit.add_dart();
            let second = edit.add_dart();
            let second_end = edit.add_dart();
            edit.link(Dim::Zero, first, first_end).unwrap();
            edit.link(Dim::Zero, second, second_end).unwrap();
            let first_edge = edit.add_edge(EdgeAttr::new(
                first,
                Curve::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0)),
                "left".to_owned(),
            ));
            let second_edge = edit.add_edge(EdgeAttr::new(
                second,
                Curve::line(Point3::new(1.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.0)),
                "right".to_owned(),
            ));
            Ok((first, second_end, first_edge, second_edge))
        })
        .unwrap();

    let mut policy = JoinEdgeNames;
    let result = g.edit_with_policy(&mut policy, |edit| {
        edit.sew(Dim::Two, first, second_end).unwrap();
        edit.merge_edges_into(first_edge, second_edge);
        Ok(())
    });

    result.expect("edit should commit");
    let (_, edge) = g.iter_edges().next().expect("one edge should remain");
    assert_eq!(edge.data, "left+right");
    assert_eq!(g.iter_edges().count(), 1);
    assert!(!g.is_free(first, Dim::Two));
    assert!(!g.is_free(second_end, Dim::Two));
}

struct JoinEdgeNames;

impl EditPolicy<TestPayload> for JoinEdgeNames {
    type Error = Infallible;

    fn merge_edge_data(
        &mut self,
        _survivor: EdgeKey,
        survivor_data: &mut String,
        _removed: EdgeKey,
        removed_data: String,
    ) -> Result<(), Self::Error> {
        survivor_data.push('+');
        survivor_data.push_str(&removed_data);
        Ok(())
    }
}

#[test]
fn explicit_edge_split_uses_the_policy() {
    let mut g = GMap::<TestPayload>::new();
    let (start, end, source) = g
        .edit(|edit| {
            let start = edit.add_dart();
            let end = edit.add_dart();
            edit.link(Dim::Zero, start, end).unwrap();
            let source = edit.add_edge(EdgeAttr::new(
                start,
                Curve::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0)),
                "source".to_owned(),
            ));
            Ok((start, end, source))
        })
        .unwrap();

    let mut policy = MarkSplit;
    let created = g
        .edit_with_policy(&mut policy, |edit| {
            let first_mid = edit.add_dart();
            let second_mid = edit.add_dart();
            edit.unlink(Dim::Zero, start).unwrap();
            edit.link(Dim::Zero, start, first_mid).unwrap();
            edit.link(Dim::Zero, second_mid, end).unwrap();
            edit.link(Dim::One, first_mid, second_mid).unwrap();
            let created = edit.add_edge_split_from(
                source,
                EdgeAttr::new(
                    second_mid,
                    Curve::line(Point3::new(0.5, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)),
                    "builder".to_owned(),
                ),
            );
            Ok(created)
        })
        .unwrap();

    assert_eq!(g.edge_attr_unchecked(source).data, "source");
    assert_eq!(g.edge_attr_unchecked(created).data, "source:split");
}

struct MarkSplit;

impl EditPolicy<TestPayload> for MarkSplit {
    type Error = Infallible;

    fn split_edge_data(
        &mut self,
        _source: EdgeKey,
        source_data: &String,
        _created: EdgeKey,
        created_data: &mut String,
    ) -> Result<(), Self::Error> {
        *created_data = format!("{source_data}:split");
        Ok(())
    }
}
