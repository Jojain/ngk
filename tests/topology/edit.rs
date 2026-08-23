use std::convert::Infallible;

use ngk::geometry::{Curve, Plane, Point3, Surface};
use ngk::topology::Dart;
use ngk::topology::attributes::{EdgeAttr, FaceAttr, SolidAttr, VertexAttr};
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
fn failed_transaction_closure_rolls_back_the_complete_map() {
    let mut g = GMap::<TestPayload>::new();
    let (first, second) = g
        .transaction(|edit| {
            let first = edit.add_dart();
            let second = edit.add_dart();
            edit.add_vertex(VertexAttr::new(first, Point3::origin(), ()));
            Ok::<_, TopologyEditError>((first, second))
        })
        .unwrap();
    let original_dart_count = g.dart_count();

    let result = g.transaction(|edit| {
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
fn face_registration_requires_registered_boundary_profiles() {
    let mut g = GMap::<TestPayload>::new();
    let result = g.transaction(|edit| {
        let boundary = edit.add_dart();
        edit.add_face(FaceAttr::new(
            Surface::Plane(Plane::xy()),
            (),
            boundary,
            Vec::new(),
        ));
        Ok::<_, TopologyEditError>(())
    });

    assert!(matches!(
        result,
        Err(TopologyEditError::MissingProfileRegistration { .. })
    ));
    assert_eq!(g.dart_count(), 0);
    assert_eq!(g.iter_faces().count(), 0);
}

#[test]
fn solid_registration_requires_registered_shell_sheets() {
    let mut g = GMap::<TestPayload>::new();
    let result = g.transaction(|edit| {
        let shell = edit.add_dart();
        edit.add_solid(SolidAttr::new((), shell, None));
        Ok::<_, TopologyEditError>(())
    });

    assert!(matches!(
        result,
        Err(TopologyEditError::MissingSheetRegistration { .. })
    ));
    assert_eq!(g.dart_count(), 0);
    assert_eq!(g.iter_solids().count(), 0);
}

#[test]
fn committing_topology_edit_reindexes_cells_after_explicit_merge() {
    let mut g = GMap::<TestPayload>::new();
    let (first, second) = g
        .transaction(|edit| {
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
            Ok::<_, TopologyEditError>((first, second))
        })
        .expect("transaction should commit");

    let edge = g
        .cell_key::<Cell1>(second)
        .expect("the merged edge should remain indexed");
    assert_eq!(g.cell_key::<Cell1>(first), Some(edge));
    assert_eq!(g.iter_edges().count(), 1);
}

#[test]
fn topology_transaction_rejects_repeated_merge_consumption() {
    let mut g = GMap::<TestPayload>::new();
    let (first, second) = add_two_test_edges(&mut g);

    let result = g.transaction(|edit| {
        edit.merge_edges_into(first, second);
        edit.merge_edges_into(first, second);
        Ok(())
    });

    assert!(matches!(
        result,
        Err(TopologyEditError::RepeatedMerge { .. })
    ));
    assert_eq!(g.iter_edges().count(), 2);
}

#[test]
fn topology_transaction_rejects_merge_cycles() {
    let mut g = GMap::<TestPayload>::new();
    let (first, second) = add_two_test_edges(&mut g);

    let result = g.transaction(|edit| {
        edit.merge_edges_into(first, second);
        edit.merge_edges_into(second, first);
        Ok(())
    });

    assert!(matches!(result, Err(TopologyEditError::MergeCycle { .. })));
    assert_eq!(g.iter_edges().count(), 2);
}

/// Creates two independent attributed edges for lineage-validation tests.
fn add_two_test_edges(g: &mut GMap<TestPayload>) -> (EdgeKey, EdgeKey) {
    g.transaction(|edit| {
        let first_start = edit.add_dart();
        let first_end = edit.add_dart();
        let second_start = edit.add_dart();
        let second_end = edit.add_dart();
        edit.link(Dim::Zero, first_start, first_end)?;
        edit.link(Dim::Zero, second_start, second_end)?;
        let first = edit.add_edge(EdgeAttr::new(
            first_start,
            Curve::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0)),
            "first".to_owned(),
        ));
        let second = edit.add_edge(EdgeAttr::new(
            second_start,
            Curve::line(Point3::new(2.0, 0.0, 0.0), Point3::new(3.0, 0.0, 0.0)),
            "second".to_owned(),
        ));
        Ok::<_, TopologyEditError>((first, second))
    })
    .expect("independent edges should commit")
}

#[test]
fn invalid_topology_commit_rolls_back_the_complete_map() {
    let mut g = GMap::<TestPayload>::new();
    let mut staged_darts = None;
    let result = g.transaction_with_policy(&mut PreservePayload, |edit| {
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
        .transaction(|edit| {
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
            Ok::<_, TopologyEditError>((first, second_end, first_edge, second_edge))
        })
        .unwrap();

    let mut policy = JoinEdgeNames;
    let result: Result<(), TopologyEditError> = g.transaction_with_policy(&mut policy, |edit| {
        edit.sew(Dim::Two, first, second_end).unwrap();
        edit.merge_edges_into(first_edge, second_edge);
        Ok(())
    });

    result.expect("transaction should commit");
    let (_, edge) = g.iter_edges().next().expect("one edge should remain");
    assert_eq!(edge.data, "left+right");
    assert_eq!(g.iter_edges().count(), 1);
    assert!(!g.is_free(first, Dim::Two));
    assert!(!g.is_free(second_end, Dim::Two));
}

struct JoinEdgeNames;

impl EditPolicy<TestPayload> for JoinEdgeNames {
    type Error = Infallible;

    /// Makes the merge payload order visible in the surviving edge name.
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
        .transaction(|edit| {
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

    let mut policy = MarkSplit;
    let created = g
        .transaction_with_policy(&mut policy, |edit| {
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
            Ok::<_, TopologyEditError>(created)
        })
        .unwrap();

    assert_eq!(g.edge_attr_unchecked(source).data, "source");
    assert_eq!(g.edge_attr_unchecked(created).data, "source:split");
}

struct MarkSplit;

impl EditPolicy<TestPayload> for MarkSplit {
    type Error = Infallible;

    /// Replaces builder data with a value derived from the source snapshot.
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

#[derive(Default)]
struct RecordEdgePolicy {
    splits: Vec<(EdgeKey, String, EdgeKey)>,
    merges: Vec<(EdgeKey, EdgeKey, String)>,
}

impl EditPolicy<TestPayload> for RecordEdgePolicy {
    type Error = Infallible;

    /// Records split arguments so tests can inspect the resolved net lineage.
    fn split_edge_data(
        &mut self,
        source: EdgeKey,
        source_data: &String,
        created: EdgeKey,
        _created_data: &mut String,
    ) -> Result<(), Self::Error> {
        self.splits.push((source, source_data.clone(), created));
        Ok(())
    }

    /// Records merge arguments so tests can inspect final survivors and payloads.
    fn merge_edge_data(
        &mut self,
        survivor: EdgeKey,
        _survivor_data: &mut String,
        removed: EdgeKey,
        removed_data: String,
    ) -> Result<(), Self::Error> {
        self.merges.push((survivor, removed, removed_data));
        Ok(())
    }
}

#[test]
fn fresh_creation_followed_by_merge_does_not_call_policy() {
    let mut g = GMap::<TestPayload>::new();
    let mut policy = RecordEdgePolicy::default();

    g.transaction_with_policy(&mut policy, |edit| {
        let start = edit.add_dart();
        let end = edit.add_dart();
        edit.link(Dim::Zero, start, end)?;
        let survivor = edit.add_edge(EdgeAttr::new(
            start,
            Curve::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0)),
            "survivor".to_owned(),
        ));
        let removed = edit.add_edge(EdgeAttr::new(
            start,
            Curve::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0)),
            "temporary".to_owned(),
        ));
        edit.merge_edges_into(survivor, removed);
        Ok::<_, TopologyEditError>(())
    })
    .expect("the local merge should commit");

    assert!(policy.splits.is_empty());
    assert!(policy.merges.is_empty());
    assert_eq!(g.iter_edges().count(), 1);
}

#[test]
fn surviving_split_calls_policy_once() {
    let mut g = GMap::<TestPayload>::new();
    let (_, source) = add_named_test_edge(&mut g, 0.0, "source");
    let mut policy = RecordEdgePolicy::default();

    let created = g
        .transaction_with_policy(&mut policy, |edit| {
            let start = edit.add_dart();
            let end = edit.add_dart();
            edit.link(Dim::Zero, start, end)?;
            Ok::<_, TopologyEditError>(edit.add_edge_split_from(
                source,
                EdgeAttr::new(
                    start,
                    Curve::line(Point3::new(2.0, 0.0, 0.0), Point3::new(3.0, 0.0, 0.0)),
                    "created".to_owned(),
                ),
            ))
        })
        .expect("the split should commit");

    assert_eq!(policy.splits, vec![(source, "source".to_owned(), created)]);
    assert!(policy.merges.is_empty());
}

#[test]
fn transient_split_does_not_call_policy() {
    let mut g = GMap::<TestPayload>::new();
    let (dart, source) = add_named_test_edge(&mut g, 0.0, "source");
    let mut policy = RecordEdgePolicy::default();

    g.transaction_with_policy(&mut policy, |edit| {
        let created = edit.add_edge_split_from(
            source,
            EdgeAttr::new(
                dart,
                Curve::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0)),
                "temporary".to_owned(),
            ),
        );
        edit.merge_edges_into(source, created);
        Ok::<_, TopologyEditError>(())
    })
    .expect("the transient split should commit");

    assert!(policy.splits.is_empty());
    assert!(policy.merges.is_empty());
    assert_eq!(g.iter_edges().count(), 1);
}

#[test]
fn chained_merges_target_the_final_survivor_in_declaration_order() {
    let mut g = GMap::<TestPayload>::new();
    let (_, first) = add_named_test_edge(&mut g, 0.0, "first");
    let (_, second) = add_named_test_edge(&mut g, 2.0, "second");
    let (_, final_survivor) = add_named_test_edge(&mut g, 4.0, "final");
    let mut policy = RecordEdgePolicy::default();

    g.transaction_with_policy(&mut policy, |edit| {
        edit.merge_edges_into(first, second);
        edit.merge_edges_into(final_survivor, first);
        Ok::<_, TopologyEditError>(())
    })
    .expect("the merge chain should commit");

    assert_eq!(
        policy.merges,
        vec![
            (final_survivor, second, "second".to_owned()),
            (final_survivor, first, "first".to_owned()),
        ]
    );
    assert_eq!(g.iter_edges().count(), 1);
}

#[test]
fn policy_receives_transaction_start_source_and_removed_payloads() {
    let mut g = GMap::<TestPayload>::new();
    let (_, source) = add_named_test_edge(&mut g, 0.0, "source-start");
    let (_, survivor) = add_named_test_edge(&mut g, 2.0, "survivor-start");
    let (_, removed) = add_named_test_edge(&mut g, 4.0, "removed-start");
    let mut policy = RecordEdgePolicy::default();

    let created = g
        .transaction_with_policy(&mut policy, |edit| {
            edit.edge_attr_mut(source).unwrap().data = "source-staged".to_owned();
            edit.edge_attr_mut(removed).unwrap().data = "removed-staged".to_owned();
            let start = edit.add_dart();
            let end = edit.add_dart();
            edit.link(Dim::Zero, start, end)?;
            let created = edit.add_edge_split_from(
                source,
                EdgeAttr::new(
                    start,
                    Curve::line(Point3::new(6.0, 0.0, 0.0), Point3::new(7.0, 0.0, 0.0)),
                    "created".to_owned(),
                ),
            );
            edit.merge_edges_into(survivor, removed);
            Ok::<_, TopologyEditError>(created)
        })
        .expect("the transaction should commit");

    assert_eq!(
        policy.splits,
        vec![(source, "source-start".to_owned(), created)]
    );
    assert_eq!(
        policy.merges,
        vec![(survivor, removed, "removed-start".to_owned())]
    );
}

struct RejectEdgeSplit;

impl EditPolicy<TestPayload> for RejectEdgeSplit {
    type Error = std::io::Error;

    /// Forces policy application to fail after all structural commit work.
    fn split_edge_data(
        &mut self,
        _source: EdgeKey,
        _source_data: &String,
        _created: EdgeKey,
        _created_data: &mut String,
    ) -> Result<(), Self::Error> {
        Err(std::io::Error::other("split rejected"))
    }
}

#[test]
fn policy_failure_restores_topology_and_payloads() {
    let mut g = GMap::<TestPayload>::new();
    let (_, source) = add_named_test_edge(&mut g, 0.0, "source-start");
    let original_dart_count = g.dart_count();
    let mut policy = RejectEdgeSplit;

    let result = g.transaction_with_policy(&mut policy, |edit| {
        edit.edge_attr_mut(source).unwrap().data = "source-staged".to_owned();
        let start = edit.add_dart();
        let end = edit.add_dart();
        edit.link(Dim::Zero, start, end)?;
        edit.add_edge_split_from(
            source,
            EdgeAttr::new(
                start,
                Curve::line(Point3::new(2.0, 0.0, 0.0), Point3::new(3.0, 0.0, 0.0)),
                "created".to_owned(),
            ),
        );
        Ok(())
    });

    assert!(matches!(result, Err(TopologyEditError::Policy(_))));
    assert_eq!(g.dart_count(), original_dart_count);
    assert_eq!(g.iter_edges().count(), 1);
    assert_eq!(g.edge_attr_unchecked(source).data, "source-start");
}

/// Creates a positioned edge whose payload makes reconciliation choices observable.
fn add_named_test_edge(g: &mut GMap<TestPayload>, start_x: f64, data: &str) -> (Dart, EdgeKey) {
    g.transaction(|edit| {
        let start = edit.add_dart();
        let end = edit.add_dart();
        edit.link(Dim::Zero, start, end)?;
        let key = edit.add_edge(EdgeAttr::new(
            start,
            Curve::line(
                Point3::new(start_x, 0.0, 0.0),
                Point3::new(start_x + 1.0, 0.0, 0.0),
            ),
            data.to_owned(),
        ));
        Ok::<_, TopologyEditError>((start, key))
    })
    .expect("the edge should commit")
}

#[test]
fn local_local_collision_keeps_the_earliest_created_key() {
    let mut g = GMap::<TestPayload>::new();

    let (earliest, later) = g
        .transaction(|edit| {
            let start = edit.add_dart();
            let end = edit.add_dart();
            edit.link(Dim::Zero, start, end)?;
            let earliest = edit.add_edge(EdgeAttr::new(
                start,
                Curve::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0)),
                "earliest".to_owned(),
            ));
            let later = edit.add_edge(EdgeAttr::new(
                start,
                Curve::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0)),
                "later".to_owned(),
            ));
            Ok::<_, TopologyEditError>((earliest, later))
        })
        .expect("local identities should reconcile");

    assert!(g.edge_attr(earliest).is_some());
    assert!(g.edge_attr(later).is_none());
    assert_eq!(g.iter_edges().count(), 1);
}

#[test]
fn local_existing_collision_keeps_the_existing_key() {
    let mut g = GMap::<TestPayload>::new();
    let (dart, existing) = add_named_test_edge(&mut g, 0.0, "existing");

    let local = g
        .transaction(|edit| {
            Ok::<_, TopologyEditError>(edit.add_edge(EdgeAttr::new(
                dart,
                Curve::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0)),
                "local".to_owned(),
            )))
        })
        .expect("the existing identity should win");

    assert!(g.edge_attr(existing).is_some());
    assert!(g.edge_attr(local).is_none());
    assert_eq!(g.iter_edges().count(), 1);
}

#[test]
fn multiple_existing_identities_require_explicit_lineage() {
    let mut g = GMap::<TestPayload>::new();
    let (first_dart, first) = add_named_test_edge(&mut g, 0.0, "first");
    let (second_dart, second) = add_named_test_edge(&mut g, 1.0, "second");
    let second_end = g.alpha(Dim::Zero, second_dart);

    let result = g.transaction(|edit| edit.sew(Dim::Two, first_dart, second_end));

    assert!(matches!(
        result,
        Err(TopologyEditError::UnresolvedPreExistingCollision { entity: "edge", .. })
    ));
    assert!(g.edge_attr(first).is_some());
    assert!(g.edge_attr(second).is_some());
    assert!(g.is_free(first_dart, Dim::Two));
}

#[test]
fn explicit_existing_collision_keeps_declared_survivor_and_calls_policy_once() {
    let mut g = GMap::<TestPayload>::new();
    let (survivor_dart, survivor) = add_named_test_edge(&mut g, 0.0, "survivor");
    let (removed_dart, removed) = add_named_test_edge(&mut g, 1.0, "removed");
    let removed_end = g.alpha(Dim::Zero, removed_dart);
    let mut policy = RecordEdgePolicy::default();

    g.transaction_with_policy(&mut policy, |edit| {
        edit.sew(Dim::Two, survivor_dart, removed_end)?;
        edit.merge_edges_into(survivor, removed);
        Ok::<_, TopologyEditError>(())
    })
    .expect("explicit lineage should select the survivor");

    assert!(g.edge_attr(survivor).is_some());
    assert!(g.edge_attr(removed).is_none());
    assert_eq!(
        policy.merges,
        vec![(survivor, removed, "removed".to_owned())]
    );
    assert_eq!(g.iter_edges().count(), 1);
}

#[test]
fn explicit_lineage_survivor_must_survive_reconciliation() {
    let mut g = GMap::<TestPayload>::new();
    let (existing_dart, existing) = add_named_test_edge(&mut g, 0.0, "existing");
    let (_, removed) = add_named_test_edge(&mut g, 2.0, "removed");

    let result = g.transaction(|edit| {
        let local_survivor = edit.add_edge(EdgeAttr::new(
            existing_dart,
            Curve::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0)),
            "local".to_owned(),
        ));
        edit.merge_edges_into(local_survivor, removed);
        Ok(local_survivor)
    });

    assert!(matches!(
        result,
        Err(TopologyEditError::InvalidLineageSurvivor { .. })
    ));
    assert!(g.edge_attr(existing).is_some());
    assert!(g.edge_attr(removed).is_some());
    assert_eq!(g.iter_edges().count(), 2);
}

#[test]
fn split_discarded_by_reconciliation_does_not_call_policy() {
    let mut g = GMap::<TestPayload>::new();
    let (_, source) = add_named_test_edge(&mut g, 0.0, "source");
    let (existing_dart, existing) = add_named_test_edge(&mut g, 2.0, "existing");
    let mut policy = RecordEdgePolicy::default();

    let created = g
        .transaction_with_policy(&mut policy, |edit| {
            Ok::<_, TopologyEditError>(edit.add_edge_split_from(
                source,
                EdgeAttr::new(
                    existing_dart,
                    Curve::line(Point3::new(2.0, 0.0, 0.0), Point3::new(3.0, 0.0, 0.0)),
                    "split".to_owned(),
                ),
            ))
        })
        .expect("the existing identity should absorb the local split identity");

    assert!(g.edge_attr(existing).is_some());
    assert!(g.edge_attr(created).is_none());
    assert!(policy.splits.is_empty());
}
