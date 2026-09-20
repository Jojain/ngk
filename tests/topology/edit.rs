use std::convert::Infallible;

use ngk::geometry::{Curve, Frame, Plane, Point2, Point3, Surface, TrimmedCurve2};
use ngk::model::{Cell1, Model};
use ngk::topology::Dart;
use ngk::topology::attributes::{EdgeAttr, FaceAttr, SolidAttr, VertexAttr};
use ngk::topology::edit::{EditKey, EditPolicy, ModelEditError, Origin, PreservePayload};
use ngk::topology::gmap::Dim;
use ngk::topology::payload::Payload;
use ngk::topology::shape_keys::{EdgeKey, FaceKey, ProfileKey, SheetKey, SolidKey, VertexKey};
use ngk::topology::validation::CellOccupancyError;

#[derive(Clone, Default)]
struct TestPayload;

impl Payload for TestPayload {
    type V = ();
    type E = String;
    type Profile = ();
    type F = ();
    type Sheet = ();
    type S = ();

    type Policy = PreservePayload;
}

#[test]
fn failed_transaction_closure_rolls_back_the_complete_map() {
    let mut g = Model::<TestPayload>::new();
    let (first, second) = g
        .transaction(|edit| {
            let first = edit.add_dart();
            let second = edit.add_dart();
            edit.add_vertex(VertexAttr::new(first, Point3::origin()));
            Ok::<_, ModelEditError>((first, second))
        })
        .unwrap();
    let original_dart_count = g.dart_count();

    let result = g.transaction(|edit| {
        let added = edit.add_dart();
        edit.link(Dim::Zero, first, added)?;
        assert_eq!(edit.dart_count(), original_dart_count + 1);
        Err::<(), _>(ModelEditError::SameDart { dart: added })
    });

    assert!(matches!(result, Err(ModelEditError::SameDart { .. })));
    assert_eq!(g.dart_count(), original_dart_count);
    assert!(g.is_free(first, Dim::Zero));
    assert_eq!(second.id(), 1);
}

#[test]
fn face_registration_requires_registered_boundary_profiles() {
    let mut g = Model::<TestPayload>::new();
    let result = g.transaction(|edit| {
        let boundary = edit.add_dart();
        edit.add_face(FaceAttr::new(
            Surface::Plane(Plane::xy()),
            boundary,
            Vec::new(),
        ));
        Ok::<_, ModelEditError>(())
    });

    assert!(matches!(
        result,
        Err(ModelEditError::MissingProfileRegistration { .. })
    ));
    assert_eq!(g.dart_count(), 0);
    assert_eq!(g.iter_faces().count(), 0);
}

#[test]
fn solid_registration_requires_registered_shell_sheets() {
    let mut g = Model::<TestPayload>::new();
    let result = g.transaction(|edit| {
        let shell = edit.add_dart();
        edit.add_solid(SolidAttr::new(shell, None));
        Ok::<_, ModelEditError>(())
    });

    assert!(matches!(
        result,
        Err(ModelEditError::MissingSheetRegistration { .. })
    ));
    assert_eq!(g.dart_count(), 0);
    assert_eq!(g.iter_solids().count(), 0);
}

#[test]
fn commit_rejects_and_rolls_back_a_face_spanning_two_raw_cells() {
    let mut g = Model::<TestPayload>::new();
    let solid = ngk::builders::solids::add_sphere(&mut g, Frame::xyz(), 1.0)
        .expect("a sphere should build")
        .solid;
    let face = g.solid_unchecked(solid).faces()[0].key();
    let before_darts = g.dart_count();

    let result = g.transaction(|edit| {
        let foreign = edit.add_dart();
        edit.face_attr_mut_unchecked(face).pcurves.insert(
            foreign,
            TrimmedCurve2::segment(Point2::origin(), Point2::new(1.0, 0.0)),
        );
        Ok::<_, ModelEditError>(())
    });

    assert!(matches!(
        result,
        Err(ModelEditError::InvalidCellOccupancy(
            CellOccupancyError::SpansSeveralCells { .. }
        ))
    ));
    assert_eq!(g.dart_count(), before_darts);
    assert!(g.face_attr_unchecked(face).pcurves.is_empty());
}

#[test]
fn committing_topology_edit_reindexes_cells_after_explicit_merge() {
    let mut g = Model::<TestPayload>::new();
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
            ));
            let second_edge = edit.add_edge(EdgeAttr::new(
                second,
                Curve::line(Point3::new(1.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.0)),
            ));
            edit.sew(Dim::Two, first, second_end)
                .expect("matching edges should sew");
            edit.merge_edges_into(first_edge, second_edge);
            Ok::<_, ModelEditError>((first, second))
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
    let mut g = Model::<TestPayload>::new();
    let (first, second) = add_two_test_edges(&mut g);

    let result = g.transaction(|edit| {
        edit.merge_edges_into(first, second);
        edit.merge_edges_into(first, second);
        Ok(())
    });

    assert!(matches!(result, Err(ModelEditError::RepeatedMerge { .. })));
    assert_eq!(g.iter_edges().count(), 2);
}

#[test]
fn topology_transaction_rejects_merge_cycles() {
    let mut g = Model::<TestPayload>::new();
    let (first, second) = add_two_test_edges(&mut g);

    let result = g.transaction(|edit| {
        edit.merge_edges_into(first, second);
        edit.merge_edges_into(second, first);
        Ok(())
    });

    assert!(matches!(result, Err(ModelEditError::MergeCycle { .. })));
    assert_eq!(g.iter_edges().count(), 2);
}

/// Creates two independent attributed edges for lineage-validation tests.
fn add_two_test_edges(g: &mut Model<TestPayload>) -> (EdgeKey, EdgeKey) {
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
        ));
        let second = edit.add_edge(EdgeAttr::new(
            second_start,
            Curve::line(Point3::new(2.0, 0.0, 0.0), Point3::new(3.0, 0.0, 0.0)),
        ));
        Ok::<_, ModelEditError>((first, second))
    })
    .expect("independent edges should commit")
}

#[test]
fn invalid_topology_commit_rolls_back_the_complete_map() {
    let mut g = Model::<TestPayload>::new();
    let mut staged_darts = None;
    let result = g.transaction_with_policy(&mut PreservePayload, |edit| {
        let darts: [Dart; 4] = std::array::from_fn(|_| edit.add_dart());
        staged_darts = Some(darts);
        edit.link(Dim::Zero, darts[0], darts[1]).unwrap();
        edit.link(Dim::Zero, darts[2], darts[3]).unwrap();
        edit.link(Dim::Two, darts[0], darts[2]).unwrap();
        Ok(())
    });

    assert!(matches!(result, Err(ModelEditError::InvalidTopology(_))));
    assert!(staged_darts.is_some());
    assert_eq!(g.dart_count(), 0);
}

#[test]
fn explicit_edge_merge_uses_the_policy_and_removes_the_consumed_key() {
    let mut g = Model::<TestPayload>::new();
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
            ));
            let second_edge = edit.add_edge(EdgeAttr::new(
                second,
                Curve::line(Point3::new(1.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.0)),
            ));
            Ok::<_, ModelEditError>((first, second_end, first_edge, second_edge))
        })
        .unwrap();
    g.transaction(|edit| {
        *edit.edge_attr_mut_unchecked(first_edge).data_mut() = "left".to_owned();
        *edit.edge_attr_mut_unchecked(second_edge).data_mut() = "right".to_owned();
        Ok::<_, ModelEditError>(())
    })
    .unwrap();

    let mut policy = JoinEdgeNames;
    let result: Result<(), ModelEditError> = g.transaction_with_policy(&mut policy, |edit| {
        edit.sew(Dim::Two, first, second_end).unwrap();
        edit.merge_edges_into(first_edge, second_edge);
        Ok(())
    });

    result.expect("transaction should commit");
    let (_, edge) = g.iter_edges().next().expect("one edge should remain");
    assert_eq!(edge.data(), "left+right");
    assert_eq!(g.iter_edges().count(), 1);
    assert!(!g.is_free(first, Dim::Two));
    assert!(!g.is_free(second_end, Dim::Two));
}

struct JoinEdgeNames;

impl EditPolicy<TestPayload> for JoinEdgeNames {
    type Error = Infallible;

    fn vertex_created(
        &mut self,
        _: VertexKey,
        _: Origin,
        _: &Model<TestPayload>,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    fn edge_created(
        &mut self,
        _: EdgeKey,
        _: Origin,
        _: &Model<TestPayload>,
    ) -> Result<String, Self::Error> {
        Ok(String::new())
    }

    fn profile_created(
        &mut self,
        _: ProfileKey,
        _: Origin,
        _: &Model<TestPayload>,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    fn face_created(
        &mut self,
        _: FaceKey,
        _: Origin,
        _: &Model<TestPayload>,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    fn sheet_created(
        &mut self,
        _: SheetKey,
        _: Origin,
        _: &Model<TestPayload>,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    fn solid_created(
        &mut self,
        _: SolidKey,
        _: Origin,
        _: &Model<TestPayload>,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    /// Makes the merge payload order visible in the surviving edge name.
    fn edge_merged(
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
    let mut g = Model::<TestPayload>::new();
    let (start, end, source) = g
        .transaction(|edit| {
            let start = edit.add_dart();
            let end = edit.add_dart();
            edit.link(Dim::Zero, start, end).unwrap();
            let source = edit.add_edge(EdgeAttr::new(
                start,
                Curve::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0)),
            ));
            Ok::<_, ModelEditError>((start, end, source))
        })
        .unwrap();
    g.transaction(|edit| {
        *edit.edge_attr_mut_unchecked(source).data_mut() = "source".to_owned();
        Ok::<_, ModelEditError>(())
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
                ),
            );
            Ok::<_, ModelEditError>(created)
        })
        .unwrap();

    assert_eq!(g.edge_attr_unchecked(source).data(), "source");
    assert_eq!(g.edge_attr_unchecked(created).data(), "source:split");
}

struct MarkSplit;

impl EditPolicy<TestPayload> for MarkSplit {
    type Error = Infallible;

    fn vertex_created(
        &mut self,
        _: VertexKey,
        _: Origin,
        _: &Model<TestPayload>,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    /// Replaces builder data with a value derived from the source snapshot.
    fn edge_created(
        &mut self,
        _key: EdgeKey,
        origin: Origin,
        before: &Model<TestPayload>,
    ) -> Result<String, Self::Error> {
        match origin {
            Origin::Split(EditKey::Edge(source)) => Ok(format!(
                "{}:split",
                before.edge_attr_unchecked(source).data()
            )),
            _ => Ok(String::new()),
        }
    }

    fn profile_created(
        &mut self,
        _: ProfileKey,
        _: Origin,
        _: &Model<TestPayload>,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    fn face_created(
        &mut self,
        _: FaceKey,
        _: Origin,
        _: &Model<TestPayload>,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    fn sheet_created(
        &mut self,
        _: SheetKey,
        _: Origin,
        _: &Model<TestPayload>,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    fn solid_created(
        &mut self,
        _: SolidKey,
        _: Origin,
        _: &Model<TestPayload>,
    ) -> Result<(), Self::Error> {
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

    fn vertex_created(
        &mut self,
        _: VertexKey,
        _: Origin,
        _: &Model<TestPayload>,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    /// Records split arguments so tests can inspect the resolved net lineage.
    fn edge_created(
        &mut self,
        created: EdgeKey,
        origin: Origin,
        before: &Model<TestPayload>,
    ) -> Result<String, Self::Error> {
        match origin {
            Origin::Split(EditKey::Edge(source)) => {
                let source_data = before.edge_attr_unchecked(source).data().clone();
                self.splits.push((source, source_data.clone(), created));
                Ok(source_data)
            }
            _ => Ok(String::new()),
        }
    }

    fn profile_created(
        &mut self,
        _: ProfileKey,
        _: Origin,
        _: &Model<TestPayload>,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    fn face_created(
        &mut self,
        _: FaceKey,
        _: Origin,
        _: &Model<TestPayload>,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    fn sheet_created(
        &mut self,
        _: SheetKey,
        _: Origin,
        _: &Model<TestPayload>,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    fn solid_created(
        &mut self,
        _: SolidKey,
        _: Origin,
        _: &Model<TestPayload>,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    /// Records merge arguments so tests can inspect final survivors and payloads.
    fn edge_merged(
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
    let mut g = Model::<TestPayload>::new();
    let mut policy = RecordEdgePolicy::default();

    g.transaction_with_policy(&mut policy, |edit| {
        let start = edit.add_dart();
        let end = edit.add_dart();
        edit.link(Dim::Zero, start, end)?;
        let survivor = edit.add_edge(EdgeAttr::new(
            start,
            Curve::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0)),
        ));
        let removed = edit.add_edge(EdgeAttr::new(
            start,
            Curve::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0)),
        ));
        edit.merge_edges_into(survivor, removed);
        Ok::<_, ModelEditError>(())
    })
    .expect("the local merge should commit");

    assert!(policy.splits.is_empty());
    assert!(policy.merges.is_empty());
    assert_eq!(g.iter_edges().count(), 1);
}

#[test]
fn surviving_split_calls_policy_once() {
    let mut g = Model::<TestPayload>::new();
    let (_, source) = add_named_test_edge(&mut g, 0.0, "source");
    let mut policy = RecordEdgePolicy::default();

    let created = g
        .transaction_with_policy(&mut policy, |edit| {
            let start = edit.add_dart();
            let end = edit.add_dart();
            edit.link(Dim::Zero, start, end)?;
            Ok::<_, ModelEditError>(edit.add_edge_split_from(
                source,
                EdgeAttr::new(
                    start,
                    Curve::line(Point3::new(2.0, 0.0, 0.0), Point3::new(3.0, 0.0, 0.0)),
                ),
            ))
        })
        .expect("the split should commit");

    assert_eq!(policy.splits, vec![(source, "source".to_owned(), created)]);
    assert!(policy.merges.is_empty());
}

#[test]
fn transient_split_does_not_call_policy() {
    let mut g = Model::<TestPayload>::new();
    let (dart, source) = add_named_test_edge(&mut g, 0.0, "source");
    let mut policy = RecordEdgePolicy::default();

    g.transaction_with_policy(&mut policy, |edit| {
        let created = edit.add_edge_split_from(
            source,
            EdgeAttr::new(
                dart,
                Curve::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0)),
            ),
        );
        edit.merge_edges_into(source, created);
        Ok::<_, ModelEditError>(())
    })
    .expect("the transient split should commit");

    assert!(policy.splits.is_empty());
    assert!(policy.merges.is_empty());
    assert_eq!(g.iter_edges().count(), 1);
}

#[test]
fn chained_merges_target_the_final_survivor_in_declaration_order() {
    let mut g = Model::<TestPayload>::new();
    let (_, first) = add_named_test_edge(&mut g, 0.0, "first");
    let (_, second) = add_named_test_edge(&mut g, 2.0, "second");
    let (_, final_survivor) = add_named_test_edge(&mut g, 4.0, "final");
    let mut policy = RecordEdgePolicy::default();

    g.transaction_with_policy(&mut policy, |edit| {
        edit.merge_edges_into(first, second);
        edit.merge_edges_into(final_survivor, first);
        Ok::<_, ModelEditError>(())
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
    let mut g = Model::<TestPayload>::new();
    let (_, source) = add_named_test_edge(&mut g, 0.0, "source-start");
    let (_, survivor) = add_named_test_edge(&mut g, 2.0, "survivor-start");
    let (_, removed) = add_named_test_edge(&mut g, 4.0, "removed-start");
    let mut policy = RecordEdgePolicy::default();

    let created = g
        .transaction_with_policy(&mut policy, |edit| {
            *edit.edge_attr_mut(source).unwrap().data_mut() = "source-staged".to_owned();
            *edit.edge_attr_mut(removed).unwrap().data_mut() = "removed-staged".to_owned();
            let start = edit.add_dart();
            let end = edit.add_dart();
            edit.link(Dim::Zero, start, end)?;
            let created = edit.add_edge_split_from(
                source,
                EdgeAttr::new(
                    start,
                    Curve::line(Point3::new(6.0, 0.0, 0.0), Point3::new(7.0, 0.0, 0.0)),
                ),
            );
            edit.merge_edges_into(survivor, removed);
            Ok::<_, ModelEditError>(created)
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

    fn vertex_created(
        &mut self,
        _: VertexKey,
        _: Origin,
        _: &Model<TestPayload>,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    /// Forces policy application to fail after all structural commit work.
    fn edge_created(
        &mut self,
        _key: EdgeKey,
        _origin: Origin,
        _before: &Model<TestPayload>,
    ) -> Result<String, Self::Error> {
        Err(std::io::Error::other("split rejected"))
    }

    fn profile_created(
        &mut self,
        _: ProfileKey,
        _: Origin,
        _: &Model<TestPayload>,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    fn face_created(
        &mut self,
        _: FaceKey,
        _: Origin,
        _: &Model<TestPayload>,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    fn sheet_created(
        &mut self,
        _: SheetKey,
        _: Origin,
        _: &Model<TestPayload>,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    fn solid_created(
        &mut self,
        _: SolidKey,
        _: Origin,
        _: &Model<TestPayload>,
    ) -> Result<(), Self::Error> {
        Ok(())
    }
}

#[test]
fn policy_failure_restores_topology_and_payloads() {
    let mut g = Model::<TestPayload>::new();
    let (_, source) = add_named_test_edge(&mut g, 0.0, "source-start");
    let original_dart_count = g.dart_count();
    let mut policy = RejectEdgeSplit;

    let result = g.transaction_with_policy(&mut policy, |edit| {
        *edit.edge_attr_mut(source).unwrap().data_mut() = "source-staged".to_owned();
        let start = edit.add_dart();
        let end = edit.add_dart();
        edit.link(Dim::Zero, start, end)?;
        edit.add_edge_split_from(
            source,
            EdgeAttr::new(
                start,
                Curve::line(Point3::new(2.0, 0.0, 0.0), Point3::new(3.0, 0.0, 0.0)),
            ),
        );
        Ok(())
    });

    assert!(matches!(result, Err(ModelEditError::Policy(_))));
    assert_eq!(g.dart_count(), original_dart_count);
    assert_eq!(g.iter_edges().count(), 1);
    assert_eq!(g.edge_attr_unchecked(source).data(), "source-start");
}

/// Creates a positioned edge whose payload makes reconciliation choices observable.
///
/// The name is written in a second transaction: a fresh creation's payload is
/// the default policy's to decide, so a plain `add_edge` under `PreservePayload`
/// defaults it regardless of what is passed to the constructor. Setting `data`
/// afterwards is a payload-only mutation, which reaches no hook.
fn add_named_test_edge(g: &mut Model<TestPayload>, start_x: f64, data: &str) -> (Dart, EdgeKey) {
    let (start, key) = g
        .transaction(|edit| {
            let start = edit.add_dart();
            let end = edit.add_dart();
            edit.link(Dim::Zero, start, end)?;
            let key = edit.add_edge(EdgeAttr::new(
                start,
                Curve::line(
                    Point3::new(start_x, 0.0, 0.0),
                    Point3::new(start_x + 1.0, 0.0, 0.0),
                ),
            ));
            Ok::<_, ModelEditError>((start, key))
        })
        .expect("the edge should commit");
    g.transaction(|edit| {
        *edit.edge_attr_mut_unchecked(key).data_mut() = data.to_owned();
        Ok::<_, ModelEditError>(())
    })
    .expect("naming the edge should commit");
    (start, key)
}

#[test]
fn local_local_collision_keeps_the_earliest_created_key() {
    let mut g = Model::<TestPayload>::new();

    let (earliest, later) = g
        .transaction(|edit| {
            let start = edit.add_dart();
            let end = edit.add_dart();
            edit.link(Dim::Zero, start, end)?;
            let earliest = edit.add_edge(EdgeAttr::new(
                start,
                Curve::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0)),
            ));
            let later = edit.add_edge(EdgeAttr::new(
                start,
                Curve::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0)),
            ));
            Ok::<_, ModelEditError>((earliest, later))
        })
        .expect("local identities should reconcile");

    assert!(g.edge_attr(earliest).is_some());
    assert!(g.edge_attr(later).is_none());
    assert_eq!(g.iter_edges().count(), 1);
}

#[test]
fn local_existing_collision_keeps_the_existing_key() {
    let mut g = Model::<TestPayload>::new();
    let (dart, existing) = add_named_test_edge(&mut g, 0.0, "existing");

    let local = g
        .transaction(|edit| {
            Ok::<_, ModelEditError>(edit.add_edge(EdgeAttr::new(
                dart,
                Curve::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0)),
            )))
        })
        .expect("the existing identity should win");

    assert!(g.edge_attr(existing).is_some());
    assert!(g.edge_attr(local).is_none());
    assert_eq!(g.iter_edges().count(), 1);
}

#[test]
fn multiple_existing_identities_require_explicit_lineage() {
    let mut g = Model::<TestPayload>::new();
    let (first_dart, first) = add_named_test_edge(&mut g, 0.0, "first");
    let (second_dart, second) = add_named_test_edge(&mut g, 1.0, "second");
    let second_end = g.alpha(Dim::Zero, second_dart);

    let result = g.transaction(|edit| edit.sew(Dim::Two, first_dart, second_end));

    assert!(matches!(
        result,
        Err(ModelEditError::UnresolvedPreExistingCollision { entity: "edge", .. })
    ));
    assert!(g.edge_attr(first).is_some());
    assert!(g.edge_attr(second).is_some());
    assert!(g.is_free(first_dart, Dim::Two));
}

#[test]
fn explicit_existing_collision_keeps_declared_survivor_and_calls_policy_once() {
    let mut g = Model::<TestPayload>::new();
    let (removed_dart, removed) = add_named_test_edge(&mut g, 0.0, "removed");
    let (survivor_dart, survivor) = add_named_test_edge(&mut g, 1.0, "survivor");
    let removed_end = g.alpha(Dim::Zero, removed_dart);
    let mut policy = RecordEdgePolicy::default();

    g.transaction_with_policy(&mut policy, |edit| {
        edit.sew(Dim::Two, survivor_dart, removed_end)?;
        edit.merge_edges_into(survivor, removed);
        assert_eq!(
            edit.cell_key::<Cell1>(removed_dart),
            Some(survivor),
            "staged lookup follows an explicit merge to its survivor"
        );
        Ok::<_, ModelEditError>(())
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
    let mut g = Model::<TestPayload>::new();
    let (existing_dart, existing) = add_named_test_edge(&mut g, 0.0, "existing");
    let (_, removed) = add_named_test_edge(&mut g, 2.0, "removed");

    let result = g.transaction(|edit| {
        let local_survivor = edit.add_edge(EdgeAttr::new(
            existing_dart,
            Curve::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0)),
        ));
        edit.merge_edges_into(local_survivor, removed);
        Ok(local_survivor)
    });

    assert!(matches!(
        result,
        Err(ModelEditError::InvalidLineageSurvivor { .. })
    ));
    assert!(g.edge_attr(existing).is_some());
    assert!(g.edge_attr(removed).is_some());
    assert_eq!(g.iter_edges().count(), 2);
}

#[test]
fn split_discarded_by_reconciliation_does_not_call_policy() {
    let mut g = Model::<TestPayload>::new();
    let (_, source) = add_named_test_edge(&mut g, 0.0, "source");
    let (existing_dart, existing) = add_named_test_edge(&mut g, 2.0, "existing");
    let mut policy = RecordEdgePolicy::default();

    let created = g
        .transaction_with_policy(&mut policy, |edit| {
            Ok::<_, ModelEditError>(edit.add_edge_split_from(
                source,
                EdgeAttr::new(
                    existing_dart,
                    Curve::line(Point3::new(2.0, 0.0, 0.0), Point3::new(3.0, 0.0, 0.0)),
                ),
            ))
        })
        .expect("the existing identity should absorb the local split identity");

    assert!(g.edge_attr(existing).is_some());
    assert!(g.edge_attr(created).is_none());
    assert!(policy.splits.is_empty());
}

#[derive(Default)]
struct RecordConsumedEdges {
    consumed: Vec<(EdgeKey, String)>,
}

impl EditPolicy<TestPayload> for RecordConsumedEdges {
    type Error = Infallible;

    fn vertex_created(
        &mut self,
        _: VertexKey,
        _: Origin,
        _: &Model<TestPayload>,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    fn edge_created(
        &mut self,
        _: EdgeKey,
        _: Origin,
        _: &Model<TestPayload>,
    ) -> Result<String, Self::Error> {
        Ok(String::new())
    }

    fn profile_created(
        &mut self,
        _: ProfileKey,
        _: Origin,
        _: &Model<TestPayload>,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    fn face_created(
        &mut self,
        _: FaceKey,
        _: Origin,
        _: &Model<TestPayload>,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    fn sheet_created(
        &mut self,
        _: SheetKey,
        _: Origin,
        _: &Model<TestPayload>,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    fn solid_created(
        &mut self,
        _: SolidKey,
        _: Origin,
        _: &Model<TestPayload>,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    /// Records what a removal disposed of, since nothing inherits it.
    fn edge_consumed(&mut self, key: EdgeKey, data: String) -> Result<(), Self::Error> {
        self.consumed.push((key, data));
        Ok(())
    }
}

#[test]
fn removing_an_edge_calls_the_consumed_hook_with_its_snapshot_payload() {
    let mut g = Model::<TestPayload>::new();
    let (_, removed) = add_named_test_edge(&mut g, 0.0, "gone");
    let mut policy = RecordConsumedEdges::default();

    g.transaction_with_policy(&mut policy, |edit| {
        edit.remove_edge(removed);
        Ok::<_, ModelEditError>(())
    })
    .expect("removing the edge should commit");

    assert_eq!(policy.consumed, vec![(removed, "gone".to_owned())]);
    assert!(g.edge_attr(removed).is_none());
}

#[derive(Default)]
struct CombineDerivedEdges {
    derived_from: Vec<(EdgeKey, Vec<EditKey>)>,
}

impl EditPolicy<TestPayload> for CombineDerivedEdges {
    type Error = Infallible;

    fn vertex_created(
        &mut self,
        _: VertexKey,
        _: Origin,
        _: &Model<TestPayload>,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    /// Concatenates every source's payload, in the order the builder named them.
    fn edge_created(
        &mut self,
        key: EdgeKey,
        origin: Origin,
        before: &Model<TestPayload>,
    ) -> Result<String, Self::Error> {
        match origin {
            Origin::Derived(sources) => {
                self.derived_from.push((key, sources.clone()));
                Ok(sources
                    .iter()
                    .map(|source| match source {
                        EditKey::Edge(source) => before.edge_attr_unchecked(*source).data().clone(),
                        _ => String::new(),
                    })
                    .collect::<Vec<_>>()
                    .join("+"))
            }
            _ => Ok(String::new()),
        }
    }

    fn profile_created(
        &mut self,
        _: ProfileKey,
        _: Origin,
        _: &Model<TestPayload>,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    fn face_created(
        &mut self,
        _: FaceKey,
        _: Origin,
        _: &Model<TestPayload>,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    fn sheet_created(
        &mut self,
        _: SheetKey,
        _: Origin,
        _: &Model<TestPayload>,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    fn solid_created(
        &mut self,
        _: SolidKey,
        _: Origin,
        _: &Model<TestPayload>,
    ) -> Result<(), Self::Error> {
        Ok(())
    }
}

#[test]
fn derived_creation_names_every_source_resolved_to_its_transaction_start_identity() {
    let mut g = Model::<TestPayload>::new();
    let (_, left) = add_named_test_edge(&mut g, 0.0, "left");
    let (_, right) = add_named_test_edge(&mut g, 2.0, "right");
    let mut policy = CombineDerivedEdges::default();

    let created = g
        .transaction_with_policy(&mut policy, |edit| {
            let start = edit.add_dart();
            let end = edit.add_dart();
            edit.link(Dim::Zero, start, end)?;
            Ok::<_, ModelEditError>(edit.add_edge_derived_from(
                vec![EditKey::Edge(left), EditKey::Edge(right)],
                EdgeAttr::new(
                    start,
                    Curve::line(Point3::new(4.0, 0.0, 0.0), Point3::new(5.0, 0.0, 0.0)),
                ),
            ))
        })
        .expect("the derived creation should commit");

    assert_eq!(
        policy.derived_from,
        vec![(created, vec![EditKey::Edge(left), EditKey::Edge(right)])]
    );
    assert_eq!(g.edge_attr_unchecked(created).data(), "left+right");
}

#[test]
fn a_derived_creation_with_only_local_sources_resolves_to_new() {
    let mut g = Model::<TestPayload>::new();
    let mut policy = CombineDerivedEdges::default();

    let created = g
        .transaction_with_policy(&mut policy, |edit| {
            let source_start = edit.add_dart();
            let source_end = edit.add_dart();
            edit.link(Dim::Zero, source_start, source_end)?;
            let local_source = edit.add_edge(EdgeAttr::new(
                source_start,
                Curve::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0)),
            ));
            let start = edit.add_dart();
            let end = edit.add_dart();
            edit.link(Dim::Zero, start, end)?;
            let created = edit.add_edge_derived_from(
                vec![EditKey::Edge(local_source)],
                EdgeAttr::new(
                    start,
                    Curve::line(Point3::new(4.0, 0.0, 0.0), Point3::new(5.0, 0.0, 0.0)),
                ),
            );
            edit.remove_edge(local_source);
            Ok::<_, ModelEditError>(created)
        })
        .expect("the derived creation should commit");

    assert!(policy.derived_from.is_empty());
    assert_eq!(g.edge_attr_unchecked(created).data(), "");
}
