//! What a model owns, and what a transaction does to all of it at once.
//!
//! A `Model` holds four kinds of state that have to move together: the pure
//! map, the entity stores keyed against it, the subdivision labels saying which
//! entity each raw cell falls inside, and the derived indexes over both. A
//! commit advances every one of them or none, and a rollback puts every one of
//! them back.

use ngk::builders::edges::add_edge;
use ngk::geometry::{Curve, Point3};
use ngk::model::Model;
use ngk::topology::gmap::{Dart, Dim};
use ngk::topology::shape_keys::{EdgeKey, FaceKey};
use ngk::topology::subdivision::EntityOwner;
use ngk::topology::{ModelEditError, StandardPayload};

/// A model holding one edge between two points, and that edge's key.
fn one_edge() -> (Model<StandardPayload>, EdgeKey) {
    let mut model = Model::<StandardPayload>::new();
    let start = Point3::new(0.0, 0.0, 0.0);
    let end = Point3::new(1.0, 0.0, 0.0);
    let key = add_edge(&mut model, start, end, Curve::line(start, end)).expect("an edge builds");
    (model, key)
}

#[test]
fn a_new_model_is_at_revision_zero_and_every_commit_advances_it() {
    let mut model = Model::<StandardPayload>::new();
    assert_eq!(model.revision(), 0);

    model
        .transaction(|edit| Ok::<_, ModelEditError>(edit.add_dart()))
        .expect("adding a dart commits");
    assert_eq!(model.revision(), 1);

    model
        .transaction(|edit| Ok::<_, ModelEditError>(edit.add_dart()))
        .expect("adding another dart commits");
    assert_eq!(model.revision(), 2);
}

#[test]
fn a_failed_transaction_leaves_the_revision_where_it_was() {
    let mut model = Model::<StandardPayload>::new();
    model
        .transaction(|edit| Ok::<_, ModelEditError>(edit.add_dart()))
        .expect("adding a dart commits");
    let before = model.revision();

    let result = model.transaction(|edit| {
        let dart = edit.add_dart();
        Err::<(), _>(ModelEditError::SameDart { dart })
    });

    assert!(result.is_err());
    assert_eq!(model.revision(), before);
    assert_eq!(model.dart_count(), 1);
}

#[test]
fn a_commit_carries_the_subdivision_with_the_map() {
    let (mut model, edge) = one_edge();
    let anchor = model.edge_attr_unchecked(edge).dart;

    model
        .transaction(|edit| {
            edit.own_cell(Dim::Zero, anchor, EntityOwner::Edge(edge));
            Ok::<_, ModelEditError>(())
        })
        .expect("labelling the closure point commits");

    assert_eq!(
        model.ownership().owner(Dim::Zero, anchor),
        Some(EntityOwner::Edge(edge)),
        "the label is readable through the model's own index"
    );
    assert_eq!(model.subdivision().records().len(), 1);
}

#[test]
fn a_rollback_restores_the_subdivision_as_well_as_the_map() {
    let (mut model, edge) = one_edge();
    let anchor = model.edge_attr_unchecked(edge).dart;
    model
        .transaction(|edit| {
            edit.own_cell(Dim::Zero, anchor, EntityOwner::Edge(edge));
            Ok::<_, ModelEditError>(())
        })
        .expect("the first label commits");

    let result = model.transaction(|edit| {
        let dart = edit.add_dart();
        edit.own_cell(Dim::One, dart, EntityOwner::Edge(edge));
        Err::<(), _>(ModelEditError::SameDart { dart })
    });

    assert!(result.is_err());
    assert_eq!(
        model.subdivision().records().len(),
        1,
        "the rolled-back label is gone"
    );
    assert_eq!(
        model.ownership().owner(Dim::Zero, anchor),
        Some(EntityOwner::Edge(edge)),
        "and the committed one is still there"
    );
}

#[test]
fn a_label_two_entities_disagree_about_is_rejected_at_commit() {
    let (mut model, edge) = one_edge();
    let anchor = model.edge_attr_unchecked(edge).dart;
    let intruder = EntityOwner::Face(FaceKey::default());

    let result = model.transaction(|edit| {
        edit.own_cell(Dim::Zero, anchor, EntityOwner::Edge(edge));
        edit.own_cell(Dim::Zero, anchor, intruder);
        Ok::<_, ModelEditError>(())
    });

    assert!(
        matches!(result, Err(ModelEditError::InvalidSubdivision(_))),
        "one raw cell cannot be inside two entities"
    );
    assert!(model.subdivision().records().is_empty());
}

#[test]
fn a_label_anchored_off_the_map_is_rejected_at_commit() {
    let (mut model, edge) = one_edge();
    let beyond = Dart::new(model.dart_count());

    let result = model.transaction(|edit| {
        edit.own_cell(Dim::One, beyond, EntityOwner::Edge(edge));
        Ok::<_, ModelEditError>(())
    });

    assert!(matches!(result, Err(ModelEditError::InvalidSubdivision(_))));
}

#[test]
fn a_label_survives_the_renumbering_that_removing_darts_causes() {
    let mut model = Model::<StandardPayload>::new();
    let owner = EntityOwner::Face(FaceKey::default());

    // Two isolated darts, with the second one labelled. Dropping the first
    // renumbers the second, and the label has to move with it.
    let second = model
        .transaction(|edit| {
            let first = edit.add_dart();
            let second = edit.add_dart();
            edit.own_cell(Dim::Two, second, owner);
            let _ = first;
            Ok::<_, ModelEditError>(second)
        })
        .expect("two darts commit");
    assert_eq!(second, Dart::new(1));

    model
        .transaction(|edit| {
            edit.remove_isolated_darts(vec![ngk::topology::IsolatedDart::new(Dart::new(0))]);
            Ok::<_, ModelEditError>(())
        })
        .expect("removing the unlabelled dart commits");

    assert_eq!(model.dart_count(), 1);
    assert_eq!(
        model.ownership().owner(Dim::Two, Dart::new(0)),
        Some(owner),
        "the record follows the cell it names, not the number it had"
    );
}

#[test]
fn a_warm_index_and_a_cold_one_answer_the_same_after_an_edit() {
    let (mut model, edge) = one_edge();
    let anchor = model.edge_attr_unchecked(edge).dart;

    // Warm every derived lookup before touching the model.
    let _ = model.edge_attr(edge);
    let _ = model.cell_key::<ngk::model::Cell1>(anchor);
    let _ = model.ownership();

    model
        .transaction(|edit| {
            edit.own_cell(Dim::Zero, anchor, EntityOwner::Edge(edge));
            Ok::<_, ModelEditError>(())
        })
        .expect("labelling commits");

    let warm = model.ownership().owner(Dim::Zero, anchor);
    let cold = model
        .subdivision()
        .index(model.topology())
        .expect("the labelling describes the map")
        .owner(Dim::Zero, anchor);
    assert_eq!(warm, cold);
    assert_eq!(warm, Some(EntityOwner::Edge(edge)));
}

#[test]
fn a_rolled_back_model_still_serializes_to_the_state_it_kept() {
    let (mut model, edge) = one_edge();
    let anchor = model.edge_attr_unchecked(edge).dart;
    model
        .transaction(|edit| {
            edit.own_cell(Dim::Zero, anchor, EntityOwner::Edge(edge));
            Ok::<_, ModelEditError>(())
        })
        .expect("labelling commits");
    let committed = serde_json::to_string(&model).expect("a model serializes");

    let result = model.transaction(|edit| {
        let dart = edit.add_dart();
        Err::<(), _>(ModelEditError::SameDart { dart })
    });
    assert!(result.is_err());

    assert_eq!(
        serde_json::to_string(&model).expect("a model serializes"),
        committed,
        "a rollback leaves nothing of the failed edit to serialize"
    );
}

#[test]
fn a_deserialized_model_rebuilds_its_derived_lookups() {
    let (mut model, edge) = one_edge();
    let anchor = model.edge_attr_unchecked(edge).dart;
    model
        .transaction(|edit| {
            edit.own_cell(Dim::Zero, anchor, EntityOwner::Edge(edge));
            Ok::<_, ModelEditError>(())
        })
        .expect("labelling commits");

    let encoded = serde_json::to_string(&model).expect("a model serializes");
    let decoded: Model<StandardPayload> =
        serde_json::from_str(&encoded).expect("a model deserializes");

    assert_eq!(decoded.revision(), model.revision());
    assert_eq!(
        decoded.cell_key::<ngk::model::Cell1>(anchor),
        Some(edge),
        "the dart-to-key index comes back from the stores, not from the file"
    );
    assert_eq!(
        decoded.ownership().owner(Dim::Zero, anchor),
        Some(EntityOwner::Edge(edge))
    );
}

#[test]
fn a_key_from_a_removed_entity_never_resolves_to_the_one_that_replaced_it() {
    let (mut model, edge) = one_edge();
    let stale = edge;

    model
        .transaction(|edit| {
            edit.remove_edge(stale);
            Ok::<_, ModelEditError>(())
        })
        .expect("removing the edge commits");

    let start = Point3::new(0.0, 1.0, 0.0);
    let end = Point3::new(1.0, 1.0, 0.0);
    let fresh =
        add_edge(&mut model, start, end, Curve::line(start, end)).expect("a second edge builds");

    assert_ne!(stale, fresh, "a reused slot still gets a new generation");
    assert!(
        model.edge_attr(stale).is_none(),
        "the old key resolves to nothing at all"
    );
}
