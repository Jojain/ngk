use ngk::model::Model;
use ngk::topology::gmap::Dim;
use ngk::topology::{ModelEditError, StandardPayload};

#[test]
fn transaction_commits_all_staged_edits() {
    let mut g = Model::<StandardPayload>::new();

    let dart = g
        .transaction(|edit| Ok::<_, ModelEditError>(edit.add_dart()))
        .expect("transaction should commit");

    assert_eq!(g.dart_count(), 1);
    assert_eq!(dart.id(), 0);
}

#[test]
fn failed_transaction_restores_the_complete_map() {
    let mut g = Model::<StandardPayload>::new();

    let result = g.transaction(|edit| {
        let dart = edit.add_dart();
        Err::<(), _>(ModelEditError::SameDart { dart })
    });

    assert!(matches!(result, Err(ModelEditError::SameDart { .. })));
    assert_eq!(g.dart_count(), 0);
}

#[test]
fn transaction_defers_validation_until_the_complete_operation() {
    let mut g = Model::<StandardPayload>::new();

    g.transaction(|edit| {
        let darts: [_; 4] = std::array::from_fn(|_| edit.add_dart());
        edit.link(Dim::Zero, darts[0], darts[1])?;
        edit.link(Dim::Zero, darts[2], darts[3])?;
        edit.link(Dim::Two, darts[0], darts[2])?;
        edit.link(Dim::Two, darts[1], darts[3])?;
        Ok::<(), ModelEditError>(())
    })
    .expect("only the complete outer topology should be validated");

    assert_eq!(g.dart_count(), 4);
}
