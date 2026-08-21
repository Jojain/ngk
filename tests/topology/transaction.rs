use ngk::topology::gmap::{Dim, GMap};
use ngk::topology::{StandardPayload, TopologyEditError};

#[test]
fn transaction_commits_all_staged_edits() {
    let mut g = GMap::<StandardPayload>::new();

    let dart = g
        .transaction(|g| g.edit(|edit| Ok(edit.add_dart())))
        .expect("transaction should commit");

    assert_eq!(g.dart_count(), 1);
    assert_eq!(dart.id(), 0);
}

#[test]
fn failed_transaction_restores_the_complete_map() {
    let mut g = GMap::<StandardPayload>::new();

    let result = g.transaction(|g| {
        let dart = g.edit(|edit| Ok(edit.add_dart()))?;
        Err::<(), _>(TopologyEditError::SameDart { dart })
    });

    assert!(matches!(result, Err(TopologyEditError::SameDart { .. })));
    assert_eq!(g.dart_count(), 0);
}

#[test]
fn nested_transactions_defer_validation_until_the_outer_commit() {
    let mut g = GMap::<StandardPayload>::new();

    g.transaction(|g| {
        let darts = g.edit(|edit| {
            let darts: [_; 4] = std::array::from_fn(|_| edit.add_dart());
            edit.link(Dim::Zero, darts[0], darts[1])?;
            edit.link(Dim::Zero, darts[2], darts[3])?;
            Ok(darts)
        })?;

        g.transaction(|g| g.edit(|edit| edit.link(Dim::Two, darts[0], darts[2])))?;
        g.edit(|edit| edit.link(Dim::Two, darts[1], darts[3]))?;
        Ok::<(), TopologyEditError>(())
    })
    .expect("only the complete outer topology should be validated");

    assert_eq!(g.dart_count(), 4);
}

#[test]
fn caught_nested_failure_poisons_and_rolls_back_the_outer_transaction() {
    let mut g = GMap::<StandardPayload>::new();

    let result = g.transaction(|g| {
        let nested = g.transaction(|g| {
            let dart = g.edit(|edit| Ok(edit.add_dart()))?;
            Err::<(), _>(TopologyEditError::SameDart { dart })
        });
        assert!(matches!(nested, Err(TopologyEditError::SameDart { .. })));
        Ok::<(), TopologyEditError>(())
    });

    assert!(matches!(
        result,
        Err(TopologyEditError::TransactionPoisoned)
    ));
    assert_eq!(g.dart_count(), 0);
}
