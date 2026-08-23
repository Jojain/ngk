use ngk::topology::gmap::{Dim, GMap};
use ngk::topology::{StandardPayload, TopologyEditError};

#[test]
fn transaction_commits_all_staged_edits() {
    let mut g = GMap::<StandardPayload>::new();

    let dart = g
        .transaction(|edit| Ok::<_, TopologyEditError>(edit.add_dart()))
        .expect("transaction should commit");

    assert_eq!(g.dart_count(), 1);
    assert_eq!(dart.id(), 0);
}

#[test]
fn failed_transaction_restores_the_complete_map() {
    let mut g = GMap::<StandardPayload>::new();

    let result = g.transaction(|g| {
        let dart = g.add_dart();
        Err::<(), _>(TopologyEditError::SameDart { dart })
    });

    assert!(matches!(result, Err(TopologyEditError::SameDart { .. })));
    assert_eq!(g.dart_count(), 0);
}

#[test]
fn transaction_defers_validation_until_the_complete_operation() {
    let mut g = GMap::<StandardPayload>::new();

    g.transaction(|g| {
        let darts: [_; 4] = std::array::from_fn(|_| g.add_dart());
        g.link(Dim::Zero, darts[0], darts[1])?;
        g.link(Dim::Zero, darts[2], darts[3])?;
        g.link(Dim::Two, darts[0], darts[2])?;
        g.link(Dim::Two, darts[1], darts[3])?;
        Ok::<(), TopologyEditError>(())
    })
    .expect("only the complete outer topology should be validated");

    assert_eq!(g.dart_count(), 4);
}
