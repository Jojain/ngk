//! Sharing instances, and the two ways it could go wrong.
//!
//! `add_shared` collapses equal records into one instance. Getting that wrong
//! is silent in both directions, but only one of them is dangerous: failing to
//! share makes a file larger, while sharing two records that differ merges two
//! entities a reader will then treat as one. These pin the boundary.

use ngk::exchange::step::builder::InstanceBuilder;
use ngk::exchange::step::part21::{Record, Value};

fn point(x: f64, y: f64, z: f64) -> Record {
    Record::new(
        "CARTESIAN_POINT",
        vec![
            Value::Text(String::new()),
            Value::List(vec![Value::Real(x), Value::Real(y), Value::Real(z)]),
        ],
    )
}

#[test]
fn an_identical_record_is_written_once() {
    let mut builder = InstanceBuilder::new();

    let first = builder.add_shared(point(1.0, 2.0, 3.0));
    let second = builder.add_shared(point(1.0, 2.0, 3.0));

    assert_eq!(first, second);
    assert_eq!(builder.finish(Vec::new()).instances().len(), 1);
}

#[test]
fn a_record_added_unshared_is_never_merged_with_an_equal_one() {
    // What keeps two corners of a solid at the same position from becoming one
    // corner: identity-bearing entities do not consult the share table at all.
    let mut builder = InstanceBuilder::new();

    let first = builder.add(point(1.0, 2.0, 3.0));
    let second = builder.add(point(1.0, 2.0, 3.0));

    assert_ne!(first, second);
    assert_eq!(builder.finish(Vec::new()).instances().len(), 2);
}

#[test]
fn an_unshared_record_is_not_offered_to_later_sharers() {
    // Adding unshared must not seed the table, or the next `add_shared` would
    // silently alias an entity that was deliberately given its own name.
    let mut builder = InstanceBuilder::new();

    let owned = builder.add(point(1.0, 2.0, 3.0));
    let shared = builder.add_shared(point(1.0, 2.0, 3.0));

    assert_ne!(owned, shared);
}

#[test]
fn reals_are_compared_by_bits_rather_than_within_a_tolerance() {
    // The safe direction to err in: values that differ at all get their own
    // instances. Arithmetic error can cost file size and never correctness.
    let mut builder = InstanceBuilder::new();

    let exact = builder.add_shared(point(1.0, 0.0, 0.0));
    let nudged = builder.add_shared(point(1.0 + f64::EPSILON, 0.0, 0.0));

    assert_ne!(exact, nudged, "near-equal positions must not be merged");
}

#[test]
fn positive_and_negative_zero_stay_distinct() {
    // They are written differently — `0.0` against `-0.0` — so merging them
    // would make the shared instance disagree with one of its users. On a
    // DIRECTION the sign is the direction.
    let mut builder = InstanceBuilder::new();

    let positive = builder.add_shared(point(0.0, 0.0, 0.0));
    let negative = builder.add_shared(point(-0.0, 0.0, 0.0));

    assert_ne!(positive, negative);
}

#[test]
fn a_record_differing_only_in_a_reference_is_not_merged() {
    let mut builder = InstanceBuilder::new();
    let first_target = builder.add(Record::new("TARGET", Vec::new()));
    let second_target = builder.add(Record::new("TARGET", Vec::new()));

    let first = builder.add_shared(Record::new("HOLDER", vec![Value::Ref(first_target)]));
    let second = builder.add_shared(Record::new("HOLDER", vec![Value::Ref(second_target)]));

    assert_ne!(first, second);
}

#[test]
fn a_record_differing_only_in_its_keyword_is_not_merged() {
    let mut builder = InstanceBuilder::new();

    let first = builder.add_shared(Record::new("FIRST", vec![Value::Real(1.0)]));
    let second = builder.add_shared(Record::new("SECOND", vec![Value::Real(1.0)]));

    assert_ne!(first, second);
}

#[test]
fn an_integer_and_a_real_of_the_same_value_are_not_merged() {
    let mut builder = InstanceBuilder::new();

    let integer = builder.add_shared(Record::new("N", vec![Value::Integer(1)]));
    let real = builder.add_shared(Record::new("N", vec![Value::Real(1.0)]));

    assert_ne!(integer, real);
}

#[test]
fn a_string_cannot_impersonate_a_parameter_boundary() {
    // The encoding that keys the share table has to be uniquely decodable. If
    // a string's own bytes can be read as structure, then one parameter can
    // spell what two parameters spell, and two unrelated entities collapse
    // into one instance — silently, and in the dangerous direction.
    let mut builder = InstanceBuilder::new();

    let one_param =
        builder.add_shared(Record::new("K", vec![Value::Text("x\u{1},ty".to_string())]));
    let two_params = builder.add_shared(Record::new(
        "K",
        vec![Value::Text("x".to_string()), Value::Text("y".to_string())],
    ));

    assert_ne!(one_param, two_params);
}

#[test]
fn a_nested_aggregate_cannot_impersonate_a_flat_one() {
    let mut builder = InstanceBuilder::new();

    let nested = builder.add_shared(Record::new(
        "K",
        vec![Value::List(vec![Value::Integer(1), Value::Integer(2)])],
    ));
    let flat = builder.add_shared(Record::new("K", vec![Value::Integer(1), Value::Integer(2)]));

    assert_ne!(nested, flat);
}

#[test]
fn instance_names_are_allocated_in_emission_order() {
    let mut builder = InstanceBuilder::new();

    let first = builder.add(Record::new("A", Vec::new()));
    let second = builder.add_shared(Record::new("B", Vec::new()));
    let third = builder.add_complex(vec![
        Record::new("C", Vec::new()),
        Record::new("D", Vec::new()),
    ]);

    assert_eq!(first.0, 1);
    assert_eq!(second.0, 2);
    assert_eq!(third.0, 3);

    let exchange = builder.finish(Vec::new());
    assert_eq!(exchange.instances().len(), 3);
    assert_eq!(
        exchange
            .get(third)
            .expect("the complex instance")
            .records
            .len(),
        2,
    );
}
