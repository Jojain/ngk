//! Writing ISO 10303-21 text.
//!
//! The real-formatting table below is the point of this file. `format!("{}",
//! 1.0f64)` yields `"1"`, which is an *integer* in Part 21 and silently
//! changes the parsed type of an attribute — one of the three conversions in
//! this feature that corrupt without ever failing.

use ngk::exchange::step::part21::{
    EntityId, StepExchange, Instance, Record, Value, WriteError, encode_text, exchange_to_string,
    format_real, parse_exchange,
};

/// Returns the sole instance of an exchange structure built from `records`.
fn instance(id: u64, records: Vec<Record>) -> Instance {
    Instance {
        id: EntityId(id),
        records,
        line: 0,
    }
}

/// Writes one instance and returns the whole document.
fn write_one(records: Vec<Record>) -> String {
    let exchange =
        StepExchange::new(Vec::new(), vec![instance(1, records)]).expect("one instance is unique");
    exchange_to_string(&exchange).expect("the exchange structure should write")
}

#[test]
fn a_whole_number_real_is_written_with_its_decimal_point() {
    // Without the point this reads back as an integer and the attribute
    // changes type.
    assert_eq!(format_real(1.0).expect("finite"), "1.0");
    assert_eq!(format_real(0.0).expect("finite"), "0.0");
    assert_eq!(format_real(-0.0).expect("finite"), "-0.0");
    assert_eq!(format_real(42.0).expect("finite"), "42.0");
}

#[test]
fn a_real_with_an_exponent_carries_its_point_in_the_mantissa() {
    // `1e300.` is not a Part 21 real: the point belongs before the exponent.
    assert_eq!(format_real(1.0e300).expect("finite"), "1.E300");
    assert_eq!(format_real(1.0e-7).expect("finite"), "1.E-7");
    assert_eq!(format_real(1.0e21).expect("finite"), "1.E21");
    assert_eq!(
        format_real(f64::MAX).expect("finite"),
        "1.7976931348623157E308",
    );
}

#[test]
fn every_awkward_real_survives_being_written_and_read_back() {
    // The property that matters: the same bits, and still a real.
    let awkward = [
        0.0,
        -0.0,
        1.0,
        -1.0,
        0.1,
        1.0 / 3.0,
        2.5,
        1.0e-7,
        1.0e21,
        1.0e300,
        1.0e-300,
        f64::MAX,
        f64::MIN,
        f64::MIN_POSITIVE,
        f64::EPSILON,
        std::f64::consts::PI,
        std::f64::consts::TAU,
        -12345.6789,
    ];

    for value in awkward {
        let source = write_one(vec![Record::new("SAMPLE", vec![Value::Real(value)])]);
        let exchange = parse_exchange(&source)
            .unwrap_or_else(|error| panic!("{value:?} should write parseable text: {error}"));
        let written = exchange
            .get(EntityId(1))
            .expect("#1")
            .simple()
            .expect("simple")
            .params[0]
            .clone();

        assert_eq!(
            written,
            Value::Real(value),
            "{value:?} did not survive the round trip",
        );
        assert_eq!(
            written.as_real().expect("a real").to_bits(),
            value.to_bits(),
            "{value:?} lost bits, or its sign",
        );
    }
}

#[test]
fn an_integer_is_written_without_a_decimal_point() {
    let source = write_one(vec![Record::new(
        "SAMPLE",
        vec![Value::Integer(1), Value::Integer(-3)],
    )]);

    assert!(source.contains("SAMPLE(1,-3)"), "got {source}");
}

#[test]
fn a_non_finite_real_is_refused_rather_than_written() {
    // Part 21 has no spelling for these, so writing one produces a file no
    // reader can parse.
    for value in [f64::INFINITY, f64::NEG_INFINITY, f64::NAN] {
        let error = format_real(value).expect_err("a non-finite real should be refused");
        assert!(matches!(error, WriteError::NonFiniteReal { .. }));
    }

    let exchange = StepExchange::new(
        Vec::new(),
        vec![instance(
            1,
            vec![Record::new("SAMPLE", vec![Value::Real(f64::NAN)])],
        )],
    )
    .expect("one instance is unique");
    assert!(exchange_to_string(&exchange).is_err());
}

#[test]
fn a_quote_and_a_backslash_are_escaped_on_the_way_out() {
    assert_eq!(encode_text("it's here"), "it''s here");
    assert_eq!(encode_text(r"C:\temp"), r"C:\\temp");
    assert_eq!(encode_text("plain ASCII 123"), "plain ASCII 123");
}

#[test]
fn adjacent_non_ascii_characters_share_one_escape_run() {
    assert_eq!(encode_text("Äü"), r"\X2\00C400FC\X0\");
    assert_eq!(encode_text("café"), r"caf\X2\00E9\X0\");
    assert_eq!(encode_text("😀"), r"\X2\D83DDE00\X0\");
}

#[test]
fn an_empty_string_writes_as_an_empty_literal() {
    let source = write_one(vec![Record::new(
        "SAMPLE",
        vec![Value::Text(String::new())],
    )]);

    assert!(source.contains("SAMPLE('')"), "got {source}");
}

#[test]
fn a_complex_instance_is_written_as_one_parenthesised_group() {
    let source = write_one(vec![
        Record::new("NAMED_UNIT", vec![Value::Derived]),
        Record::new(
            "SI_UNIT",
            vec![
                Value::Enum("MILLI".to_string()),
                Value::Enum("METRE".to_string()),
            ],
        ),
    ]);

    assert!(
        source.contains("#1 = (NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));"),
        "got {source}",
    );
}

#[test]
fn the_document_carries_its_section_keywords() {
    let exchange = StepExchange::new(
        vec![Record::new(
            "FILE_SCHEMA",
            vec![Value::List(vec![Value::Text(
                "CONFIG_CONTROL_DESIGN".into(),
            )])],
        )],
        vec![instance(1, vec![Record::new("SAMPLE", Vec::new())])],
    )
    .expect("one instance is unique");

    let source = exchange_to_string(&exchange).expect("the exchange structure should write");
    for keyword in [
        "ISO-10303-21;",
        "HEADER;",
        "FILE_SCHEMA(('CONFIG_CONTROL_DESIGN'));",
        "ENDSEC;",
        "DATA;",
        "END-ISO-10303-21;",
    ] {
        assert!(source.contains(keyword), "{keyword} missing from {source}");
    }
}

#[test]
fn a_duplicate_entity_id_cannot_be_assembled_into_a_document() {
    let error = StepExchange::new(
        Vec::new(),
        vec![
            instance(1, vec![Record::new("FIRST", Vec::new())]),
            instance(1, vec![Record::new("AGAIN", Vec::new())]),
        ],
    )
    .expect_err("two instances cannot share one name");

    assert_eq!(error.id, EntityId(1));
}
