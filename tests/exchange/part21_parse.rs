//! Reading ISO 10303-21 text.
//!
//! Every case here is a string literal rather than a fixture file, because L1
//! operates on `&str` and never touches a filesystem.

use ngk::exchange::step::part21::{EntityId, SyntaxError, Value, parse_exchange};

/// Wraps data-section text in the smallest legal exchange structure.
fn data_section(instances: &str) -> String {
    format!("ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n{instances}\nENDSEC;\nEND-ISO-10303-21;\n")
}

/// Returns the parameters of the sole record of the sole instance.
fn sole_params(source: &str) -> Vec<Value> {
    let exchange = parse_exchange(source).expect("the exchange structure should parse");
    let instances = exchange.instances();
    assert_eq!(instances.len(), 1, "expected exactly one instance");
    instances[0]
        .simple()
        .expect("the instance should be simple")
        .params
        .clone()
}

#[test]
fn a_real_and_an_integer_are_distinguished_by_the_decimal_point() {
    let params = sole_params(&data_section("#1 = SAMPLE(1.,1,2.5,-3,+4.,0.);"));

    assert_eq!(
        params,
        vec![
            Value::Real(1.0),
            Value::Integer(1),
            Value::Real(2.5),
            Value::Integer(-3),
            Value::Real(4.0),
            Value::Real(0.0),
        ],
    );
}

#[test]
fn a_real_written_with_an_exponent_keeps_its_value() {
    let params = sole_params(&data_section("#1 = SAMPLE(1.E-7,1.0E3,-2.5E+2);"));

    assert_eq!(
        params,
        vec![
            Value::Real(1.0e-7),
            Value::Real(1000.0),
            Value::Real(-250.0),
        ],
    );
}

#[test]
fn an_enumeration_is_not_read_as_a_real() {
    // The hazard is `.T.`: a leading `.` must dispatch on what follows it, or
    // a boolean is read as a malformed number.
    let params = sole_params(&data_section("#1 = SAMPLE(.T.,.F.,.UNSPECIFIED.);"));

    assert_eq!(
        params,
        vec![
            Value::Enum("T".to_string()),
            Value::Enum("F".to_string()),
            Value::Enum("UNSPECIFIED".to_string()),
        ],
    );
}

#[test]
fn an_unset_parameter_and_a_derived_one_stay_distinct() {
    let params = sole_params(&data_section("#1 = SAMPLE($,*);"));

    assert_eq!(params, vec![Value::Null, Value::Derived]);
}

#[test]
fn a_doubled_quote_decodes_to_one_literal_quote() {
    let params = sole_params(&data_section("#1 = SAMPLE('it''s here','','a''''b');"));

    assert_eq!(
        params,
        vec![
            Value::Text("it's here".to_string()),
            Value::Text(String::new()),
            // Two escaped quotes decode to two literal quotes.
            Value::Text("a''b".to_string()),
        ],
    );
}

#[test]
fn an_x2_escape_decodes_to_its_utf16_text() {
    let params = sole_params(&data_section(
        r"#1 = SAMPLE('caf\X2\00E9\X0\','\X2\00C400FC\X0\');",
    ));

    assert_eq!(
        params,
        vec![
            Value::Text("café".to_string()),
            Value::Text("Äü".to_string()),
        ],
    );
}

#[test]
fn an_x2_escape_decodes_a_surrogate_pair_as_one_character() {
    let params = sole_params(&data_section(r"#1 = SAMPLE('\X2\D83DDE00\X0\');"));

    assert_eq!(params, vec![Value::Text("😀".to_string())]);
}

#[test]
fn an_x4_escape_decodes_its_code_points() {
    let params = sole_params(&data_section(r"#1 = SAMPLE('\X4\0001F600\X0\');"));

    assert_eq!(params, vec![Value::Text("😀".to_string())]);
}

#[test]
fn a_single_byte_and_a_shift_escape_decode_to_their_code_page_characters() {
    let params = sole_params(&data_section(r"#1 = SAMPLE('\X\E9','\S\A');"));

    assert_eq!(
        params,
        vec![
            Value::Text("é".to_string()),
            // `\S\A` shifts 'A' (0x41) into the upper half: 0xC1.
            Value::Text("Á".to_string()),
        ],
    );
}

#[test]
fn a_doubled_backslash_decodes_to_one_backslash() {
    let params = sole_params(&data_section(r"#1 = SAMPLE('C:\\temp');"));

    assert_eq!(params, vec![Value::Text(r"C:\temp".to_string())]);
}

#[test]
fn an_unrecognized_escape_is_passed_through_literally() {
    // A malformed escape in one vendor string must not sink an otherwise good
    // file, so it costs its own fidelity and nothing else.
    let params = sole_params(&data_section(r"#1 = SAMPLE('a\P?\b','\X2\nothex\X0\');"));

    assert_eq!(
        params,
        vec![
            Value::Text(r"a\P?\b".to_string()),
            Value::Text(r"\X2\nothex\X0\".to_string()),
        ],
    );
}

#[test]
fn a_comment_between_tokens_is_ignored() {
    let source = data_section("#1 /* named */ = SAMPLE(/* first */ 1, 2 /* last */);");
    let params = sole_params(&source);

    assert_eq!(params, vec![Value::Integer(1), Value::Integer(2)]);
}

#[test]
fn a_nested_aggregate_keeps_its_structure() {
    let params = sole_params(&data_section("#1 = SAMPLE((1,2),((3.,4.)),());"));

    assert_eq!(
        params,
        vec![
            Value::List(vec![Value::Integer(1), Value::Integer(2)]),
            Value::List(vec![Value::List(vec![Value::Real(3.0), Value::Real(4.0)])]),
            Value::List(Vec::new()),
        ],
    );
}

#[test]
fn a_typed_parameter_keeps_its_keyword() {
    // This is how a SELECT over defined types reaches the file, as in
    // TRIMMED_CURVE's trimming values.
    let params = sole_params(&data_section("#1 = SAMPLE((PARAMETER_VALUE(0.)));"));

    let list = params[0].as_list().expect("the parameter should be a list");
    let typed = list[0].as_typed().expect("the element should be typed");
    assert!(typed.is("PARAMETER_VALUE"));
    assert_eq!(typed.params, vec![Value::Real(0.0)]);
}

#[test]
fn a_reference_resolves_through_the_instance_table() {
    let source = data_section("#1 = CARTESIAN_POINT('',(0.,0.,0.));\n#2 = VERTEX_POINT('',#1);");
    let exchange = parse_exchange(&source).expect("the exchange structure should parse");

    let vertex = exchange
        .get(EntityId(2))
        .expect("instance #2 should be in the table");
    let referenced = vertex.simple().expect("simple instance").params[1]
        .as_reference()
        .expect("the second parameter should be a reference");

    assert_eq!(referenced, EntityId(1));
    assert!(
        exchange
            .get(referenced)
            .expect("the reference should resolve")
            .is("CARTESIAN_POINT")
    );
}

#[test]
fn a_complex_instance_carries_every_record_under_one_name() {
    // AP203 requires this shape for the rational B-spline forms and for the
    // unit block, so it must be the same type as a simple instance.
    let source = data_section("#1 = (NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) LENGTH_UNIT());");
    let exchange = parse_exchange(&source).expect("the exchange structure should parse");
    let instance = exchange
        .get(EntityId(1))
        .expect("instance #1 should be in the table");

    assert_eq!(instance.records.len(), 3);
    assert!(instance.simple().is_none());
    assert!(instance.is("SI_UNIT"));
    assert!(instance.is("LENGTH_UNIT"));
    assert_eq!(
        instance
            .record("SI_UNIT")
            .expect("the SI_UNIT record should be selectable")
            .params,
        vec![
            Value::Enum("MILLI".to_string()),
            Value::Enum("METRE".to_string()),
        ],
    );
    assert_eq!(
        instance
            .record("NAMED_UNIT")
            .expect("the NAMED_UNIT record should be selectable")
            .params,
        vec![Value::Derived],
    );
}

#[test]
fn header_records_are_readable_by_keyword() {
    let source = "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION((''),'2;1');\n\
         FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\nENDSEC;\nEND-ISO-10303-21;\n";
    let exchange = parse_exchange(source).expect("the exchange structure should parse");

    let schema = exchange
        .header_record("FILE_SCHEMA")
        .expect("FILE_SCHEMA should be in the header");
    assert_eq!(
        schema.params,
        vec![Value::List(vec![Value::Text(
            "AUTOMOTIVE_DESIGN".to_string()
        )])],
    );
    assert!(exchange.instances().is_empty());
}

#[test]
fn every_instance_carries_the_line_it_starts_on() {
    // Every later layer names positions with these, so a STEP error is
    // something a user can find in their file.
    let source = data_section("#1 = FIRST();\n\n#7 = SECOND();");
    let exchange = parse_exchange(&source).expect("the exchange structure should parse");

    assert_eq!(exchange.get(EntityId(1)).expect("#1").line, 5);
    assert_eq!(exchange.get(EntityId(7)).expect("#7").line, 7);
}

#[test]
fn instances_can_be_swept_by_keyword_in_file_order() {
    // The read side finds MANIFOLD_SOLID_BREP this way rather than walking
    // down through product structure.
    let source = data_section("#1 = FACE();\n#2 = SOLID();\n#3 = FACE();");
    let exchange = parse_exchange(&source).expect("the exchange structure should parse");

    let faces: Vec<EntityId> = exchange
        .instances_of("FACE")
        .map(|instance| instance.id)
        .collect();
    assert_eq!(faces, vec![EntityId(1), EntityId(3)]);
}

#[test]
fn a_dangling_reference_is_reported_rather_than_refused() {
    // Which dangling references are fatal is a schema question, so the file
    // still parses and the broken corner is merely named.
    let source = data_section("#1 = SAMPLE(#404,(#1,#405));");
    let exchange = parse_exchange(&source).expect("the exchange structure should parse");

    let dangling = exchange.dangling_references();
    assert_eq!(dangling.len(), 2);
    assert_eq!(dangling[0].from, EntityId(1));
    assert_eq!(dangling[0].to, EntityId(404));
    assert_eq!(dangling[0].line, 5);
    assert_eq!(dangling[1].to, EntityId(405));
}

#[test]
fn a_well_formed_file_reports_no_dangling_references() {
    let source = data_section("#1 = CARTESIAN_POINT('',(0.,0.,0.));\n#2 = VERTEX_POINT('',#1);");
    let exchange = parse_exchange(&source).expect("the exchange structure should parse");

    assert!(exchange.dangling_references().is_empty());
}

#[test]
fn a_duplicate_entity_id_is_refused_and_names_both_lines() {
    // The table could not answer a reference, so this breaks the structure
    // rather than any one entity.
    let source = data_section("#1 = FIRST();\n#2 = OTHER();\n#1 = AGAIN();");
    let error = parse_exchange(&source).expect_err("a duplicate instance name should be refused");

    let SyntaxError::DuplicateEntityId(duplicate) = error else {
        panic!("expected a duplicate-id error, got {error}");
    };
    assert_eq!(duplicate.id, EntityId(1));
    assert_eq!(duplicate.first_line, 5);
    assert_eq!(duplicate.line, 7);
    assert!(duplicate.to_string().contains("#1"));
}

#[test]
fn malformed_text_names_the_line_it_failed_on() {
    // A STEP error a user cannot locate in their file is not actionable.
    let source = data_section("#1 = FIRST();\n#2 = BROKEN(1,;");
    let error = parse_exchange(&source).expect_err("an unclosed parameter list should be refused");

    let SyntaxError::Malformed {
        line,
        column,
        expected,
    } = error
    else {
        panic!("expected a malformed-text error, got {error}");
    };
    assert_eq!(line, 6);
    assert_eq!(column, 14, "the column should point at the defect itself");
    assert!(
        expected.contains(')'),
        "the message should name the defect, got {expected:?}",
    );
}

#[test]
fn an_unterminated_string_is_named_as_one() {
    // Common enough in vendor files to deserve its own message rather than a
    // confusing report about whatever the parser tried next.
    let source = data_section("#1 = SAMPLE('unterminated);");
    let error = parse_exchange(&source).expect_err("an unterminated string should be refused");

    let SyntaxError::Malformed { expected, .. } = error else {
        panic!("expected a malformed-text error, got {error}");
    };
    assert!(
        expected.contains("closing quote"),
        "the message should name the unterminated string, got {expected:?}",
    );
}

#[test]
fn a_missing_instance_terminator_is_named() {
    let source = data_section("#1 = SAMPLE()");
    let error = parse_exchange(&source).expect_err("a missing ';' should be refused");

    let SyntaxError::Malformed { expected, .. } = error else {
        panic!("expected a malformed-text error, got {error}");
    };
    assert!(
        expected.contains(';'),
        "the message should name the missing terminator, got {expected:?}",
    );
}

#[test]
fn a_missing_end_tag_is_refused() {
    let source = "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n#1 = SAMPLE();\nENDSEC;\n";
    let error = parse_exchange(source).expect_err("a truncated file should be refused");

    assert!(matches!(error, SyntaxError::Malformed { .. }));
}

#[test]
fn text_that_is_not_part_21_at_all_is_refused() {
    let error = parse_exchange("solid cube\nfacet normal 0 0 1\n")
        .expect_err("an STL file should not parse as Part 21");

    let SyntaxError::Malformed { line, .. } = error else {
        panic!("expected a malformed-text error, got {error}");
    };
    assert_eq!(line, 1);
}
