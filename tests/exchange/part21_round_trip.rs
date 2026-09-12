//! Text → table → text, and back again.
//!
//! Round trips are checked by invariant rather than by text diff: the file the
//! writer emits is not required to be byte-identical to the one the reader was
//! given (whitespace, line breaks and `1.0` versus `1.` are all free), only to
//! carry the same exchange structure. Reparsing and comparing the tables is
//! what states that.

use ngk::exchange::step::part21::{
    EntityId, Value, decode_text, encode_text, exchange_to_string, parse_exchange,
};

/// A small but representative document: every value kind, a complex instance,
/// nested aggregates, references, and a header.
const SAMPLE: &str = "\
ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('a solid'),'2;1');
FILE_NAME('block.step','2026-09-12T10:00:00',('Jojain'),(''),'ngk','ngk','');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));
ENDSEC;
DATA;
#1 = CARTESIAN_POINT('origin',(0.,0.,0.));
#2 = DIRECTION('',(0.,0.,1.));
#3 = DIRECTION('',(1.,0.,0.));
#4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);
#5 = PLANE('',#4);
#6 = VERTEX_POINT('it''s here',#1);
#7 = CIRCLE('',#4,2.5);
#8 = TRIMMED_CURVE('',#7,(PARAMETER_VALUE(0.)),(PARAMETER_VALUE(1.E-7)),.T.,.PARAMETER.);
#9 = SAMPLE($,*,(1,2,3),((1.,2.),(3.,4.)),'caf\\X2\\00E9\\X0\\');
#10 = (NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) LENGTH_UNIT());
ENDSEC;
END-ISO-10303-21;
";

#[test]
fn a_document_survives_being_written_and_read_back() {
    let first = parse_exchange(SAMPLE).expect("the sample should parse");
    let written = exchange_to_string(&first).expect("the sample should write");
    let second = parse_exchange(&written).expect("written text should parse");

    assert_eq!(
        first.header(),
        second.header(),
        "the header changed across the round trip",
    );
    assert_eq!(
        first.instances().len(),
        second.instances().len(),
        "the instance count changed across the round trip",
    );
    for (before, after) in first.instances().iter().zip(second.instances()) {
        assert_eq!(before.id, after.id);
        assert_eq!(
            before.records, after.records,
            "instance {} changed across the round trip",
            before.id,
        );
    }
}

#[test]
fn a_document_is_stable_after_the_first_round_trip() {
    // The second and third texts must be byte-identical: the writer has one
    // spelling per value, so anything that keeps changing is a bug.
    let first = parse_exchange(SAMPLE).expect("the sample should parse");
    let once = exchange_to_string(&first).expect("the sample should write");
    let twice = exchange_to_string(&parse_exchange(&once).expect("written text should parse"))
        .expect("written text should write");

    assert_eq!(once, twice);
}

#[test]
fn the_round_trip_preserves_every_value_kind() {
    let exchange = parse_exchange(
        &exchange_to_string(&parse_exchange(SAMPLE).expect("the sample should parse"))
            .expect("the sample should write"),
    )
    .expect("written text should parse");

    let sample = exchange
        .get(EntityId(9))
        .expect("#9")
        .simple()
        .expect("simple");
    assert!(sample.params[0].is_null());
    assert!(sample.params[1].is_derived());
    assert_eq!(
        sample.params[2],
        Value::List(vec![
            Value::Integer(1),
            Value::Integer(2),
            Value::Integer(3),
        ]),
    );
    assert_eq!(
        sample.params[4].as_text().expect("a string"),
        "café",
        "the escape decoded, re-encoded and decoded again must be the same text",
    );

    let trimmed = exchange
        .get(EntityId(8))
        .expect("#8")
        .simple()
        .expect("simple");
    assert_eq!(trimmed.params[1].as_reference(), Some(EntityId(7)));
    assert_eq!(trimmed.params[4].as_enum(), Some("T"));
    let trim = trimmed.params[3].as_list().expect("a list")[0]
        .as_typed()
        .expect("a typed parameter");
    assert!(trim.is("PARAMETER_VALUE"));
    assert_eq!(trim.params[0], Value::Real(1.0e-7));
}

#[test]
fn a_complex_instance_survives_the_round_trip_as_a_complex_instance() {
    let exchange = parse_exchange(
        &exchange_to_string(&parse_exchange(SAMPLE).expect("the sample should parse"))
            .expect("the sample should write"),
    )
    .expect("written text should parse");

    let unit = exchange.get(EntityId(10)).expect("#10");
    assert_eq!(unit.records.len(), 3);
    assert!(unit.simple().is_none());
    assert!(unit.is("SI_UNIT") && unit.is("LENGTH_UNIT") && unit.is("NAMED_UNIT"));
}

#[test]
fn every_string_survives_encoding_and_decoding() {
    let awkward = [
        "",
        "plain",
        "it's here",
        "two '' quotes",
        r"C:\temp\file",
        "café",
        "Äü ß",
        "😀 mixed 漢字 text",
        "trailing backslash \\",
        "  leading and trailing  ",
    ];

    for text in awkward {
        assert_eq!(
            decode_text(&encode_text(text)),
            text,
            "{text:?} did not survive encoding",
        );
    }
}

#[test]
fn a_string_survives_a_round_trip_through_a_document() {
    for text in ["it's here", r"C:\temp", "café 😀", "a\nb"] {
        let source = format!(
            "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n#1 = SAMPLE('{}');\nENDSEC;\n\
             END-ISO-10303-21;\n",
            encode_text(text),
        );
        let exchange = parse_exchange(&source).expect("the document should parse");
        let written = exchange_to_string(&exchange).expect("the document should write");
        let reparsed = parse_exchange(&written).expect("written text should parse");

        assert_eq!(
            reparsed
                .get(EntityId(1))
                .expect("#1")
                .simple()
                .expect("simple")
                .params[0]
                .as_text()
                .expect("a string"),
            text,
        );
    }
}
