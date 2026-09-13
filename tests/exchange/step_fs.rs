//! Reading and writing STEP files on disk.
//!
//! The layers below work on `&str` and `impl Write` so the feature builds for
//! wasm; these wrappers are the one place a path appears. What they add beyond
//! the bytes is the path in the error, so a failure is something a user can
//! act on.

use std::fs;
use std::path::PathBuf;

use ngk::exchange::step::{
    StepError, StepReadOptions, StepWriteOptions, read_exchange_file, read_step, read_step_file,
    write_step_file,
};
use ngk::modeling::solids;

/// A unique path under the system temp directory, removed if it lingers.
fn scratch(name: &str) -> PathBuf {
    let path = std::env::temp_dir().join(format!("ngk_step_test_{name}"));
    let _ = fs::remove_file(&path);
    path
}

/// The committed fixture written by OpenCascade, via build123d.
const BOX_STEP: &str = "tests/exchange/foreign/files/box.step";

#[test]
fn a_solid_written_to_a_file_reads_back_as_the_same_document() {
    let path = scratch("round_trip.step");
    let block = solids::block(10.0, 20.0, 30.0).expect("a block should build");

    write_step_file(&path, &block, &StepWriteOptions::named("BLOCK"))
        .expect("a planar solid should write");

    let exchange = read_exchange_file(&path).expect("the written file should parse");
    assert_eq!(exchange.instances_of("ADVANCED_FACE").count(), 6);
    assert_eq!(exchange.instances_of("MANIFOLD_SOLID_BREP").count(), 1);
    assert!(exchange.dangling_references().is_empty());

    fs::remove_file(&path).expect("the scratch file should be removable");
}

#[test]
fn a_write_that_fails_names_the_file_it_failed_on() {
    // "No such file or directory" without a path is not actionable.
    let path = scratch("no_such_directory/block.step");
    let block = solids::block(1.0, 1.0, 1.0).expect("a block should build");

    let error = write_step_file(&path, &block, &StepWriteOptions::default())
        .expect_err("writing into a missing directory should fail");

    let StepError::Io {
        path: reported,
        source: _,
    } = &error
    else {
        panic!("expected an I/O error, got {error}");
    };
    assert_eq!(reported, &path);
    assert!(
        error.to_string().contains("block.step"),
        "the message should name the file, got {error}",
    );
}

#[test]
fn a_read_that_fails_names_the_file_it_failed_on() {
    let path = scratch("absent.step");

    let error =
        read_exchange_file(&path).expect_err("reading a file that is not there should fail");

    assert!(matches!(error, StepError::Io { .. }), "got {error}");
    assert!(error.to_string().contains("absent.step"), "got {error}");
}

#[test]
fn a_foreign_file_can_be_opened_and_inspected_without_being_imported() {
    // The two entry points stay separate: answering "what is actually in
    // this file?" needs L1 alone, and must not depend on every entity in it
    // being one the mapping layers can carry.
    let exchange = read_exchange_file(BOX_STEP).expect("the fixture should parse");

    let solids: Vec<_> = exchange.instances_of("MANIFOLD_SOLID_BREP").collect();
    assert_eq!(solids.len(), 1);
    assert!(solids[0].line > 0, "an instance should know its own line");
    assert_eq!(exchange.instances_of("ADVANCED_FACE").count(), 6);
}

#[test]
fn importing_a_well_formed_file_yields_its_solids() {
    // The path all the way from a path on disk to a sewn map, which is the
    // only test here that crosses every layer at once.
    let import =
        read_step_file(BOX_STEP, &StepReadOptions::default()).expect("a planar file should import");

    assert_eq!(import.shapes.len(), 1);
    assert_eq!(import.shapes[0].solid().faces().len(), 6);
}

#[test]
fn importing_a_malformed_file_reports_the_syntax_error_with_its_line() {
    // A broken file is diagnosed where it broke rather than at whatever the
    // next layer made of the wreckage.
    let error = read_step("this is not a STEP file\n", &StepReadOptions::default())
        .expect_err("malformed text should be refused");

    let StepError::Syntax(syntax) = error else {
        panic!("expected a syntax error, got {error}");
    };
    assert!(syntax.to_string().contains("line 1"), "got {syntax}");
}

#[test]
fn read_options_default_to_healing_seams_and_reporting_rather_than_failing() {
    // The defaults encode two decisions: an imported seam is not part of the
    // shape and comes off, and one bad face should not cost the file.
    let options = StepReadOptions::default();
    assert!(options.heal_seams);
    assert!(!options.strict);
    assert_eq!(options.uncertainty, None);

    assert!(!StepReadOptions::faithful().heal_seams);
    assert!(StepReadOptions::strict().strict);
}
