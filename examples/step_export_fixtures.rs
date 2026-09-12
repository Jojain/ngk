//! Writes the STEP files that `tests/fixtures/step/validate_ngk_export.py`
//! checks against OpenCascade.
//!
//! The Rust tests in `tests/exchange/step_export.rs` assert the structure NGK
//! emits; they cannot assert that another kernel accepts it. Run this, then
//! the Python script, to close that gap:
//!
//! ```text
//! cargo run --example step_export_fixtures
//! uv run python tests/fixtures/step/validate_ngk_export.py
//! ```

use std::fs;

use nalgebra::Vector3;
use ngk::builders::solids::add_extruded_face;
use ngk::exchange::step::part21::exchange_to_string;
use ngk::exchange::step::{StepWriteOptions, map_to_exchange, write_step_file};
use ngk::geometry::{Plane, Point3};
use ngk::modeling::{faces, solids};

const OUT_DIR: &str = "target/step_export";

fn main() {
    fs::create_dir_all(OUT_DIR).expect("output directory should be creatable");

    let block = solids::block(10.0, 20.0, 30.0).expect("a block should build");
    write_step_file(
        format!("{OUT_DIR}/block.step"),
        &block,
        &StepWriteOptions::named("BLOCK"),
    )
    .expect("block should export");
    println!("  {OUT_DIR}/block.step");

    write("holed_slab.step", holed_slab());

    println!("wrote fixtures to {OUT_DIR}/");
}

/// A 4 × 3 × 3 slab with a 1 × 1 hole through it, all faces planar.
fn holed_slab() -> String {
    let outer = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(4.0, 0.0, 0.0),
        Point3::new(4.0, 3.0, 0.0),
        Point3::new(0.0, 3.0, 0.0),
    ];
    let hole = [
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(1.0, 2.0, 0.0),
        Point3::new(2.0, 2.0, 0.0),
        Point3::new(2.0, 1.0, 0.0),
    ];
    let profile = faces::polygon_with_holes(Plane::xy(), &outer, &[&hole])
        .expect("a holed face should build");
    let (mut map, face) = profile.into_map();
    let solid =
        add_extruded_face(&mut map, face, Vector3::new(0.0, 0.0, 3.0)).expect("it should extrude");

    let exchange = map_to_exchange(&map, &[solid], &StepWriteOptions::named("HOLED_SLAB"))
        .expect("a planar solid should export");
    exchange_to_string(&exchange).expect("the exchange structure should write")
}

fn write(name: &str, text: String) {
    let path = format!("{OUT_DIR}/{name}");
    fs::write(&path, text).expect("fixture should be writable");
    println!("  {path}");
}
