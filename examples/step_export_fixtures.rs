//! Writes NGK's STEP output to `target/step_export/` for a foreign kernel to
//! check.
//!
//! The Rust tests in `tests/exchange/step_export.rs` assert the structure NGK
//! emits; they cannot assert that another kernel accepts it. Run this and open
//! the files in any OCCT-based tool to close that gap.
//!
//! Two of the files are *re-exports* of the foreign fixtures under
//! `tests/exchange/foreign/files/`, which is how the read direction gets
//! checked too: `tests/exchange/step_import.rs` can show that a foreign file
//! yields a map with the right cells and a valid orientation, but only another
//! kernel can say the result still has the right volume.
//!
//! ```text
//! cargo run --example step_export_fixtures
//! ```

use std::fs;

use nalgebra::Vector3;
use ngk::builders::solids::{add_extruded_face, add_sphere};
use ngk::exchange::step::part21::exchange_to_string;
use ngk::exchange::step::{
    StepReadOptions, StepWriteOptions, map_to_exchange, read_step_file, step_to_string,
    write_step_file,
};
use ngk::geometry::{Frame, Plane, Point3};
use ngk::modeling::{faces, solids};
use ngk::topology::attributes::ShellRoot;
use ngk::topology::gmap::GMap;
use ngk::topology::orientation::Orientation;
use ngk::topology::{StandardPayload, TopologyEditError};

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

    // The seam case. Nothing in the map carries the cut its wall is written
    // along, so only another kernel can say the cut came out on the surface
    // and welded back together.
    let cylinder = solids::cylinder(5.0, 10.0).expect("a cylinder should build");
    write(
        "cylinder.step",
        step_to_string(&cylinder, &StepWriteOptions::named("CYLINDER"))
            .expect("a cylinder should export"),
    );

    // The boundaryless cases. Neither shape has a single edge or corner in the
    // map, so every line of these files was synthesized from the support's own
    // domain, and only another kernel can say that what came out closes.
    let sphere = solids::sphere(5.0).expect("a sphere should build");
    write(
        "sphere.step",
        step_to_string(&sphere, &StepWriteOptions::named("SPHERE"))
            .expect("a sphere should export"),
    );
    let torus = solids::torus(3.0, 1.0).expect("a torus should build");
    write(
        "torus.step",
        step_to_string(&torus, &StepWriteOptions::named("TORUS")).expect("a torus should export"),
    );

    // The void case. A cavity written as a second outer shell reads back as a
    // solid ball with a ghost sphere in it, which is a perfectly valid file of
    // the wrong shape — so the volume is the assertion that matters.
    write("hollow_sphere.step", hollow_sphere(5.0, 2.0));

    // The spline case, and the only shape here NGK cannot build any other
    // way: cutting one block with another leaves every support planar and most
    // of the edges free-form, because an imprint's section is fitted rather
    // than recognized.
    let block = solids::block(10.0, 10.0, 10.0).expect("a block should build");
    let tool = solids::block(4.0, 4.0, 30.0).expect("a tool should build");
    let cut = solids::cut(block, tool).expect("a through cut should build");
    write(
        "cut_block.step",
        step_to_string(&cut, &StepWriteOptions::named("CUT_BLOCK"))
            .expect("a spline-edged solid should export"),
    );

    // The read direction, checked the only way it can be from outside: take a
    // file OpenCascade wrote, import it, and write it back. If the volume
    // survives that, the import understood the file rather than merely
    // producing a map that satisfies our own validators.
    reexport("box.step", "reexported_box.step");
    reexport("cylinder.step", "reexported_cylinder.step");
    reexport("frustum.step", "reexported_frustum.step");
    reexport("sphere.step", "reexported_sphere.step");
    reexport("torus.step", "reexported_torus.step");
    reexport("lofted.step", "reexported_lofted.step");
    reexport("swept_circle.step", "reexported_swept_circle.step");
    reexport("holed_slab.step", "reexported_holed_slab.step");

    println!("wrote fixtures to {OUT_DIR}/");
}

/// Imports a committed OpenCascade fixture and writes it out again.
fn reexport(fixture: &str, name: &str) {
    let import = read_step_file(
        format!("tests/exchange/foreign/files/{fixture}"),
        &StepReadOptions::default(),
    )
    .unwrap_or_else(|error| panic!("{fixture} should import: {error}"));

    let [shape] = import.shapes.as_slice() else {
        panic!("{fixture} should hold exactly one solid");
    };
    let text = step_to_string(shape, &StepWriteOptions::named("REEXPORTED"))
        .unwrap_or_else(|error| panic!("{fixture} should re-export: {error}"));
    write(name, text);
}

/// A sphere of radius `outer` with a concentric cavity of radius `inner`.
///
/// No builder makes a hollow solid, so the cavity's shell is registered by
/// hand. It is the same spherical support the outer shell would use, read the
/// other way round: every shell bounds the material from outside it, which for
/// a cavity means facing into itself.
fn hollow_sphere(outer: f64, inner: f64) -> String {
    let mut map = GMap::<StandardPayload>::new();
    let solid = add_sphere(&mut map, Frame::xyz(), outer).expect("an outer sphere should build");
    let cavity = add_sphere(&mut map, Frame::xyz(), inner).expect("a cavity sphere should build");
    let cavity_face = map
        .solid(cavity)
        .expect("the cavity is registered")
        .outer_shell()
        .faces()
        .first()
        .expect("a shell has a face")
        .key();

    map.transaction(|edit| {
        edit.remove_solid(cavity);
        let void = ShellRoot::Face {
            face: cavity_face,
            sense: Orientation::Reversed,
        };
        let sheet = edit
            .map()
            .sheet_key_at_face(cavity_face)
            .expect("the cavity's face is registered as a sheet");
        edit.sheet_attr_mut_unchecked(sheet).root = void;
        edit.solid_attr_mut_unchecked(solid).inner_shells = Some(vec![void]);
        Ok::<_, TopologyEditError>(())
    })
    .expect("a hollow sphere should commit");

    let exchange = map_to_exchange(&map, &[solid], &StepWriteOptions::named("HOLLOW_SPHERE"))
        .expect("a hollow sphere should export");
    exchange_to_string(&exchange).expect("the exchange structure should write")
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
