//! Every Boolean terminates, whether or not it succeeds.
//!
//! The searches underneath a Boolean are subdivision searches, and one without
//! a node budget can visit a tree deep enough that it never returns on a
//! configuration nobody anticipated. These tests assert only termination and a
//! typed outcome; what that outcome *is* belongs in `boolean.rs`.

use std::sync::mpsc;
use std::thread;
use std::time::Duration;

use nalgebra::Vector3;
use ngk::builders::boolean::{BooleanError, BooleanOperation, BooleanOptions, boolean};
use ngk::geometry::{Frame, Point3};
use ngk::model::Model;
use ngk::modeling::solids;
use ngk::topology::ModelEditError;
use ngk::topology::shape::{Shape, SolidTag};
use ngk::topology::shape_keys::SolidKey;

/// Generous enough that only a non-terminating search trips it.
const CEILING: Duration = Duration::from_secs(60);

type Operands = (Model<ngk::StandardPayload>, SolidKey, SolidKey);

fn merged(
    target: Shape<SolidTag, ngk::StandardPayload>,
    tool: Shape<SolidTag, ngk::StandardPayload>,
) -> Operands {
    let (tool_map, tool_key) = tool.into_model();
    let (mut map, target_key) = target.into_model();
    let imported = map
        .transaction(|edit| {
            let handle = edit.merge(tool_map.solid_unchecked(tool_key));
            Ok::<_, ModelEditError>(edit.solid_key_at(handle).expect("imported tool solid"))
        })
        .expect("import tool operand");
    (map, target_key, imported)
}

/// Runs one Boolean on its own thread and fails if it outlives [`CEILING`].
///
/// A thread over the ceiling is abandoned rather than joined: it is inside a
/// search with no interruption point, and the test has already failed.
fn terminates(name: &str, build: fn() -> Operands, operation: BooleanOperation) {
    let (sender, receiver) = mpsc::channel();
    thread::spawn(move || {
        let (mut map, first, second) = build();
        let outcome = boolean(
            &mut map,
            first,
            second,
            operation,
            BooleanOptions::default(),
        );
        let _ = sender.send(match outcome {
            Ok(_) => "ok".to_string(),
            Err(BooleanError::IncompleteIntersections { diagnostics }) => {
                format!("incomplete: {:?}", diagnostics.coverage)
            }
            Err(error) => format!("error: {error}"),
        });
    });
    match receiver.recv_timeout(CEILING) {
        Ok(outcome) => println!("{name}: {outcome}"),
        Err(_) => panic!("{name} did not terminate within {CEILING:?}"),
    }
}

fn block_and_sphere() -> Operands {
    // The original repro: a block whose own corner is the sphere's centre, so
    // three of its planes cut the sphere through a pole and along its seam.
    merged(
        solids::block(10.0, 20.0, 30.0).expect("block"),
        solids::sphere(6.0).expect("sphere"),
    )
}

fn block_and_offset_sphere() -> Operands {
    merged(
        solids::block(10.0, 20.0, 30.0).expect("block"),
        solids::sphere_at(
            Frame::from_xy(Point3::new(5.0, 10.0, 0.0), Vector3::x(), Vector3::y()),
            3.0,
        )
        .expect("sphere"),
    )
}

fn overlapping_spheres() -> Operands {
    merged(
        solids::sphere(1.0).expect("sphere"),
        solids::sphere_at(
            Frame::from_xy(Point3::new(1.2, 0.0, 0.0), Vector3::x(), Vector3::y()),
            1.0,
        )
        .expect("sphere"),
    )
}

fn orthogonal_cylinders() -> Operands {
    merged(
        solids::cylinder_at(
            Frame::from_xy(Point3::new(0.0, 0.0, -2.0), Vector3::x(), Vector3::y()),
            1.0,
            4.0,
        )
        .expect("upright cylinder"),
        solids::cylinder_at(
            Frame::from_xy(Point3::new(-2.0, 0.0, 0.0), Vector3::y(), Vector3::z()),
            0.6,
            4.0,
        )
        .expect("crossing cylinder"),
    )
}

#[test]
fn block_united_with_a_sphere_terminates() {
    terminates("block u sphere", block_and_sphere, BooleanOperation::Union);
}

#[test]
fn block_cut_by_a_sphere_terminates() {
    terminates(
        "block - sphere",
        block_and_sphere,
        BooleanOperation::Difference,
    );
}

#[test]
fn block_united_with_an_offset_sphere_terminates() {
    terminates(
        "block u offset sphere",
        block_and_offset_sphere,
        BooleanOperation::Union,
    );
}

#[test]
fn overlapping_spheres_terminate() {
    terminates(
        "sphere u sphere",
        overlapping_spheres,
        BooleanOperation::Union,
    );
}

#[test]
fn orthogonal_cylinders_terminate() {
    terminates(
        "cylinder u cylinder",
        orthogonal_cylinders,
        BooleanOperation::Union,
    );
}
