//! Cuts a grid of holes through a plate two ways and reports how long each took.
//!
//! The same `n x n` cylinders are subtracted once in a single call, which
//! imprints every tool and decides the result once, and once in a loop of
//! single-tool cuts, which re-decides the growing plate at every step. Both
//! answers are checked to agree on volume.
//!
//! ```text
//! cargo run --release --example boolean_many_tools -- 5
//! ```

use std::time::Instant;

use nalgebra::Vector3;
use ngk::geometry::{Frame, Point3};
use ngk::modeling::solids;
use ngk::topology::shape::{Shape, SolidTag};

fn plate(n: usize) -> Shape<SolidTag> {
    let size = 10.0 * n as f64 + 10.0;
    solids::block(size, size, 10.0).expect("plate")
}

fn holes(n: usize) -> Vec<Shape<SolidTag>> {
    (0..n)
        .flat_map(|i| (0..n).map(move |j| (i, j)))
        .map(|(i, j)| {
            let frame = Frame::from_xz(
                Point3::new(10.0 + 10.0 * i as f64, 10.0 + 10.0 * j as f64, -1.0),
                Vector3::x(),
                Vector3::z(),
            );
            solids::cylinder_at(frame, 2.0, 12.0).expect("hole")
        })
        .collect()
}

fn main() {
    let n = std::env::args()
        .nth(1)
        .map(|arg| arg.parse().expect("grid size"))
        .unwrap_or(5);
    let tools = holes(n);

    let started = Instant::now();
    let at_once = solids::cut(plate(n), &tools).expect("one call");
    let one_call = started.elapsed();

    let started = Instant::now();
    let mut stepped = plate(n);
    for tool in &tools {
        stepped = solids::cut(stepped, [tool]).expect("one step");
    }
    let loop_time = started.elapsed();

    let volume = |shape: &Shape<SolidTag>| shape.solid().volume().expect("volume");
    let (at_once, stepped) = (volume(&at_once), volume(&stepped));
    println!(
        "{} holes: one call {one_call:?}, {}-step loop {loop_time:?}",
        tools.len(),
        tools.len()
    );
    assert!(
        (at_once - stepped).abs() <= 1e-9 * at_once,
        "the two answers disagree: one call {at_once}, loop {stepped}"
    );
}
