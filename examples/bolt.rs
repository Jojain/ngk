//! Profiling workload: the hex bolt of `tests/parts/bolt.rs`, built and fused
//! without opening the viewer.
//!
//! A hexagonal head, a plain body, a thinner threaded core and a helical
//! thread are fused into one solid; the thread/core Boolean dominates.

use std::f64::consts::TAU;
use std::time::Instant;

use nalgebra::Vector3;
use ngk::builders::sweep::{SweepFrame, SweepOptions};
use ngk::geometry::{Axis3, Frame, Helix, NativeParam, Point3, SolverCounters};
use ngk::modeling::solids::{cylinder_at, extruded, fuse};
use ngk::modeling::sweep::sweep_face;
use ngk::modeling::{edges, faces};
use ngk::topology::shape::{Shape, SolidTag};
use ngk::topology::validation::{validate_solid_manifold, validate_solid_orientation};
use radians::Rad64;

const HEAD_ACROSS_FLATS: f64 = 10.0;
const HEAD_HEIGHT: f64 = 4.0;
const BODY_RADIUS: f64 = 3.0;
const BODY_LENGTH: f64 = 8.0;
const CORE_RADIUS: f64 = 2.5;
const THREAD_LENGTH: f64 = 12.0;
const THREAD_PITCH: f64 = 1.25;
const THREAD_DEPTH: f64 = 0.5;
const THREAD_OVERLAP: f64 = 0.1;
const THREAD_BASE_WIDTH: f64 = 0.9;
const THREAD_MARGIN: f64 = 0.75;
const SAMPLES_PER_TURN: usize = 16;

type Solid = Shape<SolidTag>;

fn head() -> Solid {
    let circumradius = HEAD_ACROSS_FLATS / 3.0_f64.sqrt();
    let corners: Vec<Point3> = (0..6)
        .map(|i| {
            let angle = TAU * i as f64 / 6.0;
            Point3::new(circumradius * angle.cos(), circumradius * angle.sin(), 0.0)
        })
        .collect();
    let hexagon = faces::polygon(&corners).expect("hexagon should build");
    extruded(hexagon, Vector3::z_axis(), HEAD_HEIGHT).expect("head should extrude")
}

fn body() -> Solid {
    cylinder_at(
        Frame::at(Point3::new(0.0, 0.0, -BODY_LENGTH)),
        BODY_RADIUS,
        BODY_LENGTH,
    )
    .expect("body should build")
}

fn core() -> Solid {
    cylinder_at(
        Frame::at(Point3::new(0.0, 0.0, -BODY_LENGTH - THREAD_LENGTH)),
        CORE_RADIUS,
        THREAD_LENGTH,
    )
    .expect("core should build")
}

fn thread() -> Solid {
    let bottom = -BODY_LENGTH - THREAD_LENGTH + THREAD_MARGIN;
    let turns = (THREAD_LENGTH - 2.0 * THREAD_MARGIN) / THREAD_PITCH;
    let axis = Axis3::new(Point3::new(0.0, 0.0, bottom), Vector3::z());
    let spine = edges::helix(
        axis,
        CORE_RADIUS,
        THREAD_PITCH,
        Rad64::new(0.0),
        Rad64::new(TAU * turns),
    )
    .expect("thread helix should build");

    let start = Helix::from_axis(axis, CORE_RADIUS, THREAD_PITCH).point_at(NativeParam::new(0.0));
    let outward = (start - axis.origin).normalize();
    let along = *axis.direction;

    let base = start - outward * THREAD_OVERLAP;
    let half = THREAD_BASE_WIDTH / 2.0;
    let profile = faces::polygon(&[
        base - along * half,
        start + outward * THREAD_DEPTH,
        base + along * half,
    ])
    .expect("thread profile should build");

    sweep_face(
        profile,
        &spine.edge(),
        SweepOptions {
            frame: SweepFrame::Axial(axis),
            samples_per_segment: (turns * SAMPLES_PER_TURN as f64).ceil() as usize,
            ..SweepOptions::default()
        },
    )
    .expect("thread should sweep")
}

fn main() {
    let counters = SolverCounters::snapshot();
    let started = Instant::now();
    let bolt = fuse(head(), body()).expect("head and body should fuse");
    let bolt = fuse(bolt, core()).expect("body and core should fuse");
    let bolt = fuse(bolt, thread()).expect("thread should fuse onto the core");
    let elapsed = started.elapsed();
    let counters = SolverCounters::snapshot().since(counters);

    validate_solid_orientation(bolt.model(), bolt.key()).expect("bolt should face outward");
    validate_solid_manifold(bolt.model(), bolt.key()).expect("bolt should be manifold");
    let volume = bolt
        .solid()
        .volume_properties()
        .expect("bolt volume should be measurable")
        .volume;
    println!("bolt built in {elapsed:.2?}, volume {volume:.6}");
    println!(
        "surface projections: {} global, {} hinted; branch fits {}",
        counters.global_surface_projections,
        counters.hinted_surface_projections,
        counters.branch_fits
    );
}
