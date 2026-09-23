//! A hex bolt: a hexagonal head, a plain cylindrical body, a thinner threaded
//! core and a helical thread, fused into one solid.
//!
//! The bolt stands on its head: the head's underside sits on `z = 0` and the
//! shank runs down the negative z axis. Head, body and core meet flush on
//! shared planes, the way they would be sketched. The thread is a triangular
//! profile drawn in a plane through the bolt's axis and screwed along a helix,
//! its base sunk into the core so that the two overlap in volume.

use std::f64::consts::TAU;

use nalgebra::Vector3;
use ngk::builders::sweep::{SweepFrame, SweepOptions};
use ngk::geometry::{Axis3, Frame, Helix, NativeParam, Point3};
use ngk::modeling::solids::{cylinder_at, extruded, fuse, intersect};
use ngk::modeling::sweep::sweep_face;
use ngk::modeling::{edges, faces};
use ngk::topology::shape::{Shape, SolidTag};
use ngk::topology::validation::{validate_solid_manifold, validate_solid_orientation};
use ngk::viz::debug_viewer::show;
use radians::Rad64;

/// Distance across the head's flats.
const HEAD_ACROSS_FLATS: f64 = 10.0;
const HEAD_HEIGHT: f64 = 4.0;
const BODY_RADIUS: f64 = 3.0;
const BODY_LENGTH: f64 = 8.0;
const CORE_RADIUS: f64 = 2.5;
const THREAD_LENGTH: f64 = 12.0;
const THREAD_PITCH: f64 = 1.25;
/// Radial reach of the thread's crest beyond the core.
const THREAD_DEPTH: f64 = 0.5;
/// How far the thread's base sinks into the core.
const THREAD_OVERLAP: f64 = 0.1;
/// Width of the thread profile at its base, along the axis.
const THREAD_BASE_WIDTH: f64 = 0.9;
/// Axial margin left unthreaded at each end of the core.
const THREAD_MARGIN: f64 = 0.75;
const SAMPLES_PER_TURN: usize = 16;

type Solid = Shape<SolidTag>;

/// A hexagonal prism standing on `z = 0`.
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

/// The plain shank, flush under the head.
fn body() -> Solid {
    cylinder_at(
        Frame::at(Point3::new(0.0, 0.0, -BODY_LENGTH)),
        BODY_RADIUS,
        BODY_LENGTH,
    )
    .expect("body should build")
}

/// The thinner cylinder the thread winds around, flush under the body.
fn core() -> Solid {
    cylinder_at(
        Frame::at(Point3::new(0.0, 0.0, -BODY_LENGTH - THREAD_LENGTH)),
        CORE_RADIUS,
        THREAD_LENGTH,
    )
    .expect("core should build")
}

/// A triangular thread profile screwed along a helix on the core's surface.
///
/// The profile stands in a plane through the axis, as a thread is drawn, and
/// the axial sweep keeps it in one all the way along.
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

    // The support `edges::helix` builds, read directly for where it starts.
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

/// A solid's volume, measured on its tessellation.
///
/// The measure is approximate on curved faces, so volumes are compared with
/// the measured volumes of the parts rather than with closed forms: the
/// fused solid meshes the same surfaces its parts did.
fn volume(solid: &Solid) -> f64 {
    solid
        .solid()
        .volume_properties()
        .expect("solid volume should be measurable")
        .volume
}

fn assert_valid(solid: &Solid) {
    validate_solid_orientation(solid.model(), solid.key()).expect("solid should face outward");
    validate_solid_manifold(solid.model(), solid.key()).expect("solid should be manifold");
    assert_eq!(
        solid.solid().shells().len(),
        1,
        "a bolt has one boundary shell"
    );
}

fn assert_close(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() <= 1e-3 * expected,
        "expected {expected}, got {actual} (error {})",
        (actual - expected).abs()
    );
}

#[test]
fn hex_head_fuses_flush_onto_the_body() {
    let expected = volume(&head()) + volume(&body());
    let fused = fuse(head(), body()).expect("head and body should fuse");

    assert_valid(&fused);
    assert_close(volume(&fused), expected);
}

#[test]
fn stepped_shank_fuses_flush_between_two_coaxial_cylinders() {
    let expected = volume(&body()) + volume(&core());
    let fused = fuse(body(), core()).expect("body and core should fuse");

    assert_valid(&fused);
    assert_close(volume(&fused), expected);
}

#[test]
fn thread_fuses_onto_the_core() {
    show(&thread());
    let expected = volume(&core()) + volume(&thread());
    let common = volume(&intersect(core(), thread()).expect("core and thread should intersect"));
    let fused = fuse(core(), thread()).expect("core and thread should fuse");
    show(&fused);
    assert_valid(&fused);
    assert!(common > 0.0, "the thread's base is sunk into the core");
    assert_close(volume(&fused) + common, expected);
}

#[test]
fn bolt_is_one_closed_solid() {
    let bolt = fuse(head(), body()).expect("head and body should fuse");
    let bolt = fuse(bolt, core()).expect("body and core should fuse");
    let bolt = fuse(bolt, thread()).expect("thread should fuse onto the core");
    show(&bolt);

    assert_valid(&bolt);
    let plain = volume(&head()) + volume(&body()) + volume(&core());
    let bolt_volume = volume(&bolt);
    assert!(
        bolt_volume > plain && bolt_volume < plain + volume(&thread()),
        "the thread adds volume outside the core: {bolt_volume} against {plain}"
    );
}

#[test]
fn bolt_meshes_with_every_edge_lying_on_its_faces() {
    // What the viewer draws: each edge as a polyline over its faces' meshes.
    // An edge the faces sample elsewhere sinks into them between samples and
    // leaves a gap between two neighbouring faces. The Boolean's traced edges
    // carry fitted pcurves, so they meet their faces to the fit, not exactly.
    use ngk::tessellate::{
        CurveOpts, SurfaceOpts, TessellateOpts, tessellate_edge, tessellate_face_key,
    };
    let opts = TessellateOpts {
        curve: CurveOpts { segments: 64 },
        surface: SurfaceOpts { nu: 64, nv: 32 },
    };
    let bolt = fuse(
        fuse(fuse(head(), body()).unwrap(), core()).unwrap(),
        thread(),
    )
    .unwrap();
    let model = bolt.model();
    for (face, _) in model.iter_faces() {
        let mesh = tessellate_face_key(model, face, opts).expect("every face should mesh");
        for edge in model.face_unchecked(face).edges() {
            let line = tessellate_edge(model, edge.key(), opts).expect("every edge should sample");
            for point in &line.points {
                let nearest = mesh
                    .positions
                    .iter()
                    .map(|vertex| (vertex - point).norm())
                    .fold(f64::INFINITY, f64::min);
                assert!(
                    nearest <= 1e-3,
                    "{point:?} on edge {:?} is {nearest} from face {face:?}'s mesh",
                    edge.key()
                );
            }
        }
    }
}

#[test]
fn bolt_survives_a_step_round_trip() {
    // Written out, read back, and still the same bolt: one closed solid with
    // the volume it left with.
    use ngk::exchange::step::{StepReadOptions, StepWriteOptions, read_step_file, write_step_file};

    let bolt = fuse(
        fuse(fuse(head(), body()).unwrap(), core()).unwrap(),
        thread(),
    )
    .unwrap();
    let path = std::env::temp_dir().join("ngk_part_bolt.step");
    write_step_file(&path, &bolt, &StepWriteOptions::named("BOLT")).expect("the bolt should write");

    let import = read_step_file(&path, &StepReadOptions::default()).expect("the file should read");
    // The traced helical edges travel as bare NURBS curves, so their pcurves
    // are fitted again on the way in; that is reported, and must stay small.
    // Nothing else may be given up.
    for skip in &import.report.skipped {
        assert!(
            matches!(
                skip.reason,
                ngk::exchange::step::ImportSkipReason::ApproximatedPcurve { deviation }
                    if deviation <= 1e-3
            ),
            "{skip:?}"
        );
    }
    assert_eq!(import.shapes.len(), 1, "the file holds one solid");
    let read = &import.shapes[0];
    assert_valid(read);
    assert_close(volume(read), volume(&bolt));
}

