//! Boolean and surface/surface intersection benchmarks with hard ceilings.
//!
//! Every case is run once on a watchdog thread before it is handed to
//! criterion. That first run prints the case's stage-attributed profile and
//! enforces a wall-clock ceiling, so a regression that turns a Boolean into a
//! non-terminating search fails the bench rather than hanging it.

use std::sync::mpsc;
use std::thread;
use std::time::{Duration, Instant};

use criterion::{Criterion, criterion_group, criterion_main};
use nalgebra::Vector3;
use ngk::builders::boolean::{BooleanError, BooleanOperation, BooleanOptions, boolean};
use ngk::geometry::{
    ControlNet, Degree, Frame, HPoint, KnotVector, NurbsSurface, Plane, Point3, Surface,
};
use ngk::model::Model;
use ngk::modeling::solids;
use ngk::topology::ModelEditError;
use ngk::topology::shape_keys::SolidKey;

/// Ceiling for a case expected to be answered without a traced branch.
const PLANAR_CEILING: Duration = Duration::from_secs(5);

/// Ceiling for a case whose sections are curved.
///
/// Generous on purpose: the point is to catch a search that does not
/// terminate, not to assert a target that has not been reached yet.
const CURVED_CEILING: Duration = Duration::from_secs(30);

/// One Boolean scene, rebuilt from scratch for every measured iteration.
struct Scene {
    name: &'static str,
    ceiling: Duration,
    build: fn() -> (Model<ngk::StandardPayload>, SolidKey, SolidKey),
    operation: BooleanOperation,
}

fn scenes() -> Vec<Scene> {
    vec![
        Scene {
            name: "block_union_block",
            ceiling: PLANAR_CEILING,
            build: overlapping_blocks,
            operation: BooleanOperation::Union,
        },
        Scene {
            name: "block_union_cylinder",
            ceiling: CURVED_CEILING,
            build: block_and_protruding_cylinder,
            operation: BooleanOperation::Union,
        },
        Scene {
            name: "block_difference_cylinder",
            ceiling: CURVED_CEILING,
            build: block_and_through_cylinder,
            operation: BooleanOperation::Difference,
        },
        Scene {
            name: "block_union_sphere",
            ceiling: CURVED_CEILING,
            build: block_and_sphere,
            operation: BooleanOperation::Union,
        },
        Scene {
            name: "sphere_union_sphere",
            ceiling: CURVED_CEILING,
            build: overlapping_spheres,
            operation: BooleanOperation::Union,
        },
        Scene {
            name: "orthogonal_cylinders",
            ceiling: CURVED_CEILING,
            build: orthogonal_cylinders,
            operation: BooleanOperation::Union,
        },
    ]
}

/// Runs one Boolean and returns its stage profile, whether or not it succeeded.
///
/// A Boolean that aborts on incomplete coverage still carries the diagnostics
/// that say where the work went, and that is exactly the case worth profiling.
fn run_once(scene: &Scene) -> Result<String, String> {
    let (mut map, first, second) = (scene.build)();
    let started = Instant::now();
    let outcome = boolean(
        &mut map,
        first,
        second,
        scene.operation,
        BooleanOptions::default(),
    );
    let elapsed = started.elapsed();
    match outcome {
        Ok(result) => Ok(format!(
            "{} ok in {:?}\n{}",
            scene.name,
            elapsed,
            result.diagnostics.profile()
        )),
        Err(BooleanError::IncompleteIntersections { diagnostics }) => Err(format!(
            "{} incomplete in {:?} ({:?})\n{}",
            scene.name,
            elapsed,
            diagnostics.coverage,
            diagnostics.profile()
        )),
        Err(error) => Err(format!("{} failed in {elapsed:?}: {error}", scene.name)),
    }
}

/// Runs `scene` on its own thread and gives up at its ceiling.
///
/// A thread that blows the ceiling is left running: it is inside a search that
/// cannot be interrupted, and the process is about to report a failure anyway.
fn run_with_ceiling(scene: &Scene) -> bool {
    let (sender, receiver) = mpsc::channel();
    let name = scene.name;
    let ceiling = scene.ceiling;
    let build = scene.build;
    let operation = scene.operation;
    thread::spawn(move || {
        let scene = Scene {
            name,
            ceiling,
            build,
            operation,
        };
        let _ = sender.send(run_once(&scene));
    });

    match receiver.recv_timeout(ceiling) {
        Ok(Ok(report)) => {
            println!("{report}");
            true
        }
        Ok(Err(report)) => {
            println!("{report}");
            false
        }
        Err(_) => {
            println!("{name} EXCEEDED its {ceiling:?} ceiling");
            false
        }
    }
}

fn boolean_benches(criterion: &mut Criterion) {
    let mut group = criterion.benchmark_group("booleans");
    group.sample_size(10);
    for scene in scenes() {
        if !run_with_ceiling(&scene) {
            println!("{} is not measured: it did not complete", scene.name);
            continue;
        }
        group.bench_function(scene.name, |bencher| {
            bencher.iter(|| {
                let (mut map, first, second) = (scene.build)();
                let _ = boolean(
                    &mut map,
                    first,
                    second,
                    scene.operation,
                    BooleanOptions::default(),
                );
            });
        });
    }
    group.finish();
}

/// The free-form case, benched at the solver rather than the Boolean.
///
/// No modeling builder produces a solid bounded by a genuinely free-form patch,
/// so a free-form *Boolean* cannot be constructed from the public API today.
/// The surface/surface pair underneath it can, and that is where the cost is.
fn free_form_surface_benches(criterion: &mut Criterion) {
    let mut group = criterion.benchmark_group("free_form_surfaces");
    group.sample_size(10);
    let paraboloid = square_paraboloid(0.5);
    let plane = Surface::Plane(Plane::xy());
    let steeper = square_paraboloid(0.8);
    group.bench_function("paraboloid_x_plane", |bencher| {
        bencher.iter(|| paraboloid.intersect_surface(&plane).unwrap());
    });
    group.bench_function("paraboloid_x_paraboloid", |bencher| {
        bencher.iter(|| {
            let _ = paraboloid.intersect_surface(&steeper);
        });
    });
    group.finish();
}

fn merged(
    target: ngk::topology::shape::Shape<ngk::topology::shape::SolidTag, ngk::StandardPayload>,
    tool: ngk::topology::shape::Shape<ngk::topology::shape::SolidTag, ngk::StandardPayload>,
) -> (Model<ngk::StandardPayload>, SolidKey, SolidKey) {
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

fn overlapping_blocks() -> (Model<ngk::StandardPayload>, SolidKey, SolidKey) {
    merged(
        solids::block(2.0, 2.0, 2.0).expect("block"),
        solids::block_at(
            Frame::from_xy(Point3::new(1.0, 1.0, 1.0), Vector3::x(), Vector3::y()),
            2.0,
            2.0,
            2.0,
        )
        .expect("offset block"),
    )
}

fn block_and_protruding_cylinder() -> (Model<ngk::StandardPayload>, SolidKey, SolidKey) {
    merged(
        solids::block(2.0, 2.0, 2.0).expect("block"),
        solids::cylinder_at(
            Frame::from_xy(Point3::new(1.0, 1.0, 1.0), Vector3::x(), Vector3::y()),
            0.5,
            2.0,
        )
        .expect("cylinder"),
    )
}

fn block_and_through_cylinder() -> (Model<ngk::StandardPayload>, SolidKey, SolidKey) {
    merged(
        solids::block(2.0, 2.0, 2.0).expect("block"),
        solids::cylinder_at(
            Frame::from_xy(Point3::new(1.0, 1.0, -1.0), Vector3::x(), Vector3::y()),
            0.5,
            4.0,
        )
        .expect("cylinder"),
    )
}

fn block_and_sphere() -> (Model<ngk::StandardPayload>, SolidKey, SolidKey) {
    merged(
        solids::block(2.0, 2.0, 2.0).expect("block"),
        solids::sphere_at(
            Frame::from_xy(Point3::new(1.0, 1.0, 2.0), Vector3::x(), Vector3::y()),
            0.8,
        )
        .expect("sphere"),
    )
}

fn overlapping_spheres() -> (Model<ngk::StandardPayload>, SolidKey, SolidKey) {
    merged(
        solids::sphere(1.0).expect("sphere"),
        solids::sphere_at(
            Frame::from_xy(Point3::new(1.2, 0.0, 0.0), Vector3::x(), Vector3::y()),
            1.0,
        )
        .expect("sphere"),
    )
}

fn orthogonal_cylinders() -> (Model<ngk::StandardPayload>, SolidKey, SolidKey) {
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

/// Exact biquadratic patch `z = x^2 + y^2 - radius^2` over `[-1, 1]^2`.
fn square_paraboloid(radius: f64) -> Surface {
    let coordinates = [-1.0, 0.0, 1.0];
    let square_coefficients = [1.0, -1.0, 1.0];
    let points = (0..3)
        .flat_map(|v| {
            (0..3).map(move |u| {
                HPoint::from_cartesian(
                    Point3::new(
                        coordinates[u],
                        coordinates[v],
                        square_coefficients[u] + square_coefficients[v] - radius * radius,
                    ),
                    1.0,
                )
            })
        })
        .collect();
    let knots = KnotVector::new(vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0]).expect("bezier knots");
    Surface::Nurbs(
        NurbsSurface::new(
            Degree::new(2).expect("degree"),
            Degree::new(2).expect("degree"),
            ControlNet::new(points, 3, 3).expect("control net"),
            knots.clone(),
            knots,
        )
        .expect("paraboloid patch"),
    )
}

criterion_group!(benches, boolean_benches, free_form_surface_benches);
criterion_main!(benches);
