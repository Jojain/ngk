//! Property-based fuzzing of solid Booleans over the analytic primitives.
//!
//! Not part of `cargo test`: the target is declared with `test = false`, so it
//! only runs when named.
//!
//! ```text
//! cargo test --release --test boolean_fuzz -- [--cases N] [--seed S] [--case I]
//!                                             [--show] [--no-shrink] [--tol T]
//! ```
//!
//! Each case builds a block, cylinder, sphere or torus `A` centred on the
//! origin and a second primitive `B` under a random rigid pose, then runs
//! `A ∪ B`, `A ∩ B`, `A − B` and `B − A`. Poses and sizes are biased towards
//! snapped values (quarter units, 45° turns) so coplanar faces, tangencies and
//! shared edges come up far more often than uniform sampling would give.
//!
//! Every outcome falls in one of three buckets:
//!
//! - **ok**: the result is a valid, outward-oriented solid.
//! - **refused**: the Boolean returned a typed [`BooleanError`]. That is a
//!   named gap, not a wrong answer; runs tally them by variant.
//! - **bug**: a panic, an `Ok` result that fails validation or measurement,
//!   or volumes that break inclusion–exclusion (`V(A∪B) + V(A∩B) = V(A) +
//!   V(B)`, `V(A−B) = V(A) − V(A∩B)`) or the obvious bounds. `EmptyResult`
//!   counts as volume 0, so a wrongly empty intersection or cut shows up as a
//!   volume mismatch whenever the other operations succeeded.
//!
//! Bugs are shrunk with proptest's value trees towards a smaller case failing
//! the same way, printed as a reproducible `--seed S --case I` line, and with
//! `--show` sent to the debug viewer (`npm run debug` in `visualization/`).

use std::cell::RefCell;
use std::collections::{BTreeMap, HashMap};
use std::f64::consts::PI;
use std::fmt;
use std::panic::{self, AssertUnwindSafe};
use std::sync::Mutex;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::{Duration, Instant};

use nalgebra::{Unit, UnitQuaternion, Vector3};
use ngk::builders::boolean::BooleanError;
use ngk::geometry::{Frame, Point3};
use ngk::modeling::solids;
use ngk::topology::shape::{Shape, SolidTag};
use ngk::topology::validation::{validate_solid_manifold, validate_solid_orientation};
use ngk::viz::debug_viewer::{
    DebugDisplay, DebugNode, DebugViewerOptions, DebugViewerPayload, send_payload,
};
use proptest::prelude::*;
use proptest::strategy::ValueTree;
use proptest::test_runner::{Config, RngAlgorithm, TestRng, TestRunner};
use rayon::prelude::*;

// ---------------------------------------------------------------------------
// Case description
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
enum Primitive {
    Block { x: f64, y: f64, z: f64 },
    Cylinder { radius: f64, height: f64 },
    Sphere { radius: f64 },
    Torus { major: f64, minor: f64 },
}

impl Primitive {
    fn kind(&self) -> &'static str {
        match self {
            Self::Block { .. } => "block",
            Self::Cylinder { .. } => "cylinder",
            Self::Sphere { .. } => "sphere",
            Self::Torus { .. } => "torus",
        }
    }

    fn analytic_volume(&self) -> f64 {
        match *self {
            Self::Block { x, y, z } => x * y * z,
            Self::Cylinder { radius, height } => PI * radius * radius * height,
            Self::Sphere { radius } => 4.0 / 3.0 * PI * radius.powi(3),
            Self::Torus { major, minor } => 2.0 * PI * PI * major * minor * minor,
        }
    }

    /// Offset from the primitive's own frame origin to its centre.
    fn centre_offset(&self) -> Vector3<f64> {
        match *self {
            Self::Block { x, y, z } => Vector3::new(x, y, z) / 2.0,
            Self::Cylinder { height, .. } => Vector3::new(0.0, 0.0, height / 2.0),
            Self::Sphere { .. } | Self::Torus { .. } => Vector3::zeros(),
        }
    }

    /// Builds the primitive centred on `pose`'s translation.
    fn build(&self, pose: &Pose) -> Result<Shape<SolidTag>, String> {
        let rotation = pose.rotation();
        let origin = Point3::from(pose.translation - rotation * self.centre_offset());
        let frame = Frame {
            origin,
            x_dir: rotation * Vector3::x_axis(),
            y_dir: rotation * Vector3::y_axis(),
            z_dir: rotation * Vector3::z_axis(),
        };
        let built = match *self {
            Self::Block { x, y, z } => solids::block_at(frame, x, y, z).map_err(|e| e.to_string()),
            Self::Cylinder { radius, height } => {
                solids::cylinder_at(frame, radius, height).map_err(|e| e.to_string())
            }
            Self::Sphere { radius } => solids::sphere_at(frame, radius).map_err(|e| e.to_string()),
            Self::Torus { major, minor } => {
                solids::torus_at(frame, major, minor).map_err(|e| e.to_string())
            }
        };
        built.map_err(|e| format!("building {self:?}: {e}"))
    }
}

#[derive(Debug, Clone)]
enum Rotation {
    Identity,
    /// `eighths` × 45° about world axis `axis` (0 = x, 1 = y, 2 = z).
    Snapped { axis: usize, eighths: i32 },
    Free { axis: [f64; 3], angle: f64 },
}

#[derive(Debug, Clone)]
struct Pose {
    translation: Vector3<f64>,
    rotation: Rotation,
}

impl Pose {
    fn identity() -> Self {
        Self {
            translation: Vector3::zeros(),
            rotation: Rotation::Identity,
        }
    }

    fn rotation(&self) -> UnitQuaternion<f64> {
        match self.rotation {
            Rotation::Identity => UnitQuaternion::identity(),
            Rotation::Snapped { axis, eighths } => {
                let mut v = Vector3::zeros();
                v[axis] = 1.0;
                UnitQuaternion::from_axis_angle(&Unit::new_normalize(v), eighths as f64 * PI / 4.0)
            }
            Rotation::Free { axis, angle } => {
                let v = Vector3::from(axis);
                match Unit::try_new(v, 1e-6) {
                    Some(axis) => UnitQuaternion::from_axis_angle(&axis, angle),
                    None => UnitQuaternion::identity(),
                }
            }
        }
    }
}

#[derive(Debug, Clone)]
struct Case {
    a: Primitive,
    b: Primitive,
    pose_b: Pose,
}

// ---------------------------------------------------------------------------
// Strategies — the first alternative of each `prop_oneof!` is the one
// shrinking falls back to, so snapped values come first.
// ---------------------------------------------------------------------------

fn length(min_quarters: i32, max_quarters: i32) -> impl Strategy<Value = f64> {
    let lo = min_quarters as f64 * 0.25;
    let hi = max_quarters as f64 * 0.25;
    prop_oneof![
        3 => (min_quarters..=max_quarters).prop_map(|q| q as f64 * 0.25),
        2 => lo..hi,
    ]
}

fn primitive() -> impl Strategy<Value = Primitive> {
    prop_oneof![
        (length(1, 8), length(1, 8), length(1, 8)).prop_map(|(x, y, z)| Primitive::Block { x, y, z }),
        (length(1, 6), length(1, 8))
            .prop_map(|(radius, height)| Primitive::Cylinder { radius, height }),
        length(1, 6).prop_map(|radius| Primitive::Sphere { radius }),
        (length(3, 8), 0.15f64..0.85)
            .prop_map(|(major, ratio)| Primitive::Torus { major, minor: major * ratio })
            .prop_filter("minor radius must be positive", |t| match t {
                Primitive::Torus { minor, .. } => *minor > 0.05,
                _ => true,
            }),
    ]
}

fn coordinate() -> impl Strategy<Value = f64> {
    prop_oneof![
        3 => (-5i32..=5).prop_map(|q| q as f64 * 0.25),
        2 => -1.25f64..1.25,
    ]
}

fn rotation() -> impl Strategy<Value = Rotation> {
    prop_oneof![
        2 => Just(Rotation::Identity),
        3 => (0usize..3, 1i32..8).prop_map(|(axis, eighths)| Rotation::Snapped { axis, eighths }),
        3 => ([-1.0f64..1.0, -1.0f64..1.0, -1.0f64..1.0], -PI..PI)
            .prop_map(|(axis, angle)| Rotation::Free { axis, angle }),
    ]
}

fn case() -> impl Strategy<Value = Case> {
    (primitive(), primitive(), [coordinate(), coordinate(), coordinate()], rotation()).prop_map(
        |(a, b, t, rotation)| Case {
            a,
            b,
            pose_b: Pose {
                translation: Vector3::from(t),
                rotation,
            },
        },
    )
}

/// One independent, reproducible runner per case index.
fn runner_for(seed: u64, index: usize) -> TestRunner {
    let mut state = seed ^ (index as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15);
    let mut bytes = [0u8; 32];
    for chunk in bytes.chunks_mut(8) {
        // splitmix64
        state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        chunk.copy_from_slice(&(z ^ (z >> 31)).to_le_bytes());
    }
    TestRunner::new_with_rng(
        Config::default(),
        TestRng::from_seed(RngAlgorithm::ChaCha, &bytes),
    )
}

// ---------------------------------------------------------------------------
// Running one case
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
enum Op {
    Union,
    Common,
    AMinusB,
    BMinusA,
}

impl Op {
    const ALL: [Op; 4] = [Op::Union, Op::Common, Op::AMinusB, Op::BMinusA];
}

impl fmt::Display for Op {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Op::Union => "A∪B",
            Op::Common => "A∩B",
            Op::AMinusB => "A−B",
            Op::BMinusA => "B−A",
        })
    }
}

enum Outcome {
    /// A solid came back; `volume` and `validity` say whether it holds up.
    Solid {
        shape: Shape<SolidTag>,
        volume: Result<f64, String>,
        validity: Result<(), String>,
    },
    Empty,
    Refused {
        variant: String,
        message: String,
    },
    Panicked(String),
}

impl Outcome {
    /// The volume to use in the invariants, when there is a trustworthy one.
    fn volume(&self) -> Option<f64> {
        match self {
            Outcome::Solid {
                volume: Ok(v),
                validity: Ok(()),
                ..
            } => Some(*v),
            Outcome::Empty => Some(0.0),
            _ => None,
        }
    }
}

thread_local! {
    static LAST_PANIC: RefCell<Option<String>> = const { RefCell::new(None) };
}

fn install_quiet_panic_hook() {
    panic::set_hook(Box::new(|info| {
        let payload = info.payload();
        let message = payload
            .downcast_ref::<&str>()
            .map(|s| s.to_string())
            .or_else(|| payload.downcast_ref::<String>().cloned())
            .unwrap_or_else(|| "<non-string panic>".into());
        let location = info
            .location()
            .map(|l| format!("{}:{}", l.file(), l.line()))
            .unwrap_or_default();
        LAST_PANIC.with(|p| *p.borrow_mut() = Some(format!("{message} @ {location}")));
    }));
}

fn catch<T>(f: impl FnOnce() -> T) -> Result<T, String> {
    panic::catch_unwind(AssertUnwindSafe(f)).map_err(|_| {
        LAST_PANIC
            .with(|p| p.borrow_mut().take())
            .unwrap_or_else(|| "<panic without message>".into())
    })
}

fn variant_name(error: &BooleanError) -> String {
    let debug = format!("{error:?}");
    let end = debug
        .find(|c: char| !(c.is_alphanumeric() || c == '_'))
        .unwrap_or(debug.len());
    let name = &debug[..end];
    // `Healing(..)` and friends: one level deeper says more than the wrapper.
    if let Some(rest) = debug[end..].strip_prefix('(') {
        let inner_end = rest
            .find(|c: char| !(c.is_alphanumeric() || c == '_'))
            .unwrap_or(rest.len());
        if inner_end > 0 {
            return format!("{name}({})", &rest[..inner_end]);
        }
    }
    name.to_string()
}

fn inspect(shape: Shape<SolidTag>) -> Outcome {
    let key = shape.key();
    let validity = validate_solid_manifold(shape.model(), key)
        .and_then(|()| validate_solid_orientation(shape.model(), key))
        .map_err(|e| e.to_string());
    let volume = catch(|| shape.solid().volume())
        .and_then(|r| r.map_err(|e| e.to_string()));
    Outcome::Solid {
        shape,
        volume,
        validity,
    }
}

fn run_op(case: &Case, op: Op) -> Outcome {
    let result = catch(|| -> Result<Result<Shape<SolidTag>, BooleanError>, String> {
        let a = case.a.build(&Pose::identity())?;
        let b = case.b.build(&case.pose_b)?;
        Ok(match op {
            Op::Union => solids::fuse(a, b),
            Op::Common => solids::intersect(a, b),
            Op::AMinusB => solids::cut(a, b),
            Op::BMinusA => solids::cut(b, a),
        })
    });
    match result {
        Err(panic) => Outcome::Panicked(panic),
        Ok(Err(build)) => Outcome::Panicked(build),
        Ok(Ok(Ok(shape))) => match catch(|| inspect(shape)) {
            Ok(outcome) => outcome,
            Err(panic) => Outcome::Panicked(format!("while inspecting result: {panic}")),
        },
        Ok(Ok(Err(BooleanError::EmptyResult))) => Outcome::Empty,
        Ok(Ok(Err(error))) => Outcome::Refused {
            variant: variant_name(&error),
            message: error.to_string(),
        },
    }
}

/// A bug: `class` groups equivalent failures and drives shrinking.
#[derive(Debug, Clone)]
struct Bug {
    class: String,
    detail: String,
}

struct Report {
    operand_volumes: Result<(f64, f64), String>,
    outcomes: Vec<(Op, Outcome)>,
    bugs: Vec<Bug>,
}

fn operand_volume(p: &Primitive, pose: &Pose) -> Result<f64, String> {
    let shape = p.build(pose)?;
    catch(|| shape.solid().volume())?.map_err(|e| format!("measuring operand {p:?}: {e}"))
}

fn run_case(case: &Case, tol: f64) -> Report {
    let mut bugs = Vec::new();
    let operand_volumes = operand_volume(&case.a, &Pose::identity())
        .and_then(|va| operand_volume(&case.b, &case.pose_b).map(|vb| (va, vb)));
    let outcomes: Vec<(Op, Outcome)> = Op::ALL.iter().map(|&op| (op, run_op(case, op))).collect();

    for (op, outcome) in &outcomes {
        match outcome {
            Outcome::Panicked(message) => bugs.push(Bug {
                class: format!("panic in {op}"),
                detail: message.clone(),
            }),
            Outcome::Solid {
                validity: Err(e), ..
            } => bugs.push(Bug {
                class: format!("invalid solid from {op}"),
                detail: e.clone(),
            }),
            Outcome::Solid { volume: Err(e), .. } => bugs.push(Bug {
                class: format!("unmeasurable solid from {op}"),
                detail: e.clone(),
            }),
            Outcome::Empty if *op == Op::Union => bugs.push(Bug {
                class: "empty union".into(),
                detail: "a union of two non-empty solids came back empty".into(),
            }),
            _ => {}
        }
    }

    let (va, vb) = match &operand_volumes {
        Ok(v) => *v,
        Err(e) => {
            bugs.push(Bug {
                class: "operand measurement".into(),
                detail: e.clone(),
            });
            return Report {
                operand_volumes,
                outcomes,
                bugs,
            };
        }
    };
    let volume_of = |op: Op| outcomes.iter().find(|(o, _)| *o == op).and_then(|(_, r)| r.volume());
    let (union, common, a_minus_b, b_minus_a) = (
        volume_of(Op::Union),
        volume_of(Op::Common),
        volume_of(Op::AMinusB),
        volume_of(Op::BMinusA),
    );
    let slack = tol * (va + vb);
    let mut check = |class: &str, lhs: f64, rhs: f64, holds: bool| {
        if !holds {
            bugs.push(Bug {
                class: class.into(),
                detail: format!(
                    "{lhs:.6} vs {rhs:.6} (diff {:.3e}, slack {slack:.3e})  V(A)={va:.6} V(B)={vb:.6}",
                    lhs - rhs
                ),
            });
        }
    };

    if let Some(u) = union {
        check("V(A∪B) < max(V(A),V(B))", u, va.max(vb), u >= va.max(vb) - slack);
        check("V(A∪B) > V(A)+V(B)", u, va + vb, u <= va + vb + slack);
    }
    if let Some(c) = common {
        check("V(A∩B) > min(V(A),V(B))", c, va.min(vb), c <= va.min(vb) + slack);
    }
    if let (Some(u), Some(c)) = (union, common) {
        check("inclusion–exclusion", u + c, va + vb, (u + c - va - vb).abs() <= slack);
    }
    if let (Some(d), Some(c)) = (a_minus_b, common) {
        check("V(A−B) ≠ V(A) − V(A∩B)", d, va - c, (d - (va - c)).abs() <= slack);
    }
    if let (Some(d), Some(c)) = (b_minus_a, common) {
        check("V(B−A) ≠ V(B) − V(A∩B)", d, vb - c, (d - (vb - c)).abs() <= slack);
    }
    if let (Some(u), Some(d1), Some(d2)) = (union, a_minus_b, b_minus_a) {
        // Holds even when the intersection itself is refused.
        check("V(A∪B) ≠ V(A−B) + V(B−A) + V(A∩B)", u - d1 - d2, va + vb - u, {
            let c = u - d1 - d2;
            (c - (va + vb - u)).abs() <= slack
        });
    }

    Report {
        operand_volumes,
        outcomes,
        bugs,
    }
}

// ---------------------------------------------------------------------------
// Shrinking, printing, viewer
// ---------------------------------------------------------------------------

fn shrink(
    mut tree: impl ValueTree<Value = Case>,
    class: &str,
    tol: f64,
    budget: usize,
) -> (Case, Report, usize) {
    let mut best = tree.current();
    let mut best_report = run_case(&best, tol);
    let mut steps = 0;
    if !tree.simplify() {
        return (best, best_report, steps);
    }
    while steps < budget {
        steps += 1;
        let candidate = tree.current();
        let report = run_case(&candidate, tol);
        if report.bugs.iter().any(|b| b.class == class) {
            best = candidate;
            best_report = report;
            if !tree.simplify() {
                break;
            }
        } else if !tree.complicate() {
            break;
        }
    }
    (best, best_report, steps)
}

fn describe_outcome(outcome: &Outcome) -> String {
    match outcome {
        Outcome::Solid {
            volume, validity, ..
        } => {
            let volume = match volume {
                Ok(v) => format!("V={v:.6}"),
                Err(e) => format!("volume failed: {e}"),
            };
            match validity {
                Ok(()) => format!("solid, {volume}"),
                Err(e) => format!("INVALID solid ({e}), {volume}"),
            }
        }
        Outcome::Empty => "empty".into(),
        Outcome::Refused { variant, message } => format!("refused {variant}: {message}"),
        Outcome::Panicked(message) => format!("PANIC {message}"),
    }
}

fn print_report(case: &Case, report: &Report) {
    println!("    A = {:?}  (analytic V={:.6})", case.a, case.a.analytic_volume());
    println!("    B = {:?}  (analytic V={:.6})", case.b, case.b.analytic_volume());
    println!("    pose(B) = {:?}", case.pose_b);
    match &report.operand_volumes {
        Ok((va, vb)) => println!("    measured V(A)={va:.6} V(B)={vb:.6}"),
        Err(e) => println!("    operand volumes: {e}"),
    }
    for (op, outcome) in &report.outcomes {
        println!("    {op}: {}", describe_outcome(outcome));
    }
    for bug in &report.bugs {
        println!("    BUG [{}] {}", bug.class, bug.detail);
    }
}

fn leaf<T: DebugDisplay>(name: &str, value: &T) -> Option<DebugNode> {
    let mut objects = Vec::new();
    value.append_debug_objects(&mut objects).ok()?;
    let object = objects.into_iter().next()?;
    Some(DebugNode::leaf(name, object))
}

fn show_in_viewer(title: &str, case: &Case, report: &Report) {
    let mut nodes = Vec::new();
    if let Ok(a) = case.a.build(&Pose::identity()) {
        nodes.extend(leaf("A", &a));
    }
    if let Ok(b) = case.b.build(&case.pose_b) {
        nodes.extend(leaf("B", &b));
    }
    for (op, outcome) in &report.outcomes {
        if let Outcome::Solid { shape, .. } = outcome {
            nodes.extend(leaf(&format!("{op} result"), shape));
        }
    }
    let options = DebugViewerOptions {
        name: title.to_string(),
        ..Default::default()
    };
    let payload = DebugViewerPayload {
        kind: "ngk.debug.v4".to_owned(),
        name: title.to_string(),
        nodes,
    };
    if let Err(e) = send_payload(&payload, &options) {
        println!("    (viewer: {e})");
    }
}

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------

struct Args {
    cases: usize,
    seed: u64,
    only: Option<usize>,
    show: bool,
    shrink: bool,
    tol: f64,
    shrink_budget: usize,
}

fn parse_args() -> Args {
    let mut args = Args {
        cases: 200,
        seed: 0x6e67_6b21,
        only: None,
        show: false,
        shrink: true,
        tol: 5e-3,
        shrink_budget: 200,
    };
    let mut it = std::env::args().skip(1);
    while let Some(arg) = it.next() {
        let mut value = |name: &str| {
            it.next()
                .unwrap_or_else(|| panic!("{name} needs a value"))
        };
        match arg.as_str() {
            "--cases" => args.cases = value("--cases").parse().expect("--cases N"),
            "--seed" => args.seed = value("--seed").parse().expect("--seed S"),
            "--case" => args.only = Some(value("--case").parse().expect("--case I")),
            "--tol" => args.tol = value("--tol").parse().expect("--tol T"),
            "--shrink-budget" => {
                args.shrink_budget = value("--shrink-budget").parse().expect("--shrink-budget N")
            }
            "--show" => args.show = true,
            "--no-shrink" => args.shrink = false,
            // Flags cargo's own harness would take; ignore rather than fail.
            _ => {}
        }
    }
    args
}

fn main() {
    let args = parse_args();
    install_quiet_panic_hook();
    let strategy = case();

    let indices: Vec<usize> = match args.only {
        Some(i) => vec![i],
        None => (0..args.cases).collect(),
    };
    println!(
        "boolean fuzz: {} case(s), seed {}, tol {:.1e}{}",
        indices.len(),
        args.seed,
        args.tol,
        if args.show { ", showing bugs in the viewer" } else { "" }
    );

    // Watchdog: a Boolean that never returns would otherwise look like a
    // silent hang; name the cases that have been running too long.
    let in_flight: &'static Mutex<HashMap<usize, Instant>> =
        Box::leak(Box::new(Mutex::new(HashMap::new())));
    std::thread::spawn(move || {
        loop {
            std::thread::sleep(Duration::from_secs(20));
            let slow: Vec<String> = in_flight
                .lock()
                .unwrap()
                .iter()
                .filter(|(_, t)| t.elapsed() > Duration::from_secs(20))
                .map(|(i, t)| format!("#{i} ({}s)", t.elapsed().as_secs()))
                .collect();
            if !slow.is_empty() {
                println!("  still running: {}", slow.join(", "));
            }
        }
    });

    let done = AtomicUsize::new(0);
    let started = Instant::now();
    let total = indices.len();
    let results: Vec<(usize, Case, Report)> = indices
        .par_iter()
        .map(|&index| {
            let case = strategy
                .new_tree(&mut runner_for(args.seed, index))
                .expect("case generation")
                .current();
            in_flight.lock().unwrap().insert(index, Instant::now());
            let report = run_case(&case, args.tol);
            in_flight.lock().unwrap().remove(&index);
            let n = done.fetch_add(1, Ordering::Relaxed) + 1;
            if n % 25 == 0 || n == total {
                println!("  {n}/{total} cases ({:.0?})", started.elapsed());
            }
            (index, case, report)
        })
        .collect();

    // Tally.
    let mut per_op: BTreeMap<(Op, String), usize> = BTreeMap::new();
    let mut bug_classes: BTreeMap<String, Vec<usize>> = BTreeMap::new();
    let mut refusal_examples: BTreeMap<String, (usize, Op, String)> = BTreeMap::new();
    // (A kind, B kind) -> [all four ok, some refused/empty-free, bug]
    let mut pairs: BTreeMap<(&str, &str), [usize; 3]> = BTreeMap::new();
    for (index, case, report) in &results {
        let cell = pairs.entry((case.a.kind(), case.b.kind())).or_default();
        if !report.bugs.is_empty() {
            cell[2] += 1;
        } else if report.outcomes.iter().any(|(_, o)| matches!(o, Outcome::Refused { .. })) {
            cell[1] += 1;
        } else {
            cell[0] += 1;
        }
        for (op, outcome) in &report.outcomes {
            let bucket = match outcome {
                Outcome::Solid {
                    volume: Ok(_),
                    validity: Ok(()),
                    ..
                } => "ok".to_string(),
                Outcome::Solid { .. } => "bad solid".to_string(),
                Outcome::Empty => "empty".to_string(),
                Outcome::Refused { variant, message } => {
                    refusal_examples
                        .entry(variant.clone())
                        .or_insert_with(|| (*index, *op, message.clone()));
                    format!("refused {variant}")
                }
                Outcome::Panicked(_) => "panic".to_string(),
            };
            *per_op.entry((*op, bucket)).or_default() += 1;
        }
        for bug in &report.bugs {
            let cases = bug_classes.entry(bug.class.clone()).or_default();
            if cases.last() != Some(index) {
                cases.push(*index);
            }
        }
    }

    println!("\n== outcomes per operation ==");
    for op in Op::ALL {
        let row: Vec<String> = per_op
            .iter()
            .filter(|((o, _), _)| *o == op)
            .map(|((_, bucket), n)| format!("{bucket}: {n}"))
            .collect();
        println!("  {op}  {}", row.join(" | "));
    }

    println!("
== refusals: one example each ==");
    for (variant, (index, op, message)) in &refusal_examples {
        println!("  {variant}: #{index} {op}: {message}");
    }

    println!("
== per pair: clean | some refused | bug ==");
    for ((a, b), [clean, refused, bug]) in &pairs {
        println!("  {:>8} × {:<8} {clean:>4} | {refused:>4} | {bug:>4}", a, b);
    }

    if bug_classes.is_empty() {
        println!("\nno bugs found in {total} case(s)");
        return;
    }

    println!("\n== bugs ==");
    for (class, cases) in &bug_classes {
        println!("  {class}: {} case(s), e.g. #{:?}", cases.len(), &cases[..cases.len().min(8)]);
    }

    // One representative per class, shrunk and shown; a case already shown
    // for an earlier class is not shown again.
    let mut shown = std::collections::HashSet::new();
    for (class, cases) in &bug_classes {
        let index = cases[0];
        if !shown.insert(index) {
            println!("
-- [{class}] first seen in case #{index}, already shown above");
            continue;
        }
        println!("\n-- [{class}] case #{index}  (rerun: --seed {} --case {index})", args.seed);
        let tree = strategy
            .new_tree(&mut runner_for(args.seed, index))
            .expect("case generation");
        let (case, report) = if args.shrink {
            let (case, report, steps) = shrink(tree, class, args.tol, args.shrink_budget);
            println!("  shrunk in {steps} step(s) to:");
            (case, report)
        } else {
            let case = tree.current();
            let report = run_case(&case, args.tol);
            (case, report)
        };
        print_report(&case, &report);
        if args.show {
            show_in_viewer(&format!("fuzz #{index}: {class}"), &case, &report);
        }
    }
    std::process::exit(1);
}
