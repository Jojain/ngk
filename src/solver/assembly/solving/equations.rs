use super::super::{Attachment, BodyId, Bound, Coordinate, Joint3d, JointId, JointKind};
use super::prepare::Component;
use super::vector::{JetVector3, rotate};
use crate::geometry::dim3::frame::Frame;
use crate::geometry::transform::Rigid;
use crate::solver::SolveOptions;
use crate::solver::numerical::{self, Jet, Row};
use nalgebra::{DVector, Vector3};
use std::collections::BTreeMap;
use std::f64::consts::TAU;

pub(super) type Evaluation = numerical::Evaluation<JointId>;

/// A side of a limit of the component's joint at this index.
pub(super) type LimitRef = (usize, Bound);

/// Per angular coordinate of a joint, the value its angle is unwrapped nearest to.
pub(super) type References = BTreeMap<Coordinate, f64>;

/// An attachment frame in the world, as expressions of the unknowns.
pub(super) struct JetFrame {
    origin: JetVector3,
    x: JetVector3,
    y: JetVector3,
    z: JetVector3,
}

impl JetFrame {
    /// A frame that does not move, in the units the rows are measured in.
    fn constant(frame: &Frame) -> Self {
        Self {
            origin: JetVector3::constant(frame.origin.coords),
            x: JetVector3::constant(frame.x_dir.into_inner()),
            y: JetVector3::constant(frame.y_dir.into_inner()),
            z: JetVector3::constant(frame.z_dir.into_inner()),
        }
    }
}

/// The joints' rows at the unknowns `x`, linear rows in units of the component's scale.
pub(super) fn permanent(component: &Component, x: &DVector<f64>) -> Evaluation {
    constrained(component, x, &[])
}

/// The joints' rows with the `active` sides of their limits held as equalities, each
/// grouped with its joint's rows.
pub(super) fn constrained(
    component: &Component,
    x: &DVector<f64>,
    active: &[LimitRef],
) -> Evaluation {
    rows(
        component,
        |attachment| frame(component, x, attachment),
        component.scale,
        active,
        x.len(),
    )
}

/// Every side of every limit, with how far the unknowns `x` stand beyond it in units of
/// its tolerance: negative inside.
pub(super) fn excesses(
    component: &Component,
    x: &DVector<f64>,
    options: &SolveOptions,
) -> Vec<(LimitRef, f64)> {
    limit_excesses(
        component,
        |attachment| frame(component, x, attachment),
        component.scale,
        options,
    )
}

/// The same at published world poses, in the caller's units.
pub(super) fn world_excesses(
    component: &Component,
    poses: &BTreeMap<BodyId, Rigid>,
    options: &SolveOptions,
) -> Vec<(LimitRef, f64)> {
    limit_excesses(
        component,
        |attachment| world_frame(attachment, poses),
        1.,
        options,
    )
}

fn limit_excesses(
    component: &Component,
    frame: impl Fn(&Attachment) -> JetFrame,
    scale: f64,
    options: &SolveOptions,
) -> Vec<(LimitRef, f64)> {
    let mut output = vec![];
    for (index, ((_, joint), references)) in component
        .joints
        .iter()
        .zip(&component.references)
        .enumerate()
    {
        let (a, b) = (frame(&joint.first), frame(&joint.second));
        for (bound, limit) in joint.bounds() {
            let row = beyond(bound, limit, &a, &b, references, scale);
            let tolerance = if row.angular {
                options.angular_tolerance
            } else {
                options.linear_tolerance / scale
            };
            output.push(((index, bound), row.expression.value / tolerance));
        }
    }
    output
}

/// The joints' rows at published world poses, linear rows in the caller's units.
///
/// This audits what is published, including rounding at large world offsets.
pub(super) fn world(component: &Component, poses: &BTreeMap<BodyId, Rigid>) -> Evaluation {
    rows(
        component,
        |attachment| world_frame(attachment, poses),
        1.,
        &[],
        0,
    )
}

/// The targets' rows at the unknowns `x`, each weighted by the root of its weight.
pub(super) fn objectives(component: &Component, x: &DVector<f64>) -> Evaluation {
    let mut rows = vec![];
    for (_, target) in &component.targets {
        let entry = component.entry(target.body);
        let local = JetVector3::constant(entry.rotation * target.point.coords / component.scale);
        let actual = translation(component, x, entry.column, entry.translation)
            + turned(x, entry.column, local);
        let wanted =
            JetVector3::constant((target.position.coords - component.origin) / component.scale);
        let weight = Jet::constant(target.weight.sqrt());
        rows.extend(
            (actual - wanted)
                .scale(&weight)
                .components()
                .into_iter()
                .map(Row::linear),
        );
    }
    Evaluation::new(rows, vec![], x.len())
}

/// The references a joint's angles are unwrapped nearest to, read at the poses it
/// starts from: an imposed angle is its own reference; otherwise the representative of
/// the angle nearest its seed, or the angle in `(-π, π]` without one.
pub(super) fn references(
    joint: &Joint3d,
    seeds: &BTreeMap<Coordinate, f64>,
    poses: &BTreeMap<BodyId, Rigid>,
) -> References {
    let (a, b) = (
        world_frame(&joint.first, poses),
        world_frame(&joint.second, poses),
    );
    joint
        .kind
        .coordinates()
        .iter()
        .filter(|coordinate| coordinate.angular())
        .map(|&coordinate| {
            let raw = raw(coordinate, &a, &b).value;
            let reference = match (joint.values.get(&coordinate), seeds.get(&coordinate)) {
                (Some(value), _) => *value,
                (None, Some(seed)) => raw + TAU * ((seed - raw) / TAU).round(),
                (None, None) => raw,
            };
            (coordinate, reference)
        })
        .collect()
}

/// A joint's coordinates at published world poses, angles unwrapped near `references`.
pub(super) fn coordinates(
    joint: &Joint3d,
    references: &References,
    poses: &BTreeMap<BodyId, Rigid>,
) -> BTreeMap<Coordinate, f64> {
    let (a, b) = (
        world_frame(&joint.first, poses),
        world_frame(&joint.second, poses),
    );
    joint
        .kind
        .coordinates()
        .iter()
        .map(|&coordinate| (coordinate, value(coordinate, &a, &b, references).value))
        .collect()
}

fn world_frame(attachment: &Attachment, poses: &BTreeMap<BodyId, Rigid>) -> JetFrame {
    JetFrame::constant(&attachment.frame.moved(&poses[&attachment.body]))
}

fn rows(
    component: &Component,
    frame: impl Fn(&Attachment) -> JetFrame,
    scale: f64,
    active: &[LimitRef],
    columns: usize,
) -> Evaluation {
    let mut rows = vec![];
    let mut groups = vec![];
    for (index, ((id, joint), references)) in component
        .joints
        .iter()
        .zip(&component.references)
        .enumerate()
    {
        let start = rows.len();
        let (a, b) = (frame(&joint.first), frame(&joint.second));
        rows.extend(joint_rows(joint.kind, &a, &b, scale, references));
        for (coordinate, imposed) in &joint.values {
            let actual = value(*coordinate, &a, &b, references);
            rows.push(if coordinate.angular() {
                Row::angular(actual - *imposed)
            } else {
                Row::linear(actual - imposed / scale)
            });
        }
        for (bound, limit) in joint.bounds() {
            if active.contains(&(index, bound)) {
                rows.push(beyond(bound, limit, &a, &b, references, scale));
            }
        }
        groups.push((*id, start..rows.len()));
    }
    Evaluation::new(rows, groups, columns)
}

/// An attachment frame at the unknowns `x`, its origin in normalized units.
fn frame(component: &Component, x: &DVector<f64>, attachment: &Attachment) -> JetFrame {
    let entry = component.entry(attachment.body);
    let local = &attachment.frame;
    let axis = |v: Vector3<f64>| turned(x, entry.column, JetVector3::constant(entry.rotation * v));
    let offset = JetVector3::constant(entry.rotation * local.origin.coords / component.scale);
    JetFrame {
        origin: translation(component, x, entry.column, entry.translation)
            + turned(x, entry.column, offset),
        x: axis(local.x_dir.into_inner()),
        y: axis(local.y_dir.into_inner()),
        z: axis(local.z_dir.into_inner()),
    }
}

/// A body's normalized translation: its three unknowns, or a constant when fixed.
fn translation(
    component: &Component,
    x: &DVector<f64>,
    column: Option<usize>,
    start: Vector3<f64>,
) -> JetVector3 {
    match column {
        Some(column) => JetVector3::variables(x, column),
        None => JetVector3::constant((start - component.origin) / component.scale),
    }
}

/// A vector already carried by the body's start orientation, turned by its rotation
/// unknowns; a fixed body does not turn.
fn turned(x: &DVector<f64>, column: Option<usize>, v: JetVector3) -> JetVector3 {
    match column {
        Some(column) => rotate(&JetVector3::variables(x, column + 3), &v),
        None => v,
    }
}

/// A coordinate of the second frame in the first, an angle in `(-π, π]`.
fn raw(coordinate: Coordinate, a: &JetFrame, b: &JetFrame) -> Jet {
    let offset = b.origin.clone() - a.origin.clone();
    match coordinate {
        Coordinate::Slide => offset.dot(&a.z),
        Coordinate::X => offset.dot(&a.x),
        Coordinate::Y => offset.dot(&a.y),
        // The second X about the first Z.
        Coordinate::Angle => b.x.dot(&a.y).atan2(b.x.dot(&a.x)),
        // The second Y in the first YZ plane, from the first Y.
        Coordinate::Alpha => b.y.dot(&a.z).atan2(b.y.dot(&a.y)),
        // The second X about the second Y, from the first X.
        Coordinate::Beta => {
            let turned = b.y.cross(&a.x);
            b.x.dot(&turned).atan2(b.x.dot(&a.x))
        }
        // The second Z about the second X, from the first Z.
        Coordinate::Roll => {
            let across = a.z.cross(&b.x);
            (-b.z.dot(&across)).atan2(b.z.dot(&a.z))
        }
    }
}

/// A coordinate with its angle unwrapped within half a turn of its reference.
fn value(coordinate: Coordinate, a: &JetFrame, b: &JetFrame, references: &References) -> Jet {
    let raw = raw(coordinate, a, b);
    match references.get(&coordinate) {
        Some(&reference) => (raw - reference).wrapped() + reference,
        None => raw,
    }
}

/// How far a coordinate stands beyond one side of its limit, negative inside; linear
/// rows in units of `scale`.
fn beyond(
    bound: Bound,
    limit: f64,
    a: &JetFrame,
    b: &JetFrame,
    references: &References,
    scale: f64,
) -> Row {
    let row = |coordinate: Coordinate, excess: Jet| {
        if coordinate.angular() {
            Row::angular(excess)
        } else {
            Row::linear(excess)
        }
    };
    let unit = |coordinate: Coordinate| {
        if coordinate.angular() {
            limit
        } else {
            limit / scale
        }
    };
    match bound {
        Bound::Min(coordinate) => row(
            coordinate,
            -value(coordinate, a, b, references) + unit(coordinate),
        ),
        Bound::Max(coordinate) => row(
            coordinate,
            value(coordinate, a, b, references) - unit(coordinate),
        ),
        Bound::Cone => {
            let across = a.z.cross(&b.z);
            let angle = across.dot(&across).sqrt().atan2(a.z.dot(&b.z));
            Row::angular(angle - limit)
        }
    }
}

/// The equations a joint imposes on its two frames, linear rows in units of `scale`.
///
/// Orientation rows measure the angle through `2 tan(θ/2)`: equal to the angle near
/// zero, vanishing only when the frames agree, and growing without bound as they turn
/// over, so a correction never settles on the opposite orientation.
fn joint_rows(
    kind: JointKind,
    a: &JetFrame,
    b: &JetFrame,
    scale: f64,
    references: &References,
) -> Vec<Row> {
    let offset = b.origin.clone() - a.origin.clone();
    let coincide = || offset.clone().components().into_iter().map(Row::linear);
    let on_axis = || [Row::linear(offset.dot(&a.x)), Row::linear(offset.dot(&a.y))];
    let on_plane = || Row::linear(offset.dot(&a.z));
    let mut rows = vec![];
    match kind {
        JointKind::Fixed => {
            rows.extend(coincide());
            rows.extend(orientation(a, b));
        }
        JointKind::Revolute => {
            rows.extend(coincide());
            rows.extend(axis_alignment(a, b));
        }
        JointKind::Prismatic => {
            rows.extend(on_axis());
            rows.extend(orientation(a, b));
        }
        JointKind::Helical { pitch } => {
            rows.extend(on_axis());
            rows.extend(axis_alignment(a, b));
            let angle = value(Coordinate::Angle, a, b, references);
            rows.push(Row::linear(
                offset.dot(&a.z) - angle * (pitch / TAU / scale),
            ));
        }
        JointKind::Cylindrical => {
            rows.extend(on_axis());
            rows.extend(axis_alignment(a, b));
        }
        JointKind::Universal => {
            rows.extend(coincide());
            // The cross keeps its arms square: the first X and the second Y.
            rows.push(Row::angular(a.x.dot(&b.y)));
        }
        JointKind::Spherical => rows.extend(coincide()),
        JointKind::Planar => {
            rows.push(on_plane());
            rows.extend(axis_alignment(a, b));
        }
        JointKind::CylinderOnPlane => {
            rows.push(on_plane());
            // The cylinder's axis, the second X, lies in the plane.
            rows.push(Row::angular(b.x.dot(&a.z)));
        }
        JointKind::SphereInCylinder => rows.extend(on_axis()),
        JointKind::SphereOnPlane => rows.push(on_plane()),
    }
    rows
}

/// Three rows vanishing when the two frames' axes coincide.
///
/// With `R = Aᵀ B` the second frame's axes in the first, `vee(R - Rᵀ) / (1 + tr R)` is
/// `tan(θ/2)` along the axis of the relative rotation.
fn orientation(a: &JetFrame, b: &JetFrame) -> Vec<Row> {
    let r = |p: &JetVector3, q: &JetVector3| p.dot(q);
    let trace = r(&a.x, &b.x) + r(&a.y, &b.y) + r(&a.z, &b.z);
    let scale = Jet::constant(2.) / (trace + 1.);
    [
        r(&a.z, &b.y) - r(&a.y, &b.z),
        r(&a.x, &b.z) - r(&a.z, &b.x),
        r(&a.y, &b.x) - r(&a.x, &b.y),
    ]
    .into_iter()
    .map(|v| Row::angular(v * scale.clone()))
    .collect()
}

/// Two rows vanishing when the second Z axis coincides with the first, whatever the
/// turn about it: the second Z in the first frame, over `1 + cos θ`.
fn axis_alignment(a: &JetFrame, b: &JetFrame) -> Vec<Row> {
    let scale = Jet::constant(2.) / (a.z.dot(&b.z) + 1.);
    vec![
        Row::angular(a.x.dot(&b.z) * scale.clone()),
        Row::angular(a.y.dot(&b.z) * scale),
    ]
}
