use super::super::{Attachment, BodyId, JointId, JointKind};
use super::prepare::Component;
use super::vector::{JetVector3, rotate};
use crate::geometry::dim3::frame::Frame;
use crate::geometry::transform::Rigid;
use crate::solver::numerical::{self, Jet, Row};
use nalgebra::{DVector, Vector3};
use std::collections::BTreeMap;

pub(super) type Evaluation = numerical::Evaluation<JointId>;

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
    rows(
        component,
        |attachment| frame(component, x, attachment),
        x.len(),
    )
}

/// The joints' rows at published world poses, linear rows in the caller's units.
///
/// This audits what is published, including rounding at large world offsets.
pub(super) fn world(component: &Component, poses: &BTreeMap<BodyId, Rigid>) -> Evaluation {
    rows(
        component,
        |attachment| JetFrame::constant(&attachment.frame.moved(&poses[&attachment.body])),
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

fn rows(
    component: &Component,
    frame: impl Fn(&Attachment) -> JetFrame,
    columns: usize,
) -> Evaluation {
    let mut rows = vec![];
    let mut groups = vec![];
    for (id, joint) in &component.joints {
        let start = rows.len();
        rows.extend(joint_rows(
            joint.kind,
            &frame(&joint.first),
            &frame(&joint.second),
        ));
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

/// The equations a joint imposes on its two frames.
///
/// Orientation rows measure the angle through `2 tan(θ/2)`: equal to the angle near
/// zero, vanishing only when the frames agree, and growing without bound as they turn
/// over, so a correction never settles on the opposite orientation.
fn joint_rows(kind: JointKind, a: &JetFrame, b: &JetFrame) -> Vec<Row> {
    let offset = b.origin.clone() - a.origin.clone();
    let mut rows = vec![];
    match kind {
        JointKind::Fixed => {
            rows.extend(offset.components().into_iter().map(Row::linear));
            rows.extend(orientation(a, b));
        }
        JointKind::Revolute => {
            rows.extend(offset.components().into_iter().map(Row::linear));
            rows.extend(axis_alignment(a, b));
        }
        JointKind::Prismatic => {
            rows.push(Row::linear(offset.dot(&a.x)));
            rows.push(Row::linear(offset.dot(&a.y)));
            rows.extend(orientation(a, b));
        }
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
