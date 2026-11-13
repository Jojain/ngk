use super::BodyId;
use crate::geometry::dim3::frame::Frame;
use serde::{Deserialize, Serialize};

/// Where a joint holds a body: a frame in that body's own coordinates.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Attachment {
    pub body: BodyId,
    pub frame: Frame,
}

impl Attachment {
    pub fn new(body: BodyId, frame: Frame) -> Self {
        Self { body, frame }
    }
}

/// What a joint lets the second frame do relative to the first.
///
/// At joint value zero the two frames coincide: origin, X, Y and Z. Z carries the
/// joint's axis.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum JointKind {
    /// No relative motion (encastrement).
    Fixed,
    /// Rotation about Z (pivot).
    Revolute,
    /// Translation along Z (glissière).
    Prismatic,
}

/// A permanent joint between two bodies, each held by a frame.
///
/// The order matters: the joint's coordinates express the second frame in the first.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Joint3d {
    pub kind: JointKind,
    pub first: Attachment,
    pub second: Attachment,
}

impl Joint3d {
    pub fn new(kind: JointKind, first: Attachment, second: Attachment) -> Self {
        Self {
            kind,
            first,
            second,
        }
    }
    pub fn fixed(first: Attachment, second: Attachment) -> Self {
        Self::new(JointKind::Fixed, first, second)
    }
    pub fn revolute(first: Attachment, second: Attachment) -> Self {
        Self::new(JointKind::Revolute, first, second)
    }
    pub fn prismatic(first: Attachment, second: Attachment) -> Self {
        Self::new(JointKind::Prismatic, first, second)
    }
    pub fn bodies(&self) -> [BodyId; 2] {
        [self.first.body, self.second.body]
    }
}

/// Whether a frame is finite, orthonormal and right-handed, as a joint relies on.
pub(super) fn valid_frame(frame: &Frame) -> bool {
    let (x, y, z) = (
        frame.x_dir.into_inner(),
        frame.y_dir.into_inner(),
        frame.z_dir.into_inner(),
    );
    const SQUARE: f64 = 1e-9;
    frame
        .origin
        .coords
        .iter()
        .chain(&x)
        .chain(&y)
        .chain(&z)
        .all(|v| v.is_finite())
        && [x, y, z]
            .iter()
            .all(|axis| (axis.norm() - 1.).abs() <= SQUARE)
        && x.dot(&y).abs() <= SQUARE
        && y.dot(&z).abs() <= SQUARE
        && z.dot(&x).abs() <= SQUARE
        && x.cross(&y).dot(&z) > 0.
}
