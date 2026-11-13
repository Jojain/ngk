use super::BodyId;
use crate::geometry::dim3::frame::Frame;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

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
/// joint's axis: the revolute axis, the slide direction, the planar normal.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum JointKind {
    /// No relative motion (encastrement).
    Fixed,
    /// Rotation about Z (pivot).
    Revolute,
    /// Translation along Z (glissière).
    Prismatic,
    /// Rotation about Z driving a translation along it (hélicoïdale): the slide is
    /// `pitch` per turn, positive for a right-handed screw, negative for a left-handed one.
    Helical { pitch: f64 },
    /// Rotation about Z and translation along it (pivot glissant).
    Cylindrical,
    /// Rotation about the first X, then about the second Y; no twist about Z
    /// (rotule à doigt, a cardan joint).
    Universal,
    /// Any rotation about the common origin (rotule).
    Spherical,
    /// Translation in the first XY plane and rotation about Z (appui plan).
    Planar,
    /// The second origin on the first XY plane and the second X axis in it: a cylinder
    /// along that X axis lying on the plane (linéaire rectiligne).
    CylinderOnPlane,
    /// The second origin on the first Z axis, any rotation: a sphere in a cylinder
    /// (linéaire annulaire).
    SphereInCylinder,
    /// The second origin on the first XY plane, any rotation: a sphere on a plane
    /// (ponctuelle).
    SphereOnPlane,
}

/// A scalar coordinate of a joint: the second frame relative to the first.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum Coordinate {
    /// Rotation about the first Z, measured from the first X to the second X.
    Angle,
    /// Translation along the first Z.
    Slide,
    /// Translation along the first X.
    X,
    /// Translation along the first Y.
    Y,
    /// Rotation of a cylinder on a plane about its own axis, the second X.
    Roll,
    /// The universal joint's rotation about the first X.
    Alpha,
    /// The universal joint's rotation about the second Y.
    Beta,
}

impl Coordinate {
    /// Whether the coordinate is an angle, unwrapped from its seed, rather than a length.
    pub fn angular(self) -> bool {
        matches!(
            self,
            Coordinate::Angle | Coordinate::Roll | Coordinate::Alpha | Coordinate::Beta
        )
    }
}

impl JointKind {
    /// The joint's scalar coordinates. Free rotations (spherical, sphere in cylinder,
    /// sphere on plane) have none: they read as the joint's relative motion.
    pub fn coordinates(self) -> &'static [Coordinate] {
        use Coordinate::*;
        match self {
            JointKind::Fixed | JointKind::Spherical => &[],
            JointKind::Revolute => &[Angle],
            JointKind::Prismatic | JointKind::SphereInCylinder => &[Slide],
            JointKind::Helical { .. } | JointKind::Cylindrical => &[Angle, Slide],
            JointKind::Universal => &[Alpha, Beta],
            JointKind::Planar => &[X, Y, Angle],
            JointKind::CylinderOnPlane => &[X, Y, Angle, Roll],
            JointKind::SphereOnPlane => &[X, Y],
        }
    }
    /// Whether a value can be imposed on this coordinate: it is one of the joint's, and
    /// free. A helical slide follows its angle, so only the angle is imposed.
    pub fn imposable(self, coordinate: Coordinate) -> bool {
        self.coordinates().contains(&coordinate)
            && !(matches!(self, JointKind::Helical { .. }) && coordinate == Coordinate::Slide)
    }
    /// Whether the joint leaves a free rotation, which accepts only a cone limit.
    pub fn free_rotation(self) -> bool {
        matches!(
            self,
            JointKind::Spherical | JointKind::SphereInCylinder | JointKind::SphereOnPlane
        )
    }
    fn valid(self) -> bool {
        match self {
            JointKind::Helical { pitch } => pitch.is_finite() && pitch != 0.,
            _ => true,
        }
    }
}

/// Bounds a coordinate keeps within, either side optional.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Limit {
    pub min: Option<f64>,
    pub max: Option<f64>,
}

impl Limit {
    pub fn between(min: f64, max: f64) -> Self {
        Self {
            min: Some(min),
            max: Some(max),
        }
    }
    pub fn at_least(min: f64) -> Self {
        Self {
            min: Some(min),
            max: None,
        }
    }
    pub fn at_most(max: f64) -> Self {
        Self {
            min: None,
            max: Some(max),
        }
    }
    fn contains(self, value: f64) -> bool {
        self.min.is_none_or(|min| value >= min) && self.max.is_none_or(|max| value <= max)
    }
}

/// One side of a joint's limit.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Bound {
    Min(Coordinate),
    Max(Coordinate),
    /// The cone the second Z keeps within about the first.
    Cone,
}

/// A permanent joint between two bodies, each held by a frame.
///
/// The order matters: the joint's coordinates express the second frame in the first.
/// `values` imposes coordinates, as dimensions do: an angle in radians, unwrapped, or a
/// length in the caller's unit. `limits` bounds scalar coordinates, in the same units;
/// `cone` bounds a free rotation by the largest angle between the two Z axes.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Joint3d {
    pub kind: JointKind,
    pub first: Attachment,
    pub second: Attachment,
    #[serde(default)]
    pub values: BTreeMap<Coordinate, f64>,
    #[serde(default)]
    pub limits: BTreeMap<Coordinate, Limit>,
    #[serde(default)]
    pub cone: Option<f64>,
}

impl Joint3d {
    pub fn new(kind: JointKind, first: Attachment, second: Attachment) -> Self {
        Self {
            kind,
            first,
            second,
            values: BTreeMap::new(),
            limits: BTreeMap::new(),
            cone: None,
        }
    }
    /// Bounds a coordinate; checked when the joint is added. A helical slide may be
    /// bounded: it follows the angle.
    pub fn with_limit(mut self, coordinate: Coordinate, limit: Limit) -> Self {
        self.limits.insert(coordinate, limit);
        self
    }
    /// Bounds a free rotation: the second Z stays within `angle` of the first.
    pub fn with_cone(mut self, angle: f64) -> Self {
        self.cone = Some(angle);
        self
    }
    /// Imposes a coordinate's value; checked when the joint is added.
    pub fn with_value(mut self, coordinate: Coordinate, value: f64) -> Self {
        self.values.insert(coordinate, value);
        self
    }
    pub fn bodies(&self) -> [BodyId; 2] {
        [self.first.body, self.second.body]
    }
    /// Why the joint cannot be added, independently of the assembly.
    pub(super) fn invalid(&self) -> Option<&'static str> {
        if self.first.body == self.second.body {
            return Some("a joint holds two different bodies");
        }
        if !valid_frame(&self.first.frame) || !valid_frame(&self.second.frame) {
            return Some("attachment frames must be finite, orthonormal and right-handed");
        }
        if !self.kind.valid() {
            return Some("a helical pitch must be finite and nonzero");
        }
        for (coordinate, value) in &self.values {
            if !self.kind.imposable(*coordinate) {
                return Some("only a free coordinate of the joint can be imposed");
            }
            if !value.is_finite() {
                return Some("an imposed value must be finite");
            }
        }
        for (coordinate, limit) in &self.limits {
            if !self.kind.coordinates().contains(coordinate) {
                return Some("only a coordinate of the joint can be limited");
            }
            let finite = |side: Option<f64>| side.is_none_or(f64::is_finite);
            if limit.min.is_none() && limit.max.is_none()
                || !finite(limit.min)
                || !finite(limit.max)
            {
                return Some("a limit has a finite min, max or both");
            }
            if let (Some(min), Some(max)) = (limit.min, limit.max)
                && min > max
            {
                return Some("a limit's min exceeds its max");
            }
        }
        if !self.values.iter().all(|(coordinate, value)| {
            self.limits
                .get(coordinate)
                .is_none_or(|limit| limit.contains(*value))
        }) {
            return Some("an imposed value lies outside its limit");
        }
        if let (JointKind::Helical { pitch }, Some(angle), Some(limit)) = (
            self.kind,
            self.values.get(&Coordinate::Angle),
            self.limits.get(&Coordinate::Slide),
        ) && !limit.contains(pitch * angle / std::f64::consts::TAU)
        {
            return Some("an imposed angle drives the slide outside its limit");
        }
        if let Some(angle) = self.cone {
            if !self.kind.free_rotation() {
                return Some("only a free rotation has a cone limit");
            }
            if !(angle > 0. && angle <= std::f64::consts::PI) {
                return Some("a cone limit lies in (0, pi]");
            }
        }
        None
    }
    /// Every side of the joint's limits, with its value.
    pub(super) fn bounds(&self) -> Vec<(Bound, f64)> {
        let mut bounds = vec![];
        for (coordinate, limit) in &self.limits {
            bounds.extend(limit.min.map(|min| (Bound::Min(*coordinate), min)));
            bounds.extend(limit.max.map(|max| (Bound::Max(*coordinate), max)));
        }
        bounds.extend(self.cone.map(|angle| (Bound::Cone, angle)));
        bounds
    }
}

/// Whether a frame is finite, orthonormal and right-handed, as a joint relies on.
fn valid_frame(frame: &Frame) -> bool {
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
