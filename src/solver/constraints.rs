use super::{ArcId, CircleId, CircularId, GeometryId, PointRef, SegmentId};
use crate::geometry::dim2::utils::Point2;
use crate::geometry::parameter::{Fraction, NativeParam};
use Constraint2d::*;
use nalgebra::Vector2;
use radians::Rad64;
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

/// The contact branch of two supporting circles.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum TangencyMode {
    Automatic,
    External,
    /// The initially larger circle contains the other; ties select the first.
    Internal,
    FirstContainsSecond,
    SecondContainsFirst,
}

/// `Rad64` has no serial form of its own: an angle is stored as its radians.
mod stored_angle {
    use radians::Rad64;
    use serde::{Deserialize, Deserializer, Serialize, Serializer};

    pub fn serialize<S: Serializer>(angle: &Rad64, serializer: S) -> Result<S::Ok, S::Error> {
        angle.val().serialize(serializer)
    }
    pub fn deserialize<'de, D: Deserializer<'de>>(deserializer: D) -> Result<Rad64, D::Error> {
        f64::deserialize(deserializer).map(Rad64::new)
    }
}

/// Permanent geometric relations. Lengths use the caller's unit.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Constraint2d {
    Coincident(PointRef, PointRef),
    PointOnLine(PointRef, SegmentId),
    PointOnCircle(PointRef, CircularId),
    Horizontal(SegmentId),
    Vertical(SegmentId),
    Distance(PointRef, PointRef, f64),
    DistanceToLine(PointRef, SegmentId, f64),
    OffsetX(PointRef, PointRef, f64),
    OffsetY(PointRef, PointRef, f64),
    Length(SegmentId, f64),
    Radius(CircularId, f64),
    EqualLength(SegmentId, SegmentId),
    EqualRadius(CircularId, CircularId),
    Parallel(SegmentId, SegmentId),
    Perpendicular(SegmentId, SegmentId),
    /// Unsigned values lie in [0, pi]; oriented values are interpreted modulo a turn.
    Angle {
        first: SegmentId,
        second: SegmentId,
        #[serde(with = "stored_angle")]
        angle: Rad64,
        oriented: bool,
    },
    Concentric(CircularId, CircularId),
    Midpoint {
        point: PointRef,
        start: PointRef,
        end: PointRef,
    },
    SymmetricAboutLine {
        first: PointRef,
        second: PointRef,
        line: SegmentId,
    },
    SymmetricAboutPoint {
        first: PointRef,
        second: PointRef,
        center: PointRef,
    },
    /// Captures this point's position when the constraint is added or replaced.
    FixedPoint(PointRef),
    /// Captures all of the primitive's parameters when added or replaced.
    Fixed(GeometryId),
    TangentLineCircle(SegmentId, CircularId),
    TangentCircles {
        first: CircularId,
        second: CircularId,
        mode: TangencyMode,
    },
}

impl Constraint2d {
    pub fn coincident(a: impl Into<PointRef>, b: impl Into<PointRef>) -> Self {
        Self::Coincident(a.into(), b.into())
    }
    pub fn point_on_line(p: impl Into<PointRef>, line: SegmentId) -> Self {
        Self::PointOnLine(p.into(), line)
    }
    pub fn point_on_circle(p: impl Into<PointRef>, circle: impl Into<CircularId>) -> Self {
        Self::PointOnCircle(p.into(), circle.into())
    }
    pub fn horizontal(line: SegmentId) -> Self {
        Self::Horizontal(line)
    }
    pub fn vertical(line: SegmentId) -> Self {
        Self::Vertical(line)
    }
    pub fn distance(a: impl Into<PointRef>, b: impl Into<PointRef>, value: f64) -> Self {
        Self::Distance(a.into(), b.into(), value)
    }
    pub fn distance_to_line(p: impl Into<PointRef>, line: SegmentId, value: f64) -> Self {
        Self::DistanceToLine(p.into(), line, value)
    }
    pub fn offset_x(a: impl Into<PointRef>, b: impl Into<PointRef>, value: f64) -> Self {
        Self::OffsetX(a.into(), b.into(), value)
    }
    pub fn offset_y(a: impl Into<PointRef>, b: impl Into<PointRef>, value: f64) -> Self {
        Self::OffsetY(a.into(), b.into(), value)
    }
    pub fn length(line: SegmentId, value: f64) -> Self {
        Self::Length(line, value)
    }
    pub fn radius(circle: impl Into<CircularId>, value: f64) -> Self {
        Self::Radius(circle.into(), value)
    }
    pub fn diameter(circle: impl Into<CircularId>, value: f64) -> Self {
        Self::Radius(circle.into(), value / 2.)
    }
    pub fn equal_length(a: SegmentId, b: SegmentId) -> Self {
        Self::EqualLength(a, b)
    }
    pub fn equal_radius(a: impl Into<CircularId>, b: impl Into<CircularId>) -> Self {
        Self::EqualRadius(a.into(), b.into())
    }
    pub fn parallel(a: SegmentId, b: SegmentId) -> Self {
        Self::Parallel(a, b)
    }
    pub fn perpendicular(a: SegmentId, b: SegmentId) -> Self {
        Self::Perpendicular(a, b)
    }
    pub fn angle(first: SegmentId, second: SegmentId, angle: Rad64) -> Self {
        Self::Angle {
            first,
            second,
            angle,
            oriented: false,
        }
    }
    pub fn oriented_angle(first: SegmentId, second: SegmentId, angle: Rad64) -> Self {
        Self::Angle {
            first,
            second,
            angle,
            oriented: true,
        }
    }
    pub fn concentric(a: impl Into<CircularId>, b: impl Into<CircularId>) -> Self {
        Self::Concentric(a.into(), b.into())
    }
    pub fn midpoint(point: impl Into<PointRef>, line: SegmentId) -> Self {
        Self::Midpoint {
            point: point.into(),
            start: line.start(),
            end: line.end(),
        }
    }
    pub fn midpoint_between(
        point: impl Into<PointRef>,
        start: impl Into<PointRef>,
        end: impl Into<PointRef>,
    ) -> Self {
        Self::Midpoint {
            point: point.into(),
            start: start.into(),
            end: end.into(),
        }
    }
    pub fn symmetric_about_line(
        first: impl Into<PointRef>,
        second: impl Into<PointRef>,
        line: SegmentId,
    ) -> Self {
        Self::SymmetricAboutLine {
            first: first.into(),
            second: second.into(),
            line,
        }
    }
    pub fn symmetric_about_point(
        first: impl Into<PointRef>,
        second: impl Into<PointRef>,
        center: impl Into<PointRef>,
    ) -> Self {
        Self::SymmetricAboutPoint {
            first: first.into(),
            second: second.into(),
            center: center.into(),
        }
    }
    pub fn fixed_point(point: impl Into<PointRef>) -> Self {
        Self::FixedPoint(point.into())
    }
    pub fn fixed(geometry: impl Into<GeometryId>) -> Self {
        Self::Fixed(geometry.into())
    }
    pub fn tangent_line_circle(line: SegmentId, circle: impl Into<CircularId>) -> Self {
        Self::TangentLineCircle(line, circle.into())
    }
    pub fn tangent_circles(
        first: impl Into<CircularId>,
        second: impl Into<CircularId>,
        mode: TangencyMode,
    ) -> Self {
        Self::TangentCircles {
            first: first.into(),
            second: second.into(),
            mode,
        }
    }

    /// Lists owning primitives; computed points do not introduce extra ownership.
    pub(crate) fn geometries(&self) -> Vec<GeometryId> {
        match self {
            Coincident(a, b) | Distance(a, b, _) | OffsetX(a, b, _) | OffsetY(a, b, _) => {
                vec![a.geometry, b.geometry]
            }
            PointOnLine(p, l) | DistanceToLine(p, l, _) => vec![p.geometry, l.0],
            PointOnCircle(p, c) => vec![p.geometry, c.0],
            Horizontal(l) | Vertical(l) | Length(l, _) => vec![l.0],
            Radius(c, _) => vec![c.0],
            EqualLength(a, b)
            | Parallel(a, b)
            | Perpendicular(a, b)
            | Angle {
                first: a,
                second: b,
                ..
            } => vec![a.0, b.0],
            EqualRadius(a, b)
            | Concentric(a, b)
            | TangentCircles {
                first: a,
                second: b,
                ..
            } => vec![a.0, b.0],
            Midpoint { point, start, end } => vec![point.geometry, start.geometry, end.geometry],
            SymmetricAboutLine {
                first,
                second,
                line,
            } => vec![first.geometry, second.geometry, line.0],
            SymmetricAboutPoint {
                first,
                second,
                center,
            } => vec![first.geometry, second.geometry, center.geometry],
            FixedPoint(p) => vec![p.geometry],
            Fixed(g) => vec![*g],
            TangentLineCircle(l, c) => vec![l.0, c.0],
        }
    }
    pub(crate) fn valid_values(&self) -> bool {
        match self {
            Length(_, v) | Radius(_, v) => v.is_finite() && *v > 0.,
            Distance(_, _, v) | DistanceToLine(_, _, v) => v.is_finite() && *v >= 0.,
            OffsetX(_, _, v) | OffsetY(_, _, v) => v.is_finite(),
            Angle {
                angle, oriented, ..
            } => angle.val().is_finite() && (*oriented || (0. ..=PI).contains(&angle.val())),
            _ => true,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub(crate) struct StoredConstraint {
    pub definition: Constraint2d,
    pub captured: Vec<f64>,
    /// Fixed primitives capture their discrete direction and circle parameter frame too.
    pub captured_frame: Option<(f64, Vector2<f64>)>,
    pub side: f64,
    pub tangency: TangencyMode,
}

/// A temporary positional objective, subordinate to all permanent constraints.
#[derive(Clone, Debug)]
pub struct Target2d {
    pub(crate) location: TargetLocation,
    pub(crate) position: Point2,
    pub(crate) weight: f64,
}
#[derive(Clone, Copy, Debug)]
pub(crate) enum TargetLocation {
    Point(PointRef),
    /// A fraction of a segment's or arc's span.
    Fraction(GeometryId, Fraction),
    /// A native parameter of a circle, which has no span to be a fraction of.
    Native(GeometryId, NativeParam),
}
impl Target2d {
    pub fn point(point: impl Into<PointRef>, position: Point2) -> Self {
        Self {
            location: TargetLocation::Point(point.into()),
            position,
            weight: 1.,
        }
    }
    /// Targets a fixed fraction in [0, 1] of the directed segment; validated on solve.
    pub fn on_segment(line: SegmentId, fraction: Fraction, position: Point2) -> Self {
        Self {
            location: TargetLocation::Fraction(line.0, fraction),
            position,
            weight: 1.,
        }
    }
    /// Targets a fixed fraction in [0, 1] of the directed arc; validated on solve.
    pub fn on_arc(arc: ArcId, fraction: Fraction, position: Point2) -> Self {
        Self {
            location: TargetLocation::Fraction(arc.0, fraction),
            position,
            weight: 1.,
        }
    }
    /// Targets the circle's point at a native parameter, its angle from the imported frame.
    pub fn on_circle(circle: CircleId, parameter: NativeParam, position: Point2) -> Self {
        Self {
            location: TargetLocation::Native(circle.0, parameter),
            position,
            weight: 1.,
        }
    }
    /// Sets a positive relative importance among positional objectives.
    pub fn with_weight(mut self, weight: f64) -> Self {
        self.weight = weight;
        self
    }
    pub(crate) fn geometry(&self) -> GeometryId {
        match self.location {
            TargetLocation::Point(p) => p.geometry,
            TargetLocation::Fraction(g, _) | TargetLocation::Native(g, _) => g,
        }
    }
}
