use super::{ArcKey, CircleKey, CircularKey, GeometryKey, PointRef, SegmentKey};
use crate::geometry::dim2::utils::Point2;
use Constraint2d::*;
use nalgebra::Vector2;
use std::f64::consts::PI;

/// The contact branch of two supporting circles.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TangencyMode {
    Automatic,
    External,
    /// The initially larger circle contains the other; ties select the first.
    Internal,
    FirstContainsSecond,
    SecondContainsFirst,
}

/// Permanent geometric relations. Lengths use the caller's unit, angles radians.
#[derive(Clone, Debug, PartialEq)]
pub enum Constraint2d {
    Coincident(PointRef, PointRef),
    PointOnLine(PointRef, SegmentKey),
    PointOnCircle(PointRef, CircularKey),
    Horizontal(SegmentKey),
    Vertical(SegmentKey),
    Distance(PointRef, PointRef, f64),
    DistanceToLine(PointRef, SegmentKey, f64),
    OffsetX(PointRef, PointRef, f64),
    OffsetY(PointRef, PointRef, f64),
    Length(SegmentKey, f64),
    Radius(CircularKey, f64),
    EqualLength(SegmentKey, SegmentKey),
    EqualRadius(CircularKey, CircularKey),
    Parallel(SegmentKey, SegmentKey),
    Perpendicular(SegmentKey, SegmentKey),
    /// Unsigned values lie in [0, pi]; oriented values are interpreted modulo a turn.
    Angle {
        first: SegmentKey,
        second: SegmentKey,
        radians: f64,
        oriented: bool,
    },
    Concentric(CircularKey, CircularKey),
    Midpoint {
        point: PointRef,
        start: PointRef,
        end: PointRef,
    },
    SymmetricAboutLine {
        first: PointRef,
        second: PointRef,
        line: SegmentKey,
    },
    SymmetricAboutPoint {
        first: PointRef,
        second: PointRef,
        center: PointRef,
    },
    /// Captures this point's position when the constraint is added or replaced.
    FixedPoint(PointRef),
    /// Captures all of the primitive's parameters when added or replaced.
    Fixed(GeometryKey),
    TangentLineCircle(SegmentKey, CircularKey),
    TangentCircles {
        first: CircularKey,
        second: CircularKey,
        mode: TangencyMode,
    },
}

impl Constraint2d {
    pub fn coincident(a: impl Into<PointRef>, b: impl Into<PointRef>) -> Self {
        Self::Coincident(a.into(), b.into())
    }
    pub fn point_on_line(p: impl Into<PointRef>, line: SegmentKey) -> Self {
        Self::PointOnLine(p.into(), line)
    }
    pub fn point_on_circle(p: impl Into<PointRef>, circle: impl Into<CircularKey>) -> Self {
        Self::PointOnCircle(p.into(), circle.into())
    }
    pub fn horizontal(line: SegmentKey) -> Self {
        Self::Horizontal(line)
    }
    pub fn vertical(line: SegmentKey) -> Self {
        Self::Vertical(line)
    }
    pub fn distance(a: impl Into<PointRef>, b: impl Into<PointRef>, value: f64) -> Self {
        Self::Distance(a.into(), b.into(), value)
    }
    pub fn distance_to_line(p: impl Into<PointRef>, line: SegmentKey, value: f64) -> Self {
        Self::DistanceToLine(p.into(), line, value)
    }
    pub fn offset_x(a: impl Into<PointRef>, b: impl Into<PointRef>, value: f64) -> Self {
        Self::OffsetX(a.into(), b.into(), value)
    }
    pub fn offset_y(a: impl Into<PointRef>, b: impl Into<PointRef>, value: f64) -> Self {
        Self::OffsetY(a.into(), b.into(), value)
    }
    pub fn length(line: SegmentKey, value: f64) -> Self {
        Self::Length(line, value)
    }
    pub fn radius(circle: impl Into<CircularKey>, value: f64) -> Self {
        Self::Radius(circle.into(), value)
    }
    pub fn diameter(circle: impl Into<CircularKey>, value: f64) -> Self {
        Self::Radius(circle.into(), value / 2.)
    }
    pub fn equal_length(a: SegmentKey, b: SegmentKey) -> Self {
        Self::EqualLength(a, b)
    }
    pub fn equal_radius(a: impl Into<CircularKey>, b: impl Into<CircularKey>) -> Self {
        Self::EqualRadius(a.into(), b.into())
    }
    pub fn parallel(a: SegmentKey, b: SegmentKey) -> Self {
        Self::Parallel(a, b)
    }
    pub fn perpendicular(a: SegmentKey, b: SegmentKey) -> Self {
        Self::Perpendicular(a, b)
    }
    pub fn angle(first: SegmentKey, second: SegmentKey, radians: f64) -> Self {
        Self::Angle {
            first,
            second,
            radians,
            oriented: false,
        }
    }
    pub fn oriented_angle(first: SegmentKey, second: SegmentKey, radians: f64) -> Self {
        Self::Angle {
            first,
            second,
            radians,
            oriented: true,
        }
    }
    pub fn concentric(a: impl Into<CircularKey>, b: impl Into<CircularKey>) -> Self {
        Self::Concentric(a.into(), b.into())
    }
    pub fn midpoint(point: impl Into<PointRef>, line: SegmentKey) -> Self {
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
        line: SegmentKey,
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
    pub fn fixed(geometry: impl Into<GeometryKey>) -> Self {
        Self::Fixed(geometry.into())
    }
    pub fn tangent_line_circle(line: SegmentKey, circle: impl Into<CircularKey>) -> Self {
        Self::TangentLineCircle(line, circle.into())
    }
    pub fn tangent_circles(
        first: impl Into<CircularKey>,
        second: impl Into<CircularKey>,
        mode: TangencyMode,
    ) -> Self {
        Self::TangentCircles {
            first: first.into(),
            second: second.into(),
            mode,
        }
    }

    /// Lists owning primitives; computed points do not introduce extra ownership.
    pub(crate) fn geometries(&self) -> Vec<GeometryKey> {
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
                radians, oriented, ..
            } => radians.is_finite() && (*oriented || (0. ..=PI).contains(radians)),
            _ => true,
        }
    }
}

#[derive(Clone, Debug)]
pub(crate) struct StoredConstraint {
    pub serial: u64,
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
    Sample(GeometryKey, f64),
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
    pub fn on_segment(line: SegmentKey, fraction: f64, position: Point2) -> Self {
        Self {
            location: TargetLocation::Sample(line.0, fraction),
            position,
            weight: 1.,
        }
    }
    /// Targets a fixed fraction in [0, 1] of the directed arc; validated on solve.
    pub fn on_arc(arc: ArcKey, fraction: f64, position: Point2) -> Self {
        Self {
            location: TargetLocation::Sample(arc.0, fraction),
            position,
            weight: 1.,
        }
    }
    /// Targets the circle's point at a native angle relative to its imported frame.
    pub fn on_circle(circle: CircleKey, radians: f64, position: Point2) -> Self {
        Self {
            location: TargetLocation::Sample(circle.0, radians),
            position,
            weight: 1.,
        }
    }
    /// Sets a positive relative importance among positional objectives.
    pub fn with_weight(mut self, weight: f64) -> Self {
        self.weight = weight;
        self
    }
    pub(crate) fn geometry(&self) -> GeometryKey {
        match self.location {
            TargetLocation::Point(p) => p.geometry,
            TargetLocation::Sample(g, _) => g,
        }
    }
}
