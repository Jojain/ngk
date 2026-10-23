use super::{EditError, keys::PointRole};
use crate::geometry::dim2::{
    curves::{Circle2, Curve2},
    trimmed::TrimmedCurve2,
    utils::Point2,
};
use nalgebra::Vector2;
use std::f64::consts::TAU;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Kind {
    Point,
    Segment,
    Circle,
    Arc,
}

/// The meaning of one stored parameter.
///
/// A parameter sits at the same position in every kind that has it, so its
/// position is a constant and naming it costs nothing at run time.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Param {
    /// A point, a segment's start, or a circle's or arc's centre.
    X,
    Y,
    /// A segment's end.
    EndX,
    EndY,
    Radius,
    /// An arc's first angle, in radians from the x axis.
    StartAngle,
    /// An arc's positive angular extent; the primitive's `sense` orients it.
    Sweep,
}

/// How a parameter is normalized: coordinates are recentred and scaled,
/// lengths scaled, angles kept in radians.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Quantity {
    X,
    Y,
    Length,
    Angle,
}

impl Kind {
    /// The parameters a primitive of this kind stores, in storage order.
    pub fn layout(self) -> &'static [Param] {
        match self {
            Kind::Point => &[Param::X, Param::Y],
            Kind::Segment => &[Param::X, Param::Y, Param::EndX, Param::EndY],
            Kind::Circle => &[Param::X, Param::Y, Param::Radius],
            Kind::Arc => &[
                Param::X,
                Param::Y,
                Param::Radius,
                Param::StartAngle,
                Param::Sweep,
            ],
        }
    }
    /// The parameters storing a characteristic point. An arc's endpoints are
    /// not stored: they are evaluated from its centre, radius and angles.
    pub fn point_params(self, role: PointRole) -> Option<(Param, Param)> {
        match (self, role) {
            (Kind::Segment, PointRole::End) => Some((Param::EndX, Param::EndY)),
            (Kind::Arc, PointRole::Start | PointRole::End) => None,
            _ => Some((Param::X, Param::Y)),
        }
    }
}

impl Param {
    /// Position within a primitive's values, the same for every kind using it.
    pub const fn index(self) -> usize {
        match self {
            Param::X => 0,
            Param::Y => 1,
            Param::EndX | Param::Radius => 2,
            Param::EndY | Param::StartAngle => 3,
            Param::Sweep => 4,
        }
    }
    pub const fn quantity(self) -> Quantity {
        match self {
            Param::X | Param::EndX => Quantity::X,
            Param::Y | Param::EndY => Quantity::Y,
            Param::Radius => Quantity::Length,
            Param::StartAngle | Param::Sweep => Quantity::Angle,
        }
    }
}

/// Canonical editable parameters; evaluated NGK curves are produced on demand.
#[derive(Clone, Debug)]
pub(crate) struct Primitive {
    pub serial: u64,
    pub kind: Kind,
    /// One value per parameter of `kind.layout()`, in that order.
    pub values: Vec<f64>,
    pub sense: f64,
    pub circle_axis: Vector2<f64>,
}

impl Primitive {
    pub fn point(point: Point2) -> Result<Self, EditError> {
        Self::checked(Kind::Point, vec![point.x, point.y], 1., Vector2::x())
    }
    pub fn segment(curve: TrimmedCurve2) -> Result<Self, EditError> {
        if !matches!(curve.curve(), Curve2::Line(_)) {
            return Err(EditError::InvalidGeometry("expected a segment"));
        }
        let (a, b) = (curve.start(), curve.end());
        Self::checked(Kind::Segment, vec![a.x, a.y, b.x, b.y], 1., Vector2::x())
    }
    pub fn circle(circle: Circle2) -> Result<Self, EditError> {
        let x = *circle.x_dir();
        let y = *circle.y_dir();
        Self::checked(
            Kind::Circle,
            vec![circle.center().x, circle.center().y, circle.radius()],
            x.perp(&y).signum(),
            x,
        )
    }
    pub fn arc(curve: TrimmedCurve2) -> Result<Self, EditError> {
        let Curve2::Circle(circle) = curve.curve() else {
            return Err(EditError::InvalidGeometry("expected a circular arc"));
        };
        let radial = curve.start() - circle.center();
        let sweep = curve.interval().delta();
        let sense = circle.x_dir().perp(&circle.y_dir()).signum() * sweep.signum();
        Self::checked(
            Kind::Arc,
            vec![
                circle.center().x,
                circle.center().y,
                circle.radius(),
                radial.y.atan2(radial.x),
                sweep.abs(),
            ],
            sense,
            Vector2::x(),
        )
    }
    fn checked(
        kind: Kind,
        values: Vec<f64>,
        sense: f64,
        circle_axis: Vector2<f64>,
    ) -> Result<Self, EditError> {
        let value = Self {
            serial: 0,
            kind,
            values,
            sense,
            circle_axis,
        };
        if value.valid() {
            Ok(value)
        } else {
            Err(EditError::InvalidGeometry(
                "non-finite or degenerate primitive",
            ))
        }
    }
    pub fn get(&self, param: Param) -> f64 {
        debug_assert!(
            self.kind.layout().contains(&param),
            "{param:?} of a {:?}",
            self.kind
        );
        self.values[param.index()]
    }
    pub fn valid(&self) -> bool {
        if !self.values.iter().all(|v| v.is_finite())
            || !self.circle_axis.iter().all(|v| v.is_finite())
            || self.sense.abs() != 1.
        {
            return false;
        }
        let v = |param| self.get(param);
        match self.kind {
            Kind::Point => true,
            Kind::Segment => v(Param::X) != v(Param::EndX) || v(Param::Y) != v(Param::EndY),
            Kind::Circle => v(Param::Radius) > 0.,
            Kind::Arc => v(Param::Radius) > 0. && v(Param::Sweep) > 0. && v(Param::Sweep) < TAU,
        }
    }
    /// Evaluates a stable characteristic-point role from the current parameters.
    pub fn point_at(&self, role: PointRole) -> Point2 {
        if let Some((x, y)) = self.kind.point_params(role) {
            return Point2::new(self.get(x), self.get(y));
        }
        let angle = self.get(Param::StartAngle)
            + if role == PointRole::End {
                self.sense * self.get(Param::Sweep)
            } else {
                0.
            };
        let radius = self.get(Param::Radius);
        Point2::new(
            self.get(Param::X) + radius * angle.cos(),
            self.get(Param::Y) + radius * angle.sin(),
        )
    }
    pub fn as_segment(&self) -> TrimmedCurve2 {
        TrimmedCurve2::segment(
            self.point_at(PointRole::Start),
            self.point_at(PointRole::End),
        )
    }
    pub fn as_circle(&self) -> Circle2 {
        let circle = Circle2::new(
            self.point_at(PointRole::Center),
            self.circle_axis,
            self.get(Param::Radius),
        );
        if self.sense < 0. {
            circle.reversed()
        } else {
            circle
        }
    }
    pub fn as_arc(&self) -> TrimmedCurve2 {
        let start = self.get(Param::StartAngle);
        TrimmedCurve2::arc(
            self.point_at(PointRole::Center),
            Vector2::new(start.cos(), start.sin()),
            self.get(Param::Radius),
            self.sense * self.get(Param::Sweep),
        )
    }
}
