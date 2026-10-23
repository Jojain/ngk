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

/// Canonical editable parameters; evaluated NGK curves are produced on demand.
#[derive(Clone, Debug)]
pub(crate) struct Primitive {
    pub serial: u64,
    pub kind: Kind,
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
    pub fn valid(&self) -> bool {
        if !self.values.iter().all(|v| v.is_finite())
            || !self.circle_axis.iter().all(|v| v.is_finite())
            || self.sense.abs() != 1.
        {
            return false;
        }
        match self.kind {
            Kind::Point => true,
            Kind::Segment => self.values[0] != self.values[2] || self.values[1] != self.values[3],
            Kind::Circle => self.values[2] > 0.,
            Kind::Arc => self.values[2] > 0. && self.values[4] > 0. && self.values[4] < TAU,
        }
    }
    /// Evaluates a stable characteristic-point role from the current parameters.
    pub fn point_at(&self, role: PointRole) -> Point2 {
        let p = &self.values;
        match (self.kind, role) {
            (Kind::Segment, PointRole::End) => Point2::new(p[2], p[3]),
            (Kind::Arc, PointRole::Start | PointRole::End) => {
                let angle = p[3]
                    + if role == PointRole::End {
                        self.sense * p[4]
                    } else {
                        0.
                    };
                Point2::new(p[0] + p[2] * angle.cos(), p[1] + p[2] * angle.sin())
            }
            _ => Point2::new(p[0], p[1]),
        }
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
            self.values[2],
        );
        if self.sense < 0. {
            circle.reversed()
        } else {
            circle
        }
    }
    pub fn as_arc(&self) -> TrimmedCurve2 {
        TrimmedCurve2::arc(
            self.point_at(PointRole::Center),
            Vector2::new(self.values[3].cos(), self.values[3].sin()),
            self.values[2],
            self.sense * self.values[4],
        )
    }
}
