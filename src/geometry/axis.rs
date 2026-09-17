use nalgebra::{Const, OVector, Point, Unit, Vector3};
use serde::{Deserialize, Serialize};

use crate::geometry::transform::Rigid;
use crate::geometry::{dim3::utils::IntoUnit, tolerance::LINEAR_TOLERANCE_SQUARED};

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Axis<const D: usize> {
    pub origin: Point<f64, D>,
    pub direction: Unit<OVector<f64, Const<D>>>,
}

impl<const D: usize> Axis<D> {
    pub fn new(origin: Point<f64, D>, direction: impl IntoUnit<D>) -> Self {
        Self {
            origin,
            direction: direction.normalized(),
        }
    }
    pub fn from_points(start: Point<f64, D>, end: Point<f64, D>) -> Self {
        Self::new(start, end - start)
    }
    pub fn project(&self, point: Point<f64, D>) -> Point<f64, D> {
        let dir = self.direction;
        let len_sq = dir.norm_squared();
        if len_sq < LINEAR_TOLERANCE_SQUARED {
            return self.origin;
        }
        self.origin + *dir * ((point - self.origin).dot(&dir) / len_sq)
    }
}

impl Axis3 {
    /// The world `x` axis through the origin.
    pub fn x() -> Self {
        Self::new(Point::origin(), Vector3::x())
    }

    /// The world `y` axis through the origin.
    pub fn y() -> Self {
        Self::new(Point::origin(), Vector3::y())
    }

    /// The world `z` axis through the origin.
    pub fn z() -> Self {
        Self::new(Point::origin(), Vector3::z())
    }

    /// This axis under a rigid motion.
    pub fn moved(&self, r: &Rigid) -> Self {
        Self {
            origin: r.apply(self.origin),
            direction: r.apply_unit(self.direction),
        }
    }
}

pub type Axis3 = Axis<3>;
pub type Axis2 = Axis<2>;
