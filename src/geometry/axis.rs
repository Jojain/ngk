use nalgebra::{Const, OVector, Point, Unit};
use serde::{Deserialize, Serialize};

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

pub type Axis3 = Axis<3>;
pub type Axis2 = Axis<2>;
