use nalgebra::{UnitVector2, UnitVector3};

use crate::geometry::{
    Point3, dim3::utils::IntoUnit3, dim3::utils::Point2, tolerance::LINEAR_TOLERANCE_SQUARED,
};

pub struct Axis2 {
    pub origin: Point2,
    pub direction: UnitVector2<f64>,
}

#[derive(Clone, Copy)]
pub struct Axis3 {
    pub origin: Point3,
    pub direction: UnitVector3<f64>,
}

impl Axis3 {
    pub fn new(origin: Point3, direction: impl IntoUnit3) -> Self {
        Self {
            origin,
            direction: direction.normalized(),
        }
    }
    pub fn project(&self, point: Point3) -> Point3 {
        let dir = self.direction;
        let len_sq = dir.norm_squared();
        if len_sq < LINEAR_TOLERANCE_SQUARED {
            return self.origin;
        }
        self.origin + *dir * ((point - self.origin).dot(&dir) / len_sq)
    }
}
