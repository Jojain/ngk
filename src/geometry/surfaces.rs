use super::utils::{IntoUnit3, Point3};
use nalgebra::UnitVector3;

#[derive(Clone)]
pub enum Surface {
    Plane(Plane),
}

#[derive(Clone)]
pub struct Plane {
    pub origin: Point3,
    pub x_dir: UnitVector3<f64>,
    pub normal: UnitVector3<f64>,
}

impl Plane {
    pub fn new(origin: Point3, x_dir: impl IntoUnit3, normal: impl IntoUnit3) -> Self {
        Self {
            origin,
            x_dir: x_dir.normalized(),
            normal: normal.normalized(),
        }
    }
    pub fn from_xy(origin: Point3, x_dir: impl IntoUnit3, y_dir: impl IntoUnit3) -> Self {
        let x_dir = x_dir.normalized();
        let y_dir = y_dir.normalized();
        let normal = UnitVector3::new_normalize(x_dir.cross(&y_dir));
        Self {
            origin,
            x_dir: x_dir.normalized(),
            normal,
        }
    }
}
