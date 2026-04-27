use super::utils::{IntoUnit3, Point3};
use nalgebra::UnitVector3;

#[derive(Clone)]
pub struct Frame {
    pub origin: Point3,
    pub x_dir: UnitVector3<f64>,
    pub y_dir: UnitVector3<f64>,
    pub z_dir: UnitVector3<f64>,
}

impl Frame {
    pub fn from_xy(origin: Point3, x_dir: impl IntoUnit3, y_dir: impl IntoUnit3) -> Self {
        let x_dir = x_dir.normalized();
        let y_dir = y_dir.normalized();
        let z_dir = UnitVector3::new_normalize(x_dir.cross(&y_dir));
        let y_dir = UnitVector3::new_normalize(z_dir.cross(&x_dir));

        Self {
            origin,
            x_dir,
            y_dir,
            z_dir,
        }
    }

    pub fn from_xz(origin: Point3, x_dir: impl IntoUnit3, z_dir: impl IntoUnit3) -> Self {
        let x_dir = x_dir.normalized();
        let z_dir = z_dir.normalized();
        let y_dir = UnitVector3::new_normalize(z_dir.cross(&x_dir));
        let x_dir = UnitVector3::new_normalize(y_dir.cross(&z_dir));

        Self {
            origin,
            x_dir,
            y_dir,
            z_dir,
        }
    }
}
