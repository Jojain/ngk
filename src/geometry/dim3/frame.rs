use crate::geometry::axis::Axis3;

use super::utils::{IntoUnit, Point3};
use nalgebra::{UnitVector3, Vector3};

#[derive(Clone)]
pub struct Frame {
    pub origin: Point3,
    pub x_dir: UnitVector3<f64>,
    pub y_dir: UnitVector3<f64>,
    pub z_dir: UnitVector3<f64>,
}

impl Frame {
    pub fn xyz() -> Self {
        Self {
            origin: Point3::new(0.0, 0.0, 0.0),
            x_dir: Vector3::x_axis(),
            y_dir: Vector3::y_axis(),
            z_dir: Vector3::z_axis(),
        }
    }
    pub fn from_xy(origin: Point3, x_dir: impl IntoUnit<3>, y_dir: impl IntoUnit<3>) -> Self {
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

    pub fn from_xz(origin: Point3, x_dir: impl IntoUnit<3>, z_dir: impl IntoUnit<3>) -> Self {
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

    pub fn x_axis(&self) -> Axis3 {
        Axis3::new(self.origin, self.x_dir)
    }
    pub fn y_axis(&self) -> Axis3 {
        Axis3::new(self.origin, self.y_dir)
    }
    pub fn z_axis(&self) -> Axis3 {
        Axis3::new(self.origin, self.z_dir)
    }

    pub fn coordinates_of(&self, point: Point3) -> Vector3<f64> {
        let offset = point - self.origin;
        Vector3::new(
            offset.dot(self.x_dir.as_ref()),
            offset.dot(self.y_dir.as_ref()),
            offset.dot(self.z_dir.as_ref()),
        )
    }

    pub fn point_at(&self, coordinates: Vector3<f64>) -> Point3 {
        let offset = self.x_dir.as_ref() * coordinates.x
            + self.y_dir.as_ref() * coordinates.y
            + self.z_dir.as_ref() * coordinates.z;
        self.origin + offset
    }
}

impl Default for Frame {
    fn default() -> Self {
        Self::xyz()
    }
}
