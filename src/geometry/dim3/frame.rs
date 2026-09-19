use crate::geometry::axis::Axis3;
use crate::geometry::transform::Rigid;

use super::utils::{IntoUnit, Point3};
use nalgebra::{UnitVector3, Vector3};
use serde::{Deserialize, Serialize};

/// A 3D coordinate frame with an origin and three orthonormal axes.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Frame {
    pub origin: Point3,
    pub x_dir: UnitVector3<f64>,
    pub y_dir: UnitVector3<f64>,
    pub z_dir: UnitVector3<f64>,
}

impl Frame {
    /// Creates a frame at the origin with axes aligned to the world axes.
    pub fn xyz() -> Self {
        Self {
            origin: Point3::new(0.0, 0.0, 0.0),
            x_dir: Vector3::x_axis(),
            y_dir: Vector3::y_axis(),
            z_dir: Vector3::z_axis(),
        }
    }
    /// Creates a frame at the given origin with the specified x and y axes. The z axis is computed to be orthogonal to both.
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
    /// Creates a frame at the given origin with the specified x and z axes. The y axis is computed to be orthogonal to both.
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
    /// Creates a frame at the given origin with axes aligned to the world axes.
    pub fn at(origin: Point3) -> Self {
        Self {
            origin,
            x_dir: Vector3::x_axis(),
            y_dir: Vector3::y_axis(),
            z_dir: Vector3::z_axis(),
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

    /// This frame under a rigid motion.
    pub fn moved(&self, r: &Rigid) -> Self {
        Self {
            origin: r.apply(self.origin),
            x_dir: r.apply_unit(self.x_dir),
            y_dir: r.apply_unit(self.y_dir),
            z_dir: r.apply_unit(self.z_dir),
        }
    }

    /// Returns the coordinates of a point in this frame's local coordinate system.
    pub fn coordinates_of(&self, point: Point3) -> Vector3<f64> {
        let offset = point - self.origin;
        Vector3::new(
            offset.dot(self.x_dir.as_ref()),
            offset.dot(self.y_dir.as_ref()),
            offset.dot(self.z_dir.as_ref()),
        )
    }

    /// Returns the world point at the given coordinates in this frame's local coordinate system.
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
