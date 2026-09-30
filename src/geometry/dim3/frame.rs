use crate::geometry::axis::Axis3;
use crate::geometry::tolerance::{ANGULAR_TOLERANCE, LINEAR_TOLERANCE};
use crate::geometry::transform::Rigid;

use super::utils::{IntoUnit, Point3};
use nalgebra::{UnitVector3, Vector3};
use serde::{Deserialize, Serialize};
use thiserror::Error;

/// Why two directions cannot span a frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum FrameError {
    /// A direction is the zero vector, so it points nowhere.
    #[error("frame direction is zero")]
    ZeroDirection,
    /// The two directions are parallel or opposite, so they span no plane.
    #[error("frame directions are parallel")]
    ParallelDirections,
}

/// Normalizes two directions, refusing a zero one or a pair that spans no plane.
fn spanning_directions(
    first: impl IntoUnit<3>,
    second: impl IntoUnit<3>,
) -> Result<(UnitVector3<f64>, UnitVector3<f64>), FrameError> {
    let first = first
        .try_normalized(LINEAR_TOLERANCE)
        .ok_or(FrameError::ZeroDirection)?;
    let second = second
        .try_normalized(LINEAR_TOLERANCE)
        .ok_or(FrameError::ZeroDirection)?;
    if first.cross(&second).norm() <= ANGULAR_TOLERANCE {
        return Err(FrameError::ParallelDirections);
    }
    Ok((first, second))
}

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
    /// Creates a frame at the given origin from its x and y axes. X is kept; y is
    /// replaced by its component perpendicular to x, and z completes the frame.
    ///
    /// # Panics
    ///
    /// When either direction is zero or the two are parallel; [`Frame::try_from_xy`]
    /// reports the same cases as an error.
    pub fn from_xy(origin: Point3, x_dir: impl IntoUnit<3>, y_dir: impl IntoUnit<3>) -> Self {
        Self::try_from_xy(origin, x_dir, y_dir).unwrap_or_else(|error| panic!("{error}"))
    }
    /// Creates a frame at the given origin from its x and z axes. Z is kept; x is
    /// replaced by its component perpendicular to z, and y completes the frame.
    ///
    /// # Panics
    ///
    /// When either direction is zero or the two are parallel; [`Frame::try_from_xz`]
    /// reports the same cases as an error.
    pub fn from_xz(origin: Point3, x_dir: impl IntoUnit<3>, z_dir: impl IntoUnit<3>) -> Self {
        Self::try_from_xz(origin, x_dir, z_dir).unwrap_or_else(|error| panic!("{error}"))
    }
    /// [`Frame::from_xy`], refusing a zero direction or two that are parallel.
    pub fn try_from_xy(
        origin: Point3,
        x_dir: impl IntoUnit<3>,
        y_dir: impl IntoUnit<3>,
    ) -> Result<Self, FrameError> {
        let (x_dir, y_dir) = spanning_directions(x_dir, y_dir)?;
        let z_dir = UnitVector3::new_normalize(x_dir.cross(&y_dir));
        let y_dir = UnitVector3::new_normalize(z_dir.cross(&x_dir));
        Ok(Self {
            origin,
            x_dir,
            y_dir,
            z_dir,
        })
    }
    /// [`Frame::from_xz`], refusing a zero direction or two that are parallel.
    pub fn try_from_xz(
        origin: Point3,
        x_dir: impl IntoUnit<3>,
        z_dir: impl IntoUnit<3>,
    ) -> Result<Self, FrameError> {
        let (x_dir, z_dir) = spanning_directions(x_dir, z_dir)?;
        let y_dir = UnitVector3::new_normalize(z_dir.cross(&x_dir));
        let x_dir = UnitVector3::new_normalize(y_dir.cross(&z_dir));
        Ok(Self {
            origin,
            x_dir,
            y_dir,
            z_dir,
        })
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
