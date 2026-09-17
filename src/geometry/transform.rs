//! Rigid motions.
//!
//! A [`Rigid`] is a rotation followed by a translation: the motion a user asks
//! for almost every time — place this here, turn it there, sit this part on
//! that face. It is deliberately smaller than a general affine map, and every
//! operation on it is total.
//!
//! # Why every rigid operation is infallible
//!
//! Distances, angles and handedness are preserved, so:
//!
//! - every curve's parameterization survives, and a parameter interval computed
//!   on the source stays valid on the image;
//! - every surface's parameterization survives, so no pcurve and no loop
//!   definition moves — stored parameter-space data comes out bit-identical;
//! - the determinant is `+1`, so no solid turns inside out;
//! - every analytic type survives as itself, so nothing degrades to NURBS.
//!
//! That is what lets `moved` return a bare value everywhere instead of a
//! `Result` and a parameter remap.
//!
//! # Who applies what
//!
//! **Whoever owns the match owns the method.** [`Point3`] and
//! [`Vector3`] are nalgebra aliases with no match to dispatch over and no room
//! for an inherent method, so the transform applies them:
//! [`Rigid::apply`], [`Rigid::apply_vector`], [`Rigid::apply_unit`]. A
//! [`Curve`](crate::geometry::Curve), a
//! [`Surface`](crate::geometry::Surface), a [`Frame`] or a
//! [`BBox`](crate::geometry::BBox) owns a match over its own variants, so it
//! applies itself: `curve.moved(&r)`.
//!
//! # Why a quaternion rather than a matrix
//!
//! Every analytic support in the kernel holds a [`Frame`], and a `Frame` is
//! orthonormal by construction. Pushing a motion through a general matrix and
//! re-deriving frame axes on every application drifts: composing a rotation
//! with itself sixty times to lay out a circular pattern leaves the sixtieth
//! frame measurably out of square. A unit quaternion renormalizes to exactly
//! the constraint it must satisfy, so composition stays rigid however long the
//! chain.

use nalgebra::{Isometry3, Matrix3, Rotation3, Translation3, UnitQuaternion, UnitVector3, Vector3};
use radians::Rad64;
use serde::{Deserialize, Serialize};

use crate::geometry::axis::Axis3;
use crate::geometry::dim3::frame::Frame;
use crate::geometry::dim3::utils::Point3;

/// A rigid motion: a rotation followed by a translation.
///
/// No scale, no shear, no reflection. See the [module
/// documentation](self) for why that makes every operation on this type total.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Rigid(Isometry3<f64>);

impl Rigid {
    /// The motion that moves nothing.
    pub fn identity() -> Self {
        Self(Isometry3::identity())
    }

    /// A pure translation along `offset`.
    pub fn translation(offset: Vector3<f64>) -> Self {
        Self(Isometry3::from_parts(
            Translation3::from(offset),
            UnitQuaternion::identity(),
        ))
    }

    /// A rotation of `angle` around `axis`, which need not pass through the
    /// origin.
    pub fn rotation(axis: Axis3, angle: Rad64) -> Self {
        let rotation = UnitQuaternion::from_axis_angle(&axis.direction, angle.val());
        let offset = axis.origin.coords - rotation * axis.origin.coords;
        Self(Isometry3::from_parts(Translation3::from(offset), rotation))
    }

    /// The motion that carries `from` onto `to`, origin onto origin and axis
    /// onto axis.
    pub fn between_frames(from: &Frame, to: &Frame) -> Self {
        let rotation = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(
            basis(to) * basis(from).transpose(),
        ));
        let offset = to.origin.coords - rotation * from.origin.coords;
        Self(Isometry3::from_parts(Translation3::from(offset), rotation))
    }

    /// The motion that applies `self` first and `other` after it.
    ///
    /// The argument order follows
    /// [`Orientation::compose`](crate::topology::orientation::Orientation::compose),
    /// the convention this crate already uses. `compose` is the documented
    /// spelling rather than an operator precisely because `*` has no argument
    /// names to hang the order on, and every kernel gets this wrong once.
    pub fn compose(self, other: Self) -> Self {
        Self(other.0 * self.0)
    }

    /// The motion that undoes this one. Total, like everything else here.
    pub fn inverse(self) -> Self {
        Self(self.0.inverse())
    }

    /// The image of `point`.
    pub fn apply(self, point: Point3) -> Point3 {
        self.0 * point
    }

    /// The image of `vector`, which the translation does not touch.
    pub fn apply_vector(self, vector: Vector3<f64>) -> Vector3<f64> {
        self.0.rotation * vector
    }

    /// The image of a unit direction, still unit.
    ///
    /// Rotating a unit vector by a unit quaternion cannot change its length, so
    /// this needs no renormalization — which is the whole reason the rotation
    /// is stored as a quaternion.
    pub fn apply_unit(self, direction: UnitVector3<f64>) -> UnitVector3<f64> {
        self.0.rotation * direction
    }

}

impl Default for Rigid {
    fn default() -> Self {
        Self::identity()
    }
}

/// The frame's axes as the columns of a rotation matrix.
fn basis(frame: &Frame) -> Matrix3<f64> {
    Matrix3::from_columns(&[
        frame.x_dir.into_inner(),
        frame.y_dir.into_inner(),
        frame.z_dir.into_inner(),
    ])
}
