//! The named rigid operations, and the chainable verbs on [`Shape`].
//!
//! [`builders::transform`](crate::builders::transform) stays at one function
//! per transform *kind*; this is where each named motion is spelled out,
//! because this is where a user looks for "how do I move a thing".
//!
//! Every verb here is infallible, which is what lets them be inherent methods
//! on [`Shape`] as well as free functions: **inherent means chainable means
//! infallible**, and a `?` in the middle of a chain gives back the only thing
//! an inherent method buys.
//!
//! ```no_run
//! use ngk::geometry::{Axis3, Frame};
//! use ngk::modeling::solids::block;
//! use nalgebra::Vector3;
//! use radians::Rad64;
//!
//! let bracket = block(40.0, 20.0, 6.0)?
//!     .rotated(Axis3::z(), Rad64::QUARTER_TURN)
//!     .translated(Vector3::new(0.0, 0.0, 12.0));
//! # Ok::<(), ngk::builders::solids::BlockError>(())
//! ```

use nalgebra::Vector3;
use radians::Rad64;

use crate::builders::transform::rigid;
use crate::geometry::axis::Axis3;
use crate::geometry::dim3::frame::Frame;
use crate::geometry::transform::Rigid;
use crate::topology::payload::Payload;
use crate::topology::shape::{Shape, ShapeKind};

/// Returns `shape` moved by a rigid motion.
pub fn moved<K: ShapeKind, P: Payload>(mut shape: Shape<K, P>, r: Rigid) -> Shape<K, P> {
    rigid(shape.model_mut(), &r);
    shape
}

/// Returns `shape` translated along `offset`.
pub fn translated<K: ShapeKind, P: Payload>(
    shape: Shape<K, P>,
    offset: Vector3<f64>,
) -> Shape<K, P> {
    moved(shape, Rigid::translation(offset))
}

/// Returns `shape` rotated by `angle` around `axis`.
pub fn rotated<K: ShapeKind, P: Payload>(
    shape: Shape<K, P>,
    axis: Axis3,
    angle: Rad64,
) -> Shape<K, P> {
    moved(shape, Rigid::rotation(axis, angle))
}

/// Returns `shape` carried from the `from` frame onto the `to` frame.
///
/// This is the spelling of "put this part where that mating face is".
pub fn placed<K: ShapeKind, P: Payload>(
    shape: Shape<K, P>,
    from: &Frame,
    to: &Frame,
) -> Shape<K, P> {
    moved(shape, Rigid::between_frames(from, to))
}

/// The chainable rigid verbs.
///
/// These exist as inherent methods precisely because they cannot fail; the
/// fallible operations live as free functions beside every other fallible
/// operation in the crate.
impl<K: ShapeKind, P: Payload> Shape<K, P> {
    /// This shape moved by a rigid motion.
    pub fn moved(self, r: Rigid) -> Self {
        moved(self, r)
    }

    /// This shape translated along `offset`.
    pub fn translated(self, offset: Vector3<f64>) -> Self {
        translated(self, offset)
    }

    /// This shape rotated by `angle` around `axis`.
    pub fn rotated(self, axis: Axis3, angle: Rad64) -> Self {
        rotated(self, axis, angle)
    }

    /// This shape carried from the `from` frame onto the `to` frame.
    pub fn placed(self, from: &Frame, to: &Frame) -> Self {
        placed(self, from, to)
    }
}
