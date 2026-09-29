//! Owned-shape fillet and chamfer operations.

use crate::builders::blend::{BlendError, BlendTarget};
use crate::builders::chamfer::chamfer;
use crate::builders::fillet::fillet;
use crate::topology::payload::Payload;
use crate::topology::shape::{FaceTag, ProfileTag, Shape, ShapeKind, SolidTag};

mod sealed {
    pub trait Sealed {}
    impl Sealed for crate::topology::shape::ProfileTag {}
    impl Sealed for crate::topology::shape::FaceTag {}
    impl Sealed for crate::topology::shape::SolidTag {}
}

/// Owned shape kinds whose primary entity survives a blend.
pub trait BlendInput: ShapeKind + sealed::Sealed {}
impl BlendInput for ProfileTag {}
impl BlendInput for FaceTag {}
impl BlendInput for SolidTag {}

/// Returns an owned shape with its selected corners or edges rounded.
///
/// The source shape is consumed. Selections use keys from its model; the
/// builder resolves vertices, edges, profiles, and faces as one set.
pub fn filleted<K: BlendInput, P: Payload>(
    mut shape: Shape<K, P>,
    target: impl Into<BlendTarget>,
    radius: f64,
) -> Result<Shape<K, P>, BlendError> {
    fillet(shape.model_mut(), target, radius)?;
    Ok(shape)
}

/// Returns an owned shape with its selected corners or edges chamfered.
pub fn chamfered<K: BlendInput, P: Payload>(
    mut shape: Shape<K, P>,
    target: impl Into<BlendTarget>,
    distance: f64,
) -> Result<Shape<K, P>, BlendError> {
    chamfer(shape.model_mut(), target, distance)?;
    Ok(shape)
}
