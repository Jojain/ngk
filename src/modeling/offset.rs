//! Owned-shape offsets and shelling.
//!
//! The offset is a new entity in the input shape's model, which keeps the
//! source beside it: the returned shape's primary entity is the moved copy.

use nalgebra::UnitVector3;

use crate::builders::offset::{
    OffsetBuildError, ShellError, offset_edge, offset_face, offset_profile, offset_sheet,
    offset_solid, shell_solid,
};
use crate::model::Model;
use crate::topology::payload::Payload;
use crate::topology::shape::{EdgeTag, FaceTag, ProfileTag, Shape, ShapeKind, SheetTag, SolidTag};
use crate::topology::shape_keys::FaceKey;

mod sealed {
    pub trait Sealed {}
    impl Sealed for crate::topology::shape::EdgeTag {}
    impl Sealed for crate::topology::shape::ProfileTag {}
    impl Sealed for crate::topology::shape::FaceTag {}
    impl Sealed for crate::topology::shape::SheetTag {}
    impl Sealed for crate::topology::shape::SolidTag {}
}

/// Shape kinds that move along their faces' normals: faces, sheets, solids.
pub trait SurfaceOffsetInput: ShapeKind + sealed::Sealed {
    #[doc(hidden)]
    fn offset_in<P: Payload>(
        model: &mut Model<P>,
        handle: Self::Handle,
        distance: f64,
    ) -> Result<Self::Handle, OffsetBuildError>;
}

/// Shape kinds that move along `T × direction`: edges and profiles.
pub trait WireOffsetInput: ShapeKind + sealed::Sealed {
    #[doc(hidden)]
    fn offset_in<P: Payload>(
        model: &mut Model<P>,
        handle: Self::Handle,
        direction: UnitVector3<f64>,
        distance: f64,
    ) -> Result<Self::Handle, OffsetBuildError>;
}

impl SurfaceOffsetInput for FaceTag {
    fn offset_in<P: Payload>(
        model: &mut Model<P>,
        handle: Self::Handle,
        distance: f64,
    ) -> Result<Self::Handle, OffsetBuildError> {
        Ok(offset_face(model, handle, distance)?.face)
    }
}

impl SurfaceOffsetInput for SheetTag {
    fn offset_in<P: Payload>(
        model: &mut Model<P>,
        handle: Self::Handle,
        distance: f64,
    ) -> Result<Self::Handle, OffsetBuildError> {
        Ok(offset_sheet(model, handle, distance)?.sheet)
    }
}

impl SurfaceOffsetInput for SolidTag {
    fn offset_in<P: Payload>(
        model: &mut Model<P>,
        handle: Self::Handle,
        distance: f64,
    ) -> Result<Self::Handle, OffsetBuildError> {
        Ok(offset_solid(model, handle, distance)?.solid)
    }
}

impl WireOffsetInput for EdgeTag {
    fn offset_in<P: Payload>(
        model: &mut Model<P>,
        handle: Self::Handle,
        direction: UnitVector3<f64>,
        distance: f64,
    ) -> Result<Self::Handle, OffsetBuildError> {
        Ok(offset_edge(model, handle, direction, distance)?.edge)
    }
}

impl WireOffsetInput for ProfileTag {
    fn offset_in<P: Payload>(
        model: &mut Model<P>,
        handle: Self::Handle,
        direction: UnitVector3<f64>,
        distance: f64,
    ) -> Result<Self::Handle, OffsetBuildError> {
        Ok(offset_profile(model, handle, direction, distance)?.profile)
    }
}

/// Returns `shape`'s face, sheet or solid moved `distance` along its normal:
/// a positive distance grows a solid.
pub fn offset<K: SurfaceOffsetInput, P: Payload>(
    shape: Shape<K, P>,
    distance: f64,
) -> Result<Shape<K, P>, OffsetBuildError> {
    shape.then(|model, handle| K::offset_in(model, handle, distance))
}

/// Returns `shape`'s edge or profile moved `distance` along `T × direction`:
/// a loop counter-clockwise about `direction` grows for a positive distance.
pub fn offset_along<K: WireOffsetInput, P: Payload>(
    shape: Shape<K, P>,
    direction: UnitVector3<f64>,
    distance: f64,
) -> Result<Shape<K, P>, OffsetBuildError> {
    shape.then(|model, handle| K::offset_in(model, handle, direction, distance))
}

/// Returns `shape`'s solid emptied to a wall `thickness` thick, open where
/// `open_faces` were; with no open face, the solid with a cavity.
pub fn shelled<P: Payload>(
    shape: Shape<SolidTag, P>,
    open_faces: &[FaceKey],
    thickness: f64,
) -> Result<Shape<SolidTag, P>, ShellError> {
    shape.then(|model, solid| Ok(shell_solid(model, solid, open_faces, thickness)?.solid))
}
