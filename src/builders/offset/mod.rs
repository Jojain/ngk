//! Offsets of edges, profiles, faces, sheets and solids.

mod errors;
mod on_surface;
mod operations;
mod reembed;

pub use errors::OffsetBuildError;
pub use operations::{
    EdgeOffset, FaceOffset, ProfileOffset, SheetOffset, SolidOffset, offset_edge, offset_face,
    offset_profile, offset_sheet, offset_solid,
};
