//! Offsets of edges, profiles, faces, sheets and solids, and shelling.

mod errors;
mod on_surface;
mod operations;
mod reembed;
mod shell;

pub use errors::OffsetBuildError;
pub use operations::{
    EdgeOffset, FaceOffset, ProfileOffset, SheetOffset, SolidOffset, offset_edge, offset_face,
    offset_profile, offset_sheet, offset_solid,
};
pub use shell::{ShellError, SolidShelling, shell_solid};
