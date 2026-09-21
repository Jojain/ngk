//! Geometric measures and mass properties for NGK topology views.
//!
//! Surface and volume measurements use the existing face tessellator. The
//! tessellation is computation-only: its triangle winding supplies the sign
//! of a closed shell, while surface and linear measures use positive density.

mod integrate;
mod types;

pub use types::{
    Area, Inertia, Length, LinearProperties, PrincipalInertia, SurfaceProperties, Volume,
    VolumeProperties,
};

pub(crate) use integrate::{
    combine_surface_properties, face_surface_properties, linear_properties_for_edge,
    linear_properties_for_edges, sheet_surface_properties, signed_shell_volume,
    signed_volume_for_faces, solid_volume_properties,
};

pub use integrate::MeasureError;
