//! profiles builder API.

mod operations;
mod projection;

pub use super::sketch::{ProfilesFromSystem, SketchError, SystemProfile, add_profiles_from_system};
pub use operations::*;
pub use projection::{
    ProfilesFromProjection, ProjectionProfile, ProjectionProfilesError,
    add_profiles_from_projection,
};
