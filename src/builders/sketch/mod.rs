//! From a solved constraint system to the shapes it draws.

mod arrangement;
mod drawn;
mod error;
mod joints;
mod path;
mod pieces;
mod profiles;
mod regions;

pub use error::SketchError;
pub use profiles::{ProfilesFromSystem, SystemProfile, add_profiles_from_system};
pub use regions::{SketchFromSystem, SystemRegion, add_sketch_from_system};
