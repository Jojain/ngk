//! From a solved constraint system to the profiles it draws.

mod build;
mod error;
mod joints;

pub use build::{ProfilesFromSystem, SystemProfile, add_profiles_from_system};
pub use error::ProfilesFromSystemError;
