//! Binary STL export in model coordinates, without unit conversion.
//!
//! The exporter tessellates through the kernel's shared mesher, computes facet
//! normals from the serialized vertices, and validates the complete mesh before
//! writing. STL carries no units, topology keys, names, colors or smooth normals.

mod error;
#[cfg(not(target_arch = "wasm32"))]
mod fs;
mod options;
mod write;

pub use error::StlError;
#[cfg(not(target_arch = "wasm32"))]
pub use fs::write_stl_file;
pub use options::StlWriteOptions;
pub use write::{stl_to_bytes, write_stl};
