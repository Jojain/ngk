use std::path::Path;

use super::{StlError, StlWriteOptions, stl_to_bytes};
use crate::tessellate::Tessellate;

/// Writes a binary STL file. Tessellation and validation finish before the
/// destination is opened, so a geometry failure preserves any existing file.
pub fn write_stl_file(
    path: impl AsRef<Path>,
    shape: &(impl Tessellate + ?Sized),
    options: &StlWriteOptions,
) -> Result<(), StlError> {
    let bytes = stl_to_bytes(shape, options)?;
    std::fs::write(path, bytes)?;
    Ok(())
}
