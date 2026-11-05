use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};

use pyo3::exceptions::{PyOSError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyBytes, PyModule};

use super::super::topology::PySolid;
use crate::exchange::stl::{StlWriteOptions, stl_to_bytes as export_bytes};
use crate::tessellate::{CurveOpts, SurfaceOpts, TessellateOpts};

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_function(wrap_pyfunction!(stl_to_bytes, module)?)?;
    module.add_function(wrap_pyfunction!(write_stl, module)?)?;
    Ok(())
}

/// Exports a closed solid as binary STL in model coordinates.
#[pyfunction]
#[pyo3(signature = (solid, segments=64))]
fn stl_to_bytes<'py>(
    py: Python<'py>,
    solid: &PySolid,
    segments: usize,
) -> PyResult<Bound<'py, PyBytes>> {
    Ok(PyBytes::new_bound(py, &bytes(solid, segments)?))
}

/// Writes binary STL, returning the chosen path; geometry errors preserve it.
#[pyfunction]
#[pyo3(signature = (solid, path=None, segments=64))]
fn write_stl(solid: &PySolid, path: Option<PathBuf>, segments: usize) -> PyResult<String> {
    let data = bytes(solid, segments)?;
    static NEXT: AtomicU64 = AtomicU64::new(0);
    let path = path.unwrap_or_else(|| {
        std::env::temp_dir().join(format!(
            "ngk_{}_{}.stl",
            std::process::id(),
            NEXT.fetch_add(1, Ordering::Relaxed)
        ))
    });
    std::fs::write(&path, data)
        .map_err(|error| PyOSError::new_err(format!("{}: {error}", path.display())))?;
    Ok(path.to_string_lossy().into_owned())
}

/// Resolves the shared solid and delegates all geometry and validation to exchange.
fn bytes(solid: &PySolid, segments: usize) -> PyResult<Vec<u8>> {
    let model = solid.inner.model();
    let view = model
        .model()
        .solid(solid.inner.key())
        .ok_or_else(|| PyValueError::new_err("missing solid"))?;
    let options = StlWriteOptions {
        tessellation: TessellateOpts {
            curve: CurveOpts { segments },
            surface: SurfaceOpts {
                nu: segments,
                nv: segments / 2,
            },
        },
        ..Default::default()
    };
    export_bytes(&view, &options).map_err(|error| PyValueError::new_err(error.to_string()))
}
