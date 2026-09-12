use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};

use pyo3::exceptions::{PyOSError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate::exchange::step::part21::exchange_to_string;
use crate::exchange::step::{StepError, StepWriteOptions, map_to_exchange};

use super::super::topology::PySolid;

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_function(wrap_pyfunction!(write_step, module)?)?;
    module.add_function(wrap_pyfunction!(step_to_string, module)?)?;
    Ok(())
}

/// Writes a solid to a STEP file and returns the path it was written to.
#[pyfunction]
#[pyo3(signature = (solid, path=None, name=None))]
pub(crate) fn write_step(
    solid: &PySolid,
    path: Option<PathBuf>,
    name: Option<String>,
) -> PyResult<String> {
    let text = step_text(solid, name)?;
    let path = path.unwrap_or_else(scratch_path);
    std::fs::write(&path, text)
        .map_err(|error| PyOSError::new_err(format!("{}: {error}", path.display())))?;
    Ok(path.to_string_lossy().into_owned())
}

/// Writes a solid as STEP text, without touching the filesystem.
#[pyfunction]
#[pyo3(signature = (solid, name=None))]
pub(crate) fn step_to_string(solid: &PySolid, name: Option<String>) -> PyResult<String> {
    step_text(solid, name)
}

fn step_text(solid: &PySolid, name: Option<String>) -> PyResult<String> {
    let options = match name {
        Some(name) => StepWriteOptions::named(name),
        None => StepWriteOptions::default(),
    };

    let map = solid.inner.gmap();
    let exchange = map_to_exchange(map.map(), &[solid.inner.key()], &options).map_err(step_err)?;
    exchange_to_string(&exchange).map_err(|error| PyValueError::new_err(error.to_string()))
}

/// Maps an export failure onto the Python exception that fits it.
///
/// A filesystem failure is an `OSError` in Python, while geometry this stage
/// cannot write is a `ValueError` about the argument — which is what it is.
fn step_err(error: StepError) -> PyErr {
    match error {
        StepError::Io { .. } => PyOSError::new_err(error.to_string()),
        other => PyValueError::new_err(other.to_string()),
    }
}

/// Returns a fresh path in the system temp directory.
///
/// Only used when the caller names none, which is the point of the default:
/// experimenting from a REPL should not require choosing a filename. Nothing
/// cleans these up — they live in the temp directory precisely so the system
/// eventually does.
fn scratch_path() -> PathBuf {
    static NEXT: AtomicU64 = AtomicU64::new(0);
    let unique = NEXT.fetch_add(1, Ordering::Relaxed);
    std::env::temp_dir().join(format!("ngk_{}_{unique}.step", std::process::id()))
}
