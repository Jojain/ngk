use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};

use pyo3::exceptions::{PyOSError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate::StandardPayload;
use crate::binding_common::explore::SharedModel;
use crate::exchange::step::part21::exchange_to_string;
use crate::exchange::step::{
    ImportSkip, StepError, StepReadOptions, StepWriteOptions, map_to_exchange,
    read_step as read_step_text,
};
use crate::topology::shape::{Shape, SolidTag};

use super::super::topology::PySolid;

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_function(wrap_pyfunction!(write_step, module)?)?;
    module.add_function(wrap_pyfunction!(step_to_string, module)?)?;
    module.add_function(wrap_pyfunction!(read_step, module)?)?;
    module.add_function(wrap_pyfunction!(step_from_string, module)?)?;
    module.add_class::<PyStepImport>()?;
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

    let map = solid.inner.model();
    let exchange =
        map_to_exchange(map.model(), &[solid.inner.key()], &options).map_err(step_err)?;
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

/// What one [`read_step`] produced.
///
/// The solids and what had to be given up travel together for the same reason
/// they do in Rust: a read that returns six solids is not a success if it also
/// had to drop a face, and a caller must be able to see both without asking
/// twice.
#[pyclass(name = "StepImport", module = "ngk")]
pub struct PyStepImport {
    /// The solids the file yielded, in the order it named them.
    #[pyo3(get)]
    pub solids: Vec<PySolid>,
    /// One line per thing that could not be carried across faithfully.
    #[pyo3(get)]
    pub skipped: Vec<String>,
}

#[pymethods]
impl PyStepImport {
    fn __repr__(&self) -> String {
        format!(
            "StepImport(solids={}, skipped={})",
            self.solids.len(),
            self.skipped.len()
        )
    }

    fn __len__(&self) -> usize {
        self.solids.len()
    }
}

/// Reads a STEP file into solids.
///
/// Best-effort by default: a face that cannot be assembled is dropped and
/// described in `skipped` rather than costing the file. Pass `strict=True` to
/// turn each of those into a `ValueError` instead.
#[pyfunction]
#[pyo3(signature = (path, strict=false))]
pub(crate) fn read_step(path: PathBuf, strict: bool) -> PyResult<PyStepImport> {
    let text = std::fs::read_to_string(&path)
        .map_err(|error| PyOSError::new_err(format!("{}: {error}", path.display())))?;
    step_from_string(text, strict)
}

/// Reads STEP text into solids, without touching the filesystem.
#[pyfunction]
#[pyo3(signature = (text, strict=false))]
pub(crate) fn step_from_string(text: String, strict: bool) -> PyResult<PyStepImport> {
    let options = if strict {
        StepReadOptions::strict()
    } else {
        StepReadOptions::default()
    };
    let import = read_step_text(&text, &options).map_err(step_err)?;

    let solids = import
        .shapes
        .into_iter()
        .map(py_solid)
        .collect::<PyResult<Vec<_>>>()?;
    let skipped = import
        .report
        .skipped
        .iter()
        .map(|skip| {
            format!(
                "{} on line {}: {:?}",
                entity_name(skip),
                skip.line,
                skip.reason
            )
        })
        .collect();
    Ok(PyStepImport { solids, skipped })
}

fn entity_name(skip: &ImportSkip) -> String {
    match skip.entity {
        Some(entity) => entity.to_string(),
        None => "the document".to_string(),
    }
}

/// Wraps an imported shape in the shared map every Python view holds.
fn py_solid(shape: Shape<SolidTag, StandardPayload>) -> PyResult<PySolid> {
    let (map, key) = shape.into_model();
    let map = SharedModel::from_model(map);
    let inner = map
        .solid_by_key(key)
        .ok_or_else(|| PyValueError::new_err(format!("missing solid {key:?}")))?;
    Ok(PySolid::from_inner(inner))
}
