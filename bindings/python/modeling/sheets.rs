use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate::modeling;

use super::super::topology::{PyFace, PySheet};
use super::common::py_sheet;

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_function(wrap_pyfunction!(from_faces, module)?)?;
    Ok(())
}

/// Joins faces along the edges they share whole into one sheet.
#[pyfunction]
pub(crate) fn from_faces(faces: Vec<PyRef<'_, PyFace>>) -> PyResult<PySheet> {
    let shapes = faces
        .iter()
        .map(|face| {
            face.inner
                .isolated_shape()
                .map_err(|error| PyValueError::new_err(error.to_string()))
        })
        .collect::<PyResult<Vec<_>>>()?;
    let references = shapes.iter().collect::<Vec<_>>();
    modeling::sheets::from_faces(&references)
        .map_err(|error| PyValueError::new_err(error.to_string()))
        .and_then(py_sheet)
}
