use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyModule;
use radians::Rad64;

use crate::modeling;

use super::super::geometry::PyAxis3;
use super::super::topology::{PyEdge, PyFace, PyProfile, PySheet, PySolid};
use super::common::{py_face, py_sheet, py_solid};

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_function(wrap_pyfunction!(edge, module)?)?;
    module.add_function(wrap_pyfunction!(profile, module)?)?;
    module.add_function(wrap_pyfunction!(face, module)?)?;
    Ok(())
}

/// Revolves an edge around an axis to make one face.
#[pyfunction]
pub(crate) fn edge(edge: &PyEdge, axis: &PyAxis3, angle: f64) -> PyResult<PyFace> {
    let shape = edge
        .inner
        .isolated_shape()
        .map_err(|error| PyValueError::new_err(error.to_string()))?;
    modeling::revolve::revolve_edge(shape, axis.axis, Rad64::new(angle))
        .map_err(|error| PyValueError::new_err(error.to_string()))
        .and_then(py_face)
}

/// Revolves a profile around an axis to make a sheet.
#[pyfunction]
pub(crate) fn profile(profile: &PyProfile, axis: &PyAxis3, angle: f64) -> PyResult<PySheet> {
    let shape = profile
        .inner
        .isolated_shape()
        .map_err(|error| PyValueError::new_err(error.to_string()))?;
    modeling::revolve::revolve_profile(shape.profile(), axis.axis, Rad64::new(angle))
        .map_err(|error| PyValueError::new_err(error.to_string()))
        .and_then(py_sheet)
}

/// Revolves a face around an axis to make a solid.
#[pyfunction]
pub(crate) fn face(face: &PyFace, axis: &PyAxis3, angle: f64) -> PyResult<PySolid> {
    let shape = face
        .inner
        .isolated_shape()
        .map_err(|error| PyValueError::new_err(error.to_string()))?;
    modeling::revolve::revolve_face(shape, axis.axis, Rad64::new(angle))
        .map_err(|error| PyValueError::new_err(error.to_string()))
        .and_then(py_solid)
}
