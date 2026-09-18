use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate::builders::loft::LoftOptions;
use crate::geometry::Degree;
use crate::modeling;

use super::super::topology::{PyFace, PyProfile, PySheet, PySolid};
use super::common::{py_sheet, py_solid};

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_function(wrap_pyfunction!(profiles, module)?)?;
    module.add_function(wrap_pyfunction!(faces, module)?)?;
    Ok(())
}

/// Skins an ordered sequence of profiles into one sheet.
#[pyfunction]
#[pyo3(signature = (sections, v_degree=None))]
pub(crate) fn profiles(
    sections: Vec<PyRef<'_, PyProfile>>,
    v_degree: Option<usize>,
) -> PyResult<PySheet> {
    let shapes = sections
        .iter()
        .map(|section| {
            section
                .inner
                .isolated_shape()
                .map_err(|error| PyValueError::new_err(error.to_string()))
        })
        .collect::<PyResult<Vec<_>>>()?;
    let references = shapes.iter().collect::<Vec<_>>();
    modeling::loft::loft(&references, loft_options(v_degree)?)
        .map_err(|error| PyValueError::new_err(error.to_string()))
        .and_then(py_sheet)
}

/// Skins an ordered sequence of faces into one solid, capped at both ends.
#[pyfunction]
#[pyo3(signature = (sections, v_degree=None))]
pub(crate) fn faces(
    sections: Vec<PyRef<'_, PyFace>>,
    v_degree: Option<usize>,
) -> PyResult<PySolid> {
    let shapes = sections
        .iter()
        .map(|section| {
            section
                .inner
                .isolated_shape()
                .map_err(|error| PyValueError::new_err(error.to_string()))
        })
        .collect::<PyResult<Vec<_>>>()?;
    let references = shapes.iter().collect::<Vec<_>>();
    modeling::loft::loft(&references, loft_options(v_degree)?)
        .map_err(|error| PyValueError::new_err(error.to_string()))
        .and_then(py_solid)
}

/// Reads the degree across the sections, `None` leaving it to the loft.
fn loft_options(v_degree: Option<usize>) -> PyResult<LoftOptions> {
    let v_degree = v_degree
        .map(Degree::new)
        .transpose()
        .map_err(|error| PyValueError::new_err(error.to_string()))?;
    Ok(LoftOptions { v_degree })
}
