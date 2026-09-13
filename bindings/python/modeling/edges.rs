use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate::geometry::Point3;
use crate::modeling;

use super::super::topology::PyEdge;
use super::common::py_edge;

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_function(wrap_pyfunction!(line, module)?)
}

#[pyfunction]
pub(crate) fn line(start: (f64, f64, f64), end: (f64, f64, f64)) -> PyResult<PyEdge> {
    modeling::edges::line(
        Point3::new(start.0, start.1, start.2),
        Point3::new(end.0, end.1, end.2),
    )
    .map_err(|error| PyValueError::new_err(error.to_string()))
    .and_then(py_edge)
}
