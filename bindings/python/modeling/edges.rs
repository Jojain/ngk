use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate::geometry::Point3;
use crate::modeling;

use super::super::geometry::PyPlane;
use super::super::topology::PyEdge;
use super::common::py_edge;

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_function(wrap_pyfunction!(line, module)?)?;
    module.add_function(wrap_pyfunction!(arc, module)?)?;
    module.add_function(wrap_pyfunction!(circle, module)?)?;
    Ok(())
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

#[pyfunction]
pub(crate) fn arc(
    plane: PyPlane,
    radius: f64,
    start_angle: f64,
    end_angle: f64,
) -> PyResult<PyEdge> {
    modeling::edges::arc(plane.plane, radius, start_angle, end_angle)
        .map_err(|error| PyValueError::new_err(error.to_string()))
        .and_then(py_edge)
}

#[pyfunction]
pub(crate) fn circle(plane: PyPlane, radius: f64) -> PyResult<PyEdge> {
    modeling::edges::circle(plane.plane, radius)
        .map_err(|error| PyValueError::new_err(error.to_string()))
        .and_then(py_edge)
}
