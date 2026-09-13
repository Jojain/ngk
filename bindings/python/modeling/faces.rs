use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate::geometry::Plane;
use crate::modeling;

use super::super::topology::PyFace;
use super::common::py_face;

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_function(wrap_pyfunction!(rectangle, module)?)
}

#[pyfunction(name = "rectangle_face")]
pub(crate) fn rectangle(x_size: f64, y_size: f64) -> PyResult<PyFace> {
    modeling::faces::rectangle(Plane::xy(), x_size, y_size)
        .map_err(|error| PyValueError::new_err(error.to_string()))
        .and_then(py_face)
}
