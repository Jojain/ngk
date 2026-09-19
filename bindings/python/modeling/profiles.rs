use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate::geometry::{Plane, Point3};
use crate::modeling;

use super::super::geometry::PyPlane;
use super::super::topology::{PyEdge, PyProfile};
use super::common::py_profile;

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_function(wrap_pyfunction!(rectangle, module)?)?;
    module.add_function(wrap_pyfunction!(polygon, module)?)?;
    module.add_function(wrap_pyfunction!(from_edges, module)?)?;
    Ok(())
}

#[pyfunction]
#[pyo3(signature = (x_size, y_size, plane=None))]
pub(crate) fn rectangle(x_size: f64, y_size: f64, plane: Option<PyPlane>) -> PyResult<PyProfile> {
    let plane = plane.map_or_else(Plane::xy, |plane| plane.plane);
    modeling::profiles::rectangle(plane, x_size, y_size)
        .map_err(|error| PyValueError::new_err(error.to_string()))
        .and_then(py_profile)
}

#[pyfunction]
pub(crate) fn polygon(points: Vec<(f64, f64, f64)>) -> PyResult<PyProfile> {
    let points = points
        .into_iter()
        .map(|(x, y, z)| Point3::new(x, y, z))
        .collect::<Vec<_>>();
    modeling::profiles::polygon(&points)
        .map_err(|error| PyValueError::new_err(error.to_string()))
        .and_then(py_profile)
}

#[pyfunction]
pub(crate) fn from_edges(edges: Vec<PyRef<'_, PyEdge>>) -> PyResult<PyProfile> {
    let shapes = edges
        .iter()
        .map(|edge| {
            edge.inner
                .isolated_shape()
                .map_err(|error| PyValueError::new_err(error.to_string()))
        })
        .collect::<PyResult<Vec<_>>>()?;
    let references = shapes.iter().collect::<Vec<_>>();
    modeling::profiles::from_edges(&references)
        .map_err(|error| PyValueError::new_err(error.to_string()))
        .and_then(py_profile)
}
