use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate::geometry::{Plane, Point3};
use crate::modeling;

use super::super::geometry::PyPlane;
use super::super::topology::{PyFace, PyProfile};
use super::common::py_face;

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_function(wrap_pyfunction!(rectangle, module)?)?;
    module.add_function(wrap_pyfunction!(polygon, module)?)?;
    module.add_function(wrap_pyfunction!(circle, module)?)?;
    module.add_function(wrap_pyfunction!(annulus, module)?)?;
    module.add_function(wrap_pyfunction!(from_profile, module)?)?;
    Ok(())
}

#[pyfunction]
#[pyo3(signature = (x_size, y_size, plane=None))]
pub(crate) fn rectangle(x_size: f64, y_size: f64, plane: Option<PyPlane>) -> PyResult<PyFace> {
    let plane = plane.map_or_else(Plane::xy, |plane| plane.plane);
    modeling::faces::rectangle(plane, x_size, y_size)
        .map_err(|error| PyValueError::new_err(error.to_string()))
        .and_then(py_face)
}

#[pyfunction]
pub(crate) fn polygon(points: Vec<(f64, f64, f64)>) -> PyResult<PyFace> {
    let points = points
        .into_iter()
        .map(|(x, y, z)| Point3::new(x, y, z))
        .collect::<Vec<_>>();
    modeling::faces::polygon(&points)
        .map_err(|error| PyValueError::new_err(error.to_string()))
        .and_then(py_face)
}

#[pyfunction]
#[pyo3(signature = (radius, plane=None))]
pub(crate) fn circle(radius: f64, plane: Option<PyPlane>) -> PyResult<PyFace> {
    let plane = plane.map_or_else(Plane::xy, |plane| plane.plane);
    modeling::faces::circle(plane, radius)
        .map_err(|error| PyValueError::new_err(error.to_string()))
        .and_then(py_face)
}

#[pyfunction]
#[pyo3(signature = (outer_radius, inner_radius, plane=None))]
pub(crate) fn annulus(
    outer_radius: f64,
    inner_radius: f64,
    plane: Option<PyPlane>,
) -> PyResult<PyFace> {
    let plane = plane.map_or_else(Plane::xy, |plane| plane.plane);
    modeling::faces::annulus(plane, outer_radius, inner_radius)
        .map_err(|error| PyValueError::new_err(error.to_string()))
        .and_then(py_face)
}

#[pyfunction]
pub(crate) fn from_profile(profile: PyRef<'_, PyProfile>) -> PyResult<PyFace> {
    let shape = profile
        .inner
        .isolated_shape()
        .map_err(|error| PyValueError::new_err(error.to_string()))?;
    modeling::faces::from_profile(&shape)
        .map_err(|error| PyValueError::new_err(error.to_string()))
        .and_then(py_face)
}
