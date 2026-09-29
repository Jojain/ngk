use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyModule;
use radians::Rad64;

use crate::geometry::{Plane, Point3};
use crate::modeling;

use super::super::geometry::PyPlane;
use super::super::topology::{PyEdge, PyProfile};
use super::common::py_profile;

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_function(wrap_pyfunction!(rectangle, module)?)?;
    module.add_function(wrap_pyfunction!(square, module)?)?;
    module.add_function(wrap_pyfunction!(polyline, module)?)?;
    module.add_function(wrap_pyfunction!(polygon, module)?)?;
    module.add_function(wrap_pyfunction!(arc, module)?)?;
    module.add_function(wrap_pyfunction!(from_edges, module)?)?;
    module.add_function(wrap_pyfunction!(from_edge, module)?)?;
    module.add_function(wrap_pyfunction!(appended, module)?)?;
    Ok(())
}

/// Promotes an owned edge to a one-edge profile.
#[pyfunction]
pub(crate) fn from_edge(edge: &PyEdge) -> PyResult<PyProfile> {
    edge.inner
        .isolated_shape()
        .map(|shape| shape.into_profile())
        .map_err(|error| PyValueError::new_err(error.to_string()))
        .and_then(py_profile)
}

/// Returns a profile with one connected edge appended.
#[pyfunction]
pub(crate) fn appended(profile: &PyProfile, edge: &PyEdge) -> PyResult<PyProfile> {
    let profile = profile
        .inner
        .isolated_shape()
        .map_err(|error| PyValueError::new_err(error.to_string()))?;
    let edge = edge
        .inner
        .isolated_shape()
        .map_err(|error| PyValueError::new_err(error.to_string()))?;
    modeling::profiles::appended(profile, &edge)
        .map_err(|error| PyValueError::new_err(error.to_string()))
        .and_then(py_profile)
}

#[pyfunction]
#[pyo3(signature = (size, plane=None))]
pub(crate) fn square(size: f64, plane: Option<PyPlane>) -> PyResult<PyProfile> {
    modeling::profiles::square(plane.map_or_else(Plane::xy, |value| value.plane), size)
        .map_err(|error| PyValueError::new_err(error.to_string()))
        .and_then(py_profile)
}

#[pyfunction]
pub(crate) fn polyline(points: Vec<(f64, f64, f64)>) -> PyResult<PyProfile> {
    let points = points
        .into_iter()
        .map(|(x, y, z)| Point3::new(x, y, z))
        .collect::<Vec<_>>();
    modeling::profiles::polyline(&points)
        .map_err(|error| PyValueError::new_err(error.to_string()))
        .and_then(py_profile)
}

#[pyfunction]
pub(crate) fn arc(
    plane: PyPlane,
    radius: f64,
    start_angle: f64,
    end_angle: f64,
) -> PyResult<PyProfile> {
    modeling::profiles::arc(
        plane.plane,
        radius,
        Rad64::new(start_angle),
        Rad64::new(end_angle),
    )
    .map_err(|error| PyValueError::new_err(error.to_string()))
    .and_then(py_profile)
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
