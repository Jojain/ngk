use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate::geometry::{Fraction, Plane, Point3};
use crate::modeling;
use crate::topology::shape::Shape;

use super::super::geometry::PyPlane;
use super::super::topology::{PyEdge, PyFace, PyProfile};
use super::common::py_face;
use super::edges::PyEdgeSplitResult;

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_function(wrap_pyfunction!(rectangle, module)?)?;
    module.add_function(wrap_pyfunction!(square, module)?)?;
    module.add_function(wrap_pyfunction!(polygon, module)?)?;
    module.add_function(wrap_pyfunction!(polygon_with_holes, module)?)?;
    module.add_function(wrap_pyfunction!(circle, module)?)?;
    module.add_function(wrap_pyfunction!(annulus, module)?)?;
    module.add_function(wrap_pyfunction!(from_profile, module)?)?;
    module.add_function(wrap_pyfunction!(split_boundary_edge, module)?)?;
    Ok(())
}

/// Cuts a boundary edge and updates every incident pcurve.
#[pyfunction]
pub(crate) fn split_boundary_edge(
    face: &PyFace,
    edge: &PyEdge,
    fraction: f64,
) -> PyResult<PyEdgeSplitResult> {
    let source = face.inner.model();
    if !source.ptr_eq(&edge.inner.model()) {
        return Err(PyValueError::new_err(
            "edge and face must belong to the same model",
        ));
    }
    let shape = Shape::new(source.model().clone(), face.inner.key());
    let result =
        modeling::faces::split_boundary_edge(shape, edge.inner.key(), Fraction::new(fraction))
            .map_err(|error| PyValueError::new_err(error.to_string()))?;
    Ok(PyEdgeSplitResult::from_result(result))
}

/// Builds a planar face from an outer polygon and zero or more holes.
#[pyfunction]
#[pyo3(signature = (outer, holes, plane=None))]
pub(crate) fn polygon_with_holes(
    outer: Vec<(f64, f64, f64)>,
    holes: Vec<Vec<(f64, f64, f64)>>,
    plane: Option<PyPlane>,
) -> PyResult<PyFace> {
    let outer = outer
        .into_iter()
        .map(|(x, y, z)| Point3::new(x, y, z))
        .collect::<Vec<_>>();
    let holes = holes
        .into_iter()
        .map(|hole| {
            hole.into_iter()
                .map(|(x, y, z)| Point3::new(x, y, z))
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();
    let hole_refs = holes.iter().map(Vec::as_slice).collect::<Vec<_>>();
    modeling::faces::polygon_with_holes(
        plane.map_or_else(Plane::xy, |value| value.plane),
        &outer,
        &hole_refs,
    )
    .map_err(|error| PyValueError::new_err(error.to_string()))
    .and_then(py_face)
}

#[pyfunction]
#[pyo3(signature = (size, plane=None))]
pub(crate) fn square(size: f64, plane: Option<PyPlane>) -> PyResult<PyFace> {
    modeling::faces::square(plane.map_or_else(Plane::xy, |value| value.plane), size)
        .map_err(|error| PyValueError::new_err(error.to_string()))
        .and_then(py_face)
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
