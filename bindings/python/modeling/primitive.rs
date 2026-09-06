use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate::StandardPayload;
use crate::binding_common::explore::SharedGMap;
use crate::builders::boolean::BooleanOperation;
use crate::geometry::{Plane, Point3};
use crate::modeling::{edges, faces, profiles, solids};
use crate::topology::shape::{Shape, SolidTag};

use super::super::topology::{PyEdge, PyFace, PyProfile, PySolid};

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_function(wrap_pyfunction!(block, module)?)?;
    module.add_function(wrap_pyfunction!(fuse, module)?)?;
    module.add_function(wrap_pyfunction!(cut, module)?)?;
    module.add_function(wrap_pyfunction!(intersect, module)?)?;
    module.add_function(wrap_pyfunction!(line, module)?)?;
    module.add_function(wrap_pyfunction!(rectangle_profile, module)?)?;
    module.add_function(wrap_pyfunction!(rectangle_face, module)?)?;
    Ok(())
}

/// Converts an owned modeling solid into its Python shared-map wrapper.
fn py_solid(shape: Shape<SolidTag, StandardPayload>) -> PyResult<PySolid> {
    let (map, key) = shape.into_map();
    let map = SharedGMap::from_map(map);
    let inner = map
        .solid_by_key(key)
        .ok_or_else(|| PyValueError::new_err(format!("missing solid {key:?}")))?;
    Ok(PySolid::from_inner(inner))
}

#[pyfunction]
pub(crate) fn block(x: f64, y: f64, z: f64) -> PyResult<PySolid> {
    let shape = solids::block(x, y, z).map_err(|error| PyValueError::new_err(error.to_string()))?;
    py_solid(shape)
}

#[pyfunction]
pub(crate) fn fuse(first: &PySolid, second: &PySolid) -> PyResult<PySolid> {
    let shape = solids::combine_views(
        first
            .inner
            .view()
            .map_err(|error| PyValueError::new_err(error.to_string()))?,
        second
            .inner
            .view()
            .map_err(|error| PyValueError::new_err(error.to_string()))?,
        BooleanOperation::Union,
    )
    .map_err(|error| PyValueError::new_err(error.to_string()))?;
    py_solid(shape)
}

#[pyfunction]
pub(crate) fn cut(target: &PySolid, tool: &PySolid) -> PyResult<PySolid> {
    let shape = solids::combine_views(
        target
            .inner
            .view()
            .map_err(|error| PyValueError::new_err(error.to_string()))?,
        tool.inner
            .view()
            .map_err(|error| PyValueError::new_err(error.to_string()))?,
        BooleanOperation::Difference,
    )
    .map_err(|error| PyValueError::new_err(error.to_string()))?;
    py_solid(shape)
}

#[pyfunction]
pub(crate) fn intersect(first: &PySolid, second: &PySolid) -> PyResult<PySolid> {
    let shape = solids::combine_views(
        first
            .inner
            .view()
            .map_err(|error| PyValueError::new_err(error.to_string()))?,
        second
            .inner
            .view()
            .map_err(|error| PyValueError::new_err(error.to_string()))?,
        BooleanOperation::Intersection,
    )
    .map_err(|error| PyValueError::new_err(error.to_string()))?;
    py_solid(shape)
}

#[pyfunction]
pub(crate) fn line(start: (f64, f64, f64), end: (f64, f64, f64)) -> PyResult<PyEdge> {
    let start = Point3::new(start.0, start.1, start.2);
    let end = Point3::new(end.0, end.1, end.2);
    let shape =
        edges::line(start, end).map_err(|error| PyValueError::new_err(error.to_string()))?;
    let (map, key) = shape.into_map();
    let map = SharedGMap::from_map(map);
    let inner = map
        .edge_by_key(key)
        .ok_or_else(|| PyValueError::new_err(format!("missing edge {key:?}")))?;
    Ok(PyEdge::from_inner(inner))
}

#[pyfunction]
pub(crate) fn rectangle_profile(x_size: f64, y_size: f64) -> PyResult<PyProfile> {
    let shape = profiles::rectangle(Plane::xy(), x_size, y_size)
        .map_err(|error| PyValueError::new_err(error.to_string()))?;
    let (map, key) = shape.into_map();
    let map = SharedGMap::from_map(map);
    let inner = map
        .profile_by_key(key)
        .ok_or_else(|| PyValueError::new_err(format!("missing profile {key:?}")))?;
    Ok(PyProfile::from_inner(inner))
}

#[pyfunction]
pub(crate) fn rectangle_face(x_size: f64, y_size: f64) -> PyResult<PyFace> {
    let shape = faces::rectangle(Plane::xy(), x_size, y_size)
        .map_err(|error| PyValueError::new_err(error.to_string()))?;
    let (map, key) = shape.into_map();
    let map = SharedGMap::from_map(map);
    let inner = map
        .face_by_key(key)
        .ok_or_else(|| PyValueError::new_err(format!("missing face {key:?}")))?;
    Ok(PyFace::from_inner(inner))
}
