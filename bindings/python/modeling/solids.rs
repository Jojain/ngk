use nalgebra::Unit;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate::builders::boolean::BooleanOperation;
use crate::modeling;

use super::super::geometry::{PyFrame, PyVector3};
use super::super::topology::{PyFace, PySolid};
use super::common::py_solid;

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_function(wrap_pyfunction!(block, module)?)?;
    module.add_function(wrap_pyfunction!(cylinder, module)?)?;
    module.add_function(wrap_pyfunction!(sphere, module)?)?;
    module.add_function(wrap_pyfunction!(torus, module)?)?;
    module.add_function(wrap_pyfunction!(extruded, module)?)?;
    module.add_function(wrap_pyfunction!(fuse, module)?)?;
    module.add_function(wrap_pyfunction!(cut, module)?)?;
    module.add_function(wrap_pyfunction!(intersect, module)?)?;
    Ok(())
}

#[pyfunction]
#[pyo3(signature = (face, direction, distance))]
pub(crate) fn extruded(
    face: PyRef<'_, PyFace>,
    direction: PyVector3,
    distance: f64,
) -> PyResult<PySolid> {
    let shape = face
        .inner
        .isolated_shape()
        .map_err(|error| PyValueError::new_err(error.to_string()))?;
    if direction.vector.norm_squared() <= 0.0 {
        return Err(PyValueError::new_err(
            "extrusion direction must be non-zero",
        ));
    }
    modeling::solids::extruded(shape, Unit::new_normalize(direction.vector), distance)
        .map_err(|error| PyValueError::new_err(error.to_string()))
        .and_then(py_solid)
}

#[pyfunction]
#[pyo3(signature = (x, y, z, frame=None))]
pub(crate) fn block(x: f64, y: f64, z: f64, frame: Option<PyFrame>) -> PyResult<PySolid> {
    match frame {
        Some(frame) => modeling::solids::block_at(frame.frame, x, y, z),
        None => modeling::solids::block(x, y, z),
    }
    .map_err(|error| PyValueError::new_err(error.to_string()))
    .and_then(py_solid)
}

#[pyfunction]
#[pyo3(signature = (radius, height, frame=None))]
pub(crate) fn cylinder(
    radius: f64,
    height: f64,
    frame: Option<PyFrame>,
) -> PyResult<PySolid> {
    match frame {
        Some(frame) => modeling::solids::cylinder_at(frame.frame, radius, height),
        None => modeling::solids::cylinder(radius, height),
    }
    .map_err(|error| PyValueError::new_err(error.to_string()))
    .and_then(py_solid)
}

#[pyfunction]
#[pyo3(signature = (radius, frame=None))]
pub(crate) fn sphere(radius: f64, frame: Option<PyFrame>) -> PyResult<PySolid> {
    match frame {
        Some(frame) => modeling::solids::sphere_at(frame.frame, radius),
        None => modeling::solids::sphere(radius),
    }
    .map_err(|error| PyValueError::new_err(error.to_string()))
    .and_then(py_solid)
}

#[pyfunction]
#[pyo3(signature = (major, minor, frame=None))]
pub(crate) fn torus(major: f64, minor: f64, frame: Option<PyFrame>) -> PyResult<PySolid> {
    match frame {
        Some(frame) => modeling::solids::torus_at(frame.frame, major, minor),
        None => modeling::solids::torus(major, minor),
    }
    .map_err(|error| PyValueError::new_err(error.to_string()))
    .and_then(py_solid)
}

#[pyfunction]
pub(crate) fn fuse(first: &PySolid, second: &PySolid) -> PyResult<PySolid> {
    combine(first, second, BooleanOperation::Union)
}

#[pyfunction]
pub(crate) fn cut(target: &PySolid, tool: &PySolid) -> PyResult<PySolid> {
    combine(target, tool, BooleanOperation::Difference)
}

#[pyfunction]
pub(crate) fn intersect(first: &PySolid, second: &PySolid) -> PyResult<PySolid> {
    combine(first, second, BooleanOperation::Intersection)
}

fn combine(first: &PySolid, second: &PySolid, operation: BooleanOperation) -> PyResult<PySolid> {
    modeling::solids::combine_views(
        first
            .inner
            .view()
            .map_err(|error| PyValueError::new_err(error.to_string()))?,
        second
            .inner
            .view()
            .map_err(|error| PyValueError::new_err(error.to_string()))?,
        operation,
    )
    .map_err(|error| PyValueError::new_err(error.to_string()))
    .and_then(py_solid)
}
