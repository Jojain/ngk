use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate::binding_common::modeling::extrusion_direction;
use crate::modeling;

use super::super::geometry::{PyFrame, PyVector3};
use super::super::topology::{PyFace, PySheet, PySolid};
use super::common::py_solid;

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_function(wrap_pyfunction!(block, module)?)?;
    module.add_function(wrap_pyfunction!(cylinder, module)?)?;
    module.add_function(wrap_pyfunction!(sphere, module)?)?;
    module.add_function(wrap_pyfunction!(torus, module)?)?;
    module.add_function(wrap_pyfunction!(extruded, module)?)?;
    module.add_function(wrap_pyfunction!(from_sheets, module)?)?;
    Ok(())
}

/// Bounds a solid with a closed outer sheet, less the regions the voids bound.
#[pyfunction]
#[pyo3(signature = (outer, voids=Vec::new()))]
pub(crate) fn from_sheets(
    outer: PyRef<'_, PySheet>,
    voids: Vec<PyRef<'_, PySheet>>,
) -> PyResult<PySolid> {
    let isolated = |sheet: &PyRef<'_, PySheet>| {
        sheet
            .inner
            .isolated_shape()
            .map_err(|error| PyValueError::new_err(error.to_string()))
    };
    let outer = isolated(&outer)?;
    let voids = voids.iter().map(isolated).collect::<PyResult<Vec<_>>>()?;
    let references = voids.iter().collect::<Vec<_>>();
    modeling::solids::from_sheets(&outer, &references)
        .map_err(|error| PyValueError::new_err(error.to_string()))
        .and_then(py_solid)
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
    let direction = extrusion_direction(direction.vector)
        .map_err(|error| PyValueError::new_err(error.to_string()))?;
    modeling::solids::extruded(shape, direction, distance)
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
pub(crate) fn cylinder(radius: f64, height: f64, frame: Option<PyFrame>) -> PyResult<PySolid> {
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
