use std::sync::Arc;

use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate::geometry::{Plane, Point3};
use crate::modeling::{edges, faces, profiles, solids};

use super::super::{PyEdge, PyFace, PyProfile, PySolid};

pub(super) fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(block, m)?)?;
    m.add_function(wrap_pyfunction!(line, m)?)?;
    m.add_function(wrap_pyfunction!(rectangle_profile, m)?)?;
    m.add_function(wrap_pyfunction!(rectangle_face, m)?)?;
    Ok(())
}

#[pyfunction]
fn block(x: f64, y: f64, z: f64) -> PyResult<PySolid> {
    let shape = solids::block(x, y, z).map_err(|err| PyValueError::new_err(err.to_string()))?;
    let (map, key) = shape.into_map();
    Ok(PySolid::new(Arc::new(map), key))
}

#[pyfunction]
fn line(start: (f64, f64, f64), end: (f64, f64, f64)) -> PyResult<PyEdge> {
    let start = Point3::new(start.0, start.1, start.2);
    let end = Point3::new(end.0, end.1, end.2);
    let shape = edges::line(start, end).map_err(|err| PyValueError::new_err(err.to_string()))?;
    let (map, key) = shape.into_map();
    Ok(PyEdge::from_key(Arc::new(map), key))
}

#[pyfunction]
fn rectangle_profile(x_size: f64, y_size: f64) -> PyResult<PyProfile> {
    let shape = profiles::rectangle(Plane::xy(), x_size, y_size)
        .map_err(|err| PyValueError::new_err(err.to_string()))?;
    let (map, key) = shape.into_map();
    Ok(PyProfile::new(Arc::new(map), key))
}

#[pyfunction]
fn rectangle_face(x_size: f64, y_size: f64) -> PyResult<PyFace> {
    let shape = faces::rectangle(Plane::xy(), x_size, y_size)
        .map_err(|err| PyValueError::new_err(err.to_string()))?;
    let (map, key) = shape.into_map();
    Ok(PyFace::new(Arc::new(map), key))
}
