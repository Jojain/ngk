use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyModule;
use radians::Rad64;

use crate::binding_common::explore::SharedModel;
use crate::builders::edges::EdgeSplit;
use crate::geometry::Fraction;
use crate::geometry::Point3;
use crate::modeling;

use super::super::geometry::{PyAxis3, PyPlane};
use super::super::topology::{PyEdge, PyFace, PyModel, PyVertex};
use super::common::py_edge;

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_function(wrap_pyfunction!(line, module)?)?;
    module.add_function(wrap_pyfunction!(arc, module)?)?;
    module.add_function(wrap_pyfunction!(helix, module)?)?;
    module.add_function(wrap_pyfunction!(circle, module)?)?;
    module.add_class::<PyEdgeSplitResult>()?;
    module.add_function(wrap_pyfunction!(split, module)?)?;
    Ok(())
}

#[pyclass(name = "EdgeSplitResult", module = "ngk")]
pub struct PyEdgeSplitResult {
    model: SharedModel,
    split: EdgeSplit,
    face: Option<crate::topology::shape_keys::FaceKey>,
}

impl PyEdgeSplitResult {
    pub(crate) fn from_result(result: modeling::edges::EdgeSplitResult) -> Self {
        let (model, split, face) = result.into_model();
        Self {
            model: SharedModel::from_model(model),
            split,
            face,
        }
    }
}

#[pymethods]
impl PyEdgeSplitResult {
    #[getter]
    fn model(&self) -> PyModel {
        PyModel::from_inner(self.model.clone())
    }

    #[getter]
    fn separated(&self) -> bool {
        matches!(self.split, EdgeSplit::Separated { .. })
    }

    fn edges(&self) -> PyResult<Vec<PyEdge>> {
        self.split
            .edges()
            .map(|key| {
                self.model
                    .edge_by_key(key)
                    .map(PyEdge::from_inner)
                    .ok_or_else(|| PyValueError::new_err(format!("missing edge {key:?}")))
            })
            .collect()
    }

    fn vertex(&self) -> PyResult<PyVertex> {
        let key = self.split.vertex();
        self.model
            .vertex_by_key(key)
            .map(PyVertex::from_inner)
            .ok_or_else(|| PyValueError::new_err(format!("missing vertex {key:?}")))
    }

    fn face(&self) -> PyResult<Option<PyFace>> {
        self.face
            .map(|key| {
                self.model
                    .face_by_key(key)
                    .map(PyFace::from_inner)
                    .ok_or_else(|| PyValueError::new_err(format!("missing face {key:?}")))
            })
            .transpose()
    }
}

#[pyfunction]
pub(crate) fn split(edge: &PyEdge, fraction: f64) -> PyResult<PyEdgeSplitResult> {
    let shape = edge
        .inner
        .isolated_shape()
        .map_err(|error| PyValueError::new_err(error.to_string()))?;
    let result = modeling::edges::split(shape, Fraction::new(fraction))
        .map_err(|error| PyValueError::new_err(error.to_string()))?;
    Ok(PyEdgeSplitResult::from_result(result))
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
    modeling::edges::arc(
        plane.plane,
        radius,
        Rad64::new(start_angle),
        Rad64::new(end_angle),
    )
    .map_err(|error| PyValueError::new_err(error.to_string()))
    .and_then(py_edge)
}

#[pyfunction]
pub(crate) fn helix(
    axis: PyAxis3,
    radius: f64,
    pitch: f64,
    start_angle: f64,
    end_angle: f64,
) -> PyResult<PyEdge> {
    modeling::edges::helix(
        axis.axis,
        radius,
        pitch,
        Rad64::new(start_angle),
        Rad64::new(end_angle),
    )
    .map_err(|error| PyValueError::new_err(error.to_string()))
    .and_then(py_edge)
}

#[pyfunction]
pub(crate) fn circle(plane: PyPlane, radius: f64) -> PyResult<PyEdge> {
    modeling::edges::circle(plane.plane, radius)
        .map_err(|error| PyValueError::new_err(error.to_string()))
        .and_then(py_edge)
}
