use pyo3::basic::CompareOp;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use super::common::{entity_methods, hash_identity, py_err};
use super::model::PyModel;
use crate::binding_common::explore::{SharedLoop, SharedProfile};
use crate::topology::StandardPayload;

use super::super::geometry::PyFrame;
use super::super::measurement::{self, PyBoundingBox, PyLinearProperties};
use super::edge::PyEdge;
use super::vertex::PyVertex;
use crate::binding_common::measurement as binding_measurement;
use crate::binding_common::measurement::BoxRequest;

#[pyclass(name = "Loop", module = "ngk")]
#[derive(Clone)]
pub struct PyLoop {
    inner: SharedLoop<StandardPayload>,
}

entity_methods!(PyLoop, SharedLoop<StandardPayload>, "loop", {
    fn darts(&self) -> PyResult<Vec<usize>> {
        self.inner.darts().map_err(py_err)
    }

    fn edges(&self) -> PyResult<Vec<PyEdge>> {
        Ok(self
            .inner
            .edges()
            .map_err(py_err)?
            .into_iter()
            .map(PyEdge::from_inner)
            .collect())
    }

    fn vertices(&self) -> PyResult<Vec<PyVertex>> {
        Ok(self
            .inner
            .vertices()
            .map_err(py_err)?
            .into_iter()
            .map(PyVertex::from_inner)
            .collect())
    }

    fn reversed(&self) -> PyResult<Self> {
        Ok(Self::from_inner(self.inner.reversed().map_err(py_err)?))
    }
});

#[pyclass(name = "Profile", module = "ngk")]
#[derive(Clone)]
pub struct PyProfile {
    pub(crate) inner: SharedProfile<StandardPayload>,
}

entity_methods!(PyProfile, SharedProfile<StandardPayload>, "profile", {
    /// This entity, copied into a model holding nothing else.
    fn isolated(&self) -> PyResult<Self> {
        Ok(Self::from_inner(self.inner.isolated().map_err(py_err)?))
    }

    #[getter]
    fn is_closed(&self) -> PyResult<bool> {
        self.inner.is_closed().map_err(py_err)
    }

    #[getter]
    fn start(&self) -> PyResult<PyVertex> {
        Ok(PyVertex::from_inner(self.inner.start().map_err(py_err)?))
    }

    #[getter]
    fn end(&self) -> PyResult<PyVertex> {
        Ok(PyVertex::from_inner(self.inner.end().map_err(py_err)?))
    }

    #[getter]
    fn length(&self) -> PyResult<f64> {
        binding_measurement::profile_length(&self.inner).map_err(PyValueError::new_err)
    }

    #[getter]
    fn linear_properties(&self) -> PyResult<PyLinearProperties> {
        measurement::linear(binding_measurement::profile(&self.inner))
    }

    #[pyo3(signature = (frame=None))]
    fn bounding_box(&self, frame: Option<PyFrame>) -> PyResult<PyBoundingBox> {
        let frame = frame.map(|frame| frame.frame);
        measurement::bounding_box(binding_measurement::profile_box(
            &self.inner,
            BoxRequest::Aligned(frame.as_ref()),
        ))
    }

    fn oriented_bounding_box(&self) -> PyResult<PyBoundingBox> {
        measurement::bounding_box(binding_measurement::profile_box(
            &self.inner,
            BoxRequest::Oriented,
        ))
    }

    fn darts(&self) -> PyResult<Vec<usize>> {
        self.inner.darts().map_err(py_err)
    }

    fn edges(&self) -> PyResult<Vec<PyEdge>> {
        Ok(self
            .inner
            .edges()
            .map_err(py_err)?
            .into_iter()
            .map(PyEdge::from_inner)
            .collect())
    }

    fn vertices(&self) -> PyResult<Vec<PyVertex>> {
        Ok(self
            .inner
            .vertices()
            .map_err(py_err)?
            .into_iter()
            .map(PyVertex::from_inner)
            .collect())
    }

    fn reversed(&self) -> PyResult<Self> {
        Ok(Self::from_inner(self.inner.reversed().map_err(py_err)?))
    }
});
