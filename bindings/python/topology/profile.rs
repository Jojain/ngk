use pyo3::basic::CompareOp;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use super::common::{entity_methods, hash_identity, py_err};
use super::gmap::PyGMap;

use crate::binding_common::explore::{SharedLoop, SharedProfile};
use crate::topology::StandardPayload;

use super::edge::PyEdge;
use super::vertex::PyVertex;

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
