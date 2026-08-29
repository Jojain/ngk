use pyo3::basic::CompareOp;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use super::common::{entity_methods, hash_identity, py_err};
use super::gmap::PyGMap;

use crate::binding_common::explore::SharedFace;
use crate::topology::StandardPayload;

use super::super::geometry::surface_to_py;
use super::edge::PyEdge;
use super::profile::PyLoop;
use super::vertex::PyVertex;

#[pyclass(name = "Face", module = "ngk")]
#[derive(Clone)]
pub struct PyFace {
    pub(crate) inner: SharedFace<StandardPayload>,
}

entity_methods!(PyFace, SharedFace<StandardPayload>, "face", {
    #[getter]
    fn surface(&self, py: Python<'_>) -> PyResult<PyObject> {
        surface_to_py(py, self.inner.surface().map_err(py_err)?)
    }

    #[getter]
    fn outer_loop(&self) -> PyResult<PyLoop> {
        Ok(PyLoop::from_inner(self.inner.outer_loop().map_err(py_err)?))
    }

    fn inner_loops(&self) -> PyResult<Vec<PyLoop>> {
        Ok(self
            .inner
            .inner_loops()
            .map_err(py_err)?
            .into_iter()
            .map(PyLoop::from_inner)
            .collect())
    }

    fn loops(&self) -> PyResult<Vec<PyLoop>> {
        Ok(self
            .inner
            .loops()
            .map_err(py_err)?
            .into_iter()
            .map(PyLoop::from_inner)
            .collect())
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

    fn __repr__(&self) -> String {
        format!(
            "Face(key={:?}, dart_id={})",
            self.inner.key(),
            self.inner.dart_id()
        )
    }
});
