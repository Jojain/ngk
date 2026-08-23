use pyo3::basic::CompareOp;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use super::common::{entity_methods, hash_identity, py_err};
use super::gmap::PyGMap;

use crate::binding_common::explore::{SharedSheet, SharedShell};
use crate::topology::StandardPayload;

use super::edge::PyEdge;
use super::face::PyFace;
use super::vertex::PyVertex;

#[pyclass(name = "Shell", module = "ngk")]
#[derive(Clone)]
pub struct PyShell {
    inner: SharedShell<StandardPayload>,
}

entity_methods!(PyShell, SharedShell<StandardPayload>, "shell", {
    fn faces(&self) -> PyResult<Vec<PyFace>> {
        Ok(self
            .inner
            .faces()
            .map_err(py_err)?
            .into_iter()
            .map(PyFace::from_inner)
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
            "Shell(key={:?}, dart_id={})",
            self.inner.key(),
            self.inner.dart_id()
        )
    }
});

#[pyclass(name = "Sheet", module = "ngk")]
#[derive(Clone)]
pub struct PySheet {
    inner: SharedSheet<StandardPayload>,
}

entity_methods!(PySheet, SharedSheet<StandardPayload>, "sheet", {
    #[getter]
    fn is_closed(&self) -> PyResult<bool> {
        self.inner.is_closed().map_err(py_err)
    }

    fn darts(&self) -> PyResult<Vec<usize>> {
        self.inner.darts().map_err(py_err)
    }

    fn faces(&self) -> PyResult<Vec<PyFace>> {
        Ok(self
            .inner
            .faces()
            .map_err(py_err)?
            .into_iter()
            .map(PyFace::from_inner)
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
});
