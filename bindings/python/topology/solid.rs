use pyo3::basic::CompareOp;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use super::common::{entity_methods, hash_identity, py_err};
use super::gmap::PyGMap;

use crate::binding_common::explore::SharedSolid;
use crate::topology::StandardPayload;

use super::edge::PyEdge;
use super::face::PyFace;
use super::sheet::PyShell;
use super::vertex::PyVertex;

#[pyclass(name = "Solid", module = "ngk")]
#[derive(Clone)]
pub struct PySolid {
    pub(crate) inner: SharedSolid<StandardPayload>,
}

entity_methods!(PySolid, SharedSolid<StandardPayload>, "solid", {
    #[getter]
    fn outer_shell(&self) -> PyResult<PyShell> {
        Ok(PyShell::from_inner(
            self.inner.outer_shell().map_err(py_err)?,
        ))
    }

    fn inner_shells(&self) -> PyResult<Vec<PyShell>> {
        Ok(self
            .inner
            .inner_shells()
            .map_err(py_err)?
            .into_iter()
            .map(PyShell::from_inner)
            .collect())
    }

    fn shells(&self) -> PyResult<Vec<PyShell>> {
        Ok(self
            .inner
            .shells()
            .map_err(py_err)?
            .into_iter()
            .map(PyShell::from_inner)
            .collect())
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

    #[getter]
    fn face_count(&self) -> PyResult<usize> {
        Ok(self.inner.faces().map_err(py_err)?.len())
    }

    #[getter]
    fn edge_count(&self) -> PyResult<usize> {
        Ok(self.inner.edges().map_err(py_err)?.len())
    }

    #[getter]
    fn vertex_count(&self) -> PyResult<usize> {
        Ok(self.inner.vertices().map_err(py_err)?.len())
    }

    fn __repr__(&self) -> String {
        format!("Solid(key={:?})", self.inner.key())
    }
});
