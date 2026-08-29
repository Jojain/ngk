use pyo3::basic::CompareOp;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use super::common::{entity_methods, hash_identity, py_err};
use super::gmap::PyGMap;

use crate::binding_common::explore::SharedEdge;
use crate::topology::StandardPayload;

use super::super::geometry::curve_to_py;
use super::face::PyFace;
use super::sheet::PySheet;
use super::vertex::PyVertex;

#[pyclass(name = "Edge", module = "ngk")]
#[derive(Clone)]
pub struct PyEdge {
    pub(crate) inner: SharedEdge<StandardPayload>,
}

entity_methods!(PyEdge, SharedEdge<StandardPayload>, "edge", {
    #[getter]
    fn start(&self) -> PyResult<PyVertex> {
        Ok(PyVertex::from_inner(self.inner.start().map_err(py_err)?))
    }

    #[getter]
    fn end(&self) -> PyResult<PyVertex> {
        Ok(PyVertex::from_inner(self.inner.end().map_err(py_err)?))
    }

    #[getter]
    fn length(&self) -> PyResult<Option<f64>> {
        self.inner.length().map_err(py_err)
    }

    #[getter]
    fn curve(&self, py: Python<'_>) -> PyResult<Option<PyObject>> {
        self.inner
            .curve()
            .map_err(py_err)?
            .map(|curve| curve_to_py(py, curve))
            .transpose()
    }

    fn darts(&self) -> PyResult<Vec<usize>> {
        self.inner.darts().map_err(py_err)
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

    fn faces(&self) -> PyResult<Vec<PyFace>> {
        Ok(self
            .inner
            .faces()
            .map_err(py_err)?
            .into_iter()
            .map(PyFace::from_inner)
            .collect())
    }

    fn sheets(&self) -> PyResult<Vec<PySheet>> {
        Ok(self
            .inner
            .sheets()
            .map_err(py_err)?
            .into_iter()
            .map(PySheet::from_inner)
            .collect())
    }

    fn reversed(&self) -> PyResult<Self> {
        Ok(Self::from_inner(self.inner.reversed().map_err(py_err)?))
    }

    fn __repr__(&self) -> String {
        format!(
            "Edge(key={:?}, dart_id={})",
            self.inner.key(),
            self.inner.dart_id()
        )
    }
});
