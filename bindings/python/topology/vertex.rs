use pyo3::basic::CompareOp;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use super::common::{entity_methods, hash_identity, py_err};
use super::gmap::PyGMap;

use crate::binding_common::explore::SharedVertex;
use crate::topology::StandardPayload;

use super::super::geometry::{PyPoint3, point};
use super::edge::PyEdge;
use super::face::PyFace;
use super::sheet::PySheet;

#[pyclass(name = "Vertex", module = "ngk")]
#[derive(Clone)]
pub struct PyVertex {
    inner: SharedVertex<StandardPayload>,
}

entity_methods!(PyVertex, SharedVertex<StandardPayload>, "vertex", {
    #[getter]
    fn point(&self) -> PyResult<Option<PyPoint3>> {
        Ok(self.inner.point().map_err(py_err)?.map(point))
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

    fn __repr__(&self) -> PyResult<String> {
        let value = match self.point()? {
            Some(point) => format!(
                "Vertex(key={:?}, point=({}, {}, {}))",
                self.inner.key(),
                point.point.x,
                point.point.y,
                point.point.z
            ),
            None => format!("Vertex(key={:?}, point=None)", self.inner.key()),
        };
        Ok(value)
    }
});
