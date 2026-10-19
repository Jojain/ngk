use pyo3::basic::CompareOp;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use super::common::{entity_methods, hash_identity, py_err};
use super::model::PyModel;
use crate::binding_common::explore::SharedFace;
use crate::topology::StandardPayload;

use super::super::geometry::PyFrame;
use super::super::geometry::{PyVector3, surface_to_py, unit_vector};
use super::super::measurement::{self, PyBoundingBox, PySurfaceProperties};
use super::edge::PyEdge;
use super::profile::PyLoop;
use super::vertex::PyVertex;
use crate::binding_common::measurement as binding_measurement;
use crate::binding_common::measurement::BoxRequest;

#[pyclass(name = "Face", module = "ngk")]
#[derive(Clone)]
pub struct PyFace {
    pub(crate) inner: SharedFace<StandardPayload>,
}

entity_methods!(PyFace, SharedFace<StandardPayload>, "face", {
    /// This entity, copied into a model holding nothing else.
    fn isolated(&self) -> PyResult<Self> {
        Ok(Self::from_inner(self.inner.isolated().map_err(py_err)?))
    }

    #[getter]
    fn surface(&self, py: Python<'_>) -> PyResult<PyObject> {
        surface_to_py(py, self.inner.surface().map_err(py_err)?)
    }

    /// The face's own normal at `(u, v)` on its surface: outward for a face of
    /// a solid, and opposite to `surface.normal_at(u, v)` where the face is
    /// reversed against its surface. `(u, v)` is not tested against the
    /// face's loops.
    fn normal_at(&self, u: f64, v: f64) -> PyResult<PyVector3> {
        Ok(unit_vector(
            self.inner.view().map_err(py_err)?.normal_at(u, v),
        ))
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

    #[getter]
    fn area(&self) -> PyResult<f64> {
        binding_measurement::face(&self.inner)
            .map(|properties| properties.area)
            .map_err(PyValueError::new_err)
    }

    #[getter]
    fn surface_properties(&self) -> PyResult<PySurfaceProperties> {
        measurement::surface(binding_measurement::face(&self.inner))
    }

    #[pyo3(signature = (frame=None))]
    fn bounding_box(&self, frame: Option<PyFrame>) -> PyResult<PyBoundingBox> {
        let frame = frame.map(|frame| frame.frame);
        measurement::bounding_box(binding_measurement::face_box(
            &self.inner,
            BoxRequest::Aligned(frame.as_ref()),
        ))
    }

    fn oriented_bounding_box(&self) -> PyResult<PyBoundingBox> {
        measurement::bounding_box(binding_measurement::face_box(
            &self.inner,
            BoxRequest::Oriented,
        ))
    }

    fn __repr__(&self) -> String {
        format!(
            "Face(key={:?}, dart_id={:?})",
            self.inner.key(),
            self.inner.dart_id()
        )
    }
});
