use pyo3::basic::CompareOp;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use super::common::{Map, py_err};
use super::{PyEdge, PyFace, PyProfile, PySheet, PySolid, PyVertex};

/// Python wrapper for a complete immutable standard-payload GMap.
#[pyclass(name = "GMap", module = "ngk")]
#[derive(Clone)]
pub struct PyGMap {
    pub(crate) inner: Map,
}

impl PyGMap {
    pub(crate) fn from_inner(inner: Map) -> Self {
        Self { inner }
    }
}

#[pymethods]
impl PyGMap {
    #[staticmethod]
    fn deserialize(serialized: &str) -> PyResult<Self> {
        Ok(Self::from_inner(
            Map::deserialize(serialized).map_err(py_err)?,
        ))
    }

    fn serialize(&self) -> PyResult<String> {
        self.inner.serialize().map_err(py_err)
    }

    #[getter]
    fn dimension(&self) -> usize {
        self.inner.dimension()
    }

    #[getter]
    fn involution_count(&self) -> usize {
        self.inner.involution_count()
    }

    #[getter]
    fn dart_count(&self) -> usize {
        self.inner.dart_count()
    }

    fn darts(&self) -> Vec<usize> {
        self.inner.darts()
    }

    fn alpha(&self, dimension: usize, dart: usize) -> PyResult<usize> {
        self.inner.alpha(dimension, dart).map_err(py_err)
    }

    fn is_free(&self, dimension: usize, dart: usize) -> PyResult<bool> {
        self.inner.is_free(dimension, dart).map_err(py_err)
    }

    fn orbit(&self, dart: usize, involutions: Vec<usize>) -> PyResult<Vec<usize>> {
        self.inner.orbit(dart, involutions).map_err(py_err)
    }

    fn cells(&self, dimension: usize) -> PyResult<Vec<usize>> {
        self.inner.cells(dimension).map_err(py_err)
    }

    fn cell_darts(&self, dart: usize, dimension: usize) -> PyResult<Vec<usize>> {
        self.inner.cell_darts(dart, dimension).map_err(py_err)
    }

    fn cell_representative(&self, dart: usize, dimension: usize) -> PyResult<usize> {
        self.inner
            .cell_representative(dart, dimension)
            .map_err(py_err)
    }

    fn incident_cells(
        &self,
        dart: usize,
        container_dimension: usize,
        target_dimension: usize,
    ) -> PyResult<Vec<usize>> {
        self.inner
            .incident_cells(dart, container_dimension, target_dimension)
            .map_err(py_err)
    }

    fn adjacent_cells(&self, dart: usize, dimension: usize) -> PyResult<Vec<usize>> {
        self.inner.adjacent_cells(dart, dimension).map_err(py_err)
    }

    fn vertices(&self) -> Vec<PyVertex> {
        self.inner
            .vertices()
            .into_iter()
            .map(PyVertex::from_inner)
            .collect()
    }

    fn edges(&self) -> Vec<PyEdge> {
        self.inner
            .edges()
            .into_iter()
            .map(PyEdge::from_inner)
            .collect()
    }

    fn profiles(&self) -> Vec<PyProfile> {
        self.inner
            .profiles()
            .into_iter()
            .map(PyProfile::from_inner)
            .collect()
    }

    fn faces(&self) -> Vec<PyFace> {
        self.inner
            .faces()
            .into_iter()
            .map(PyFace::from_inner)
            .collect()
    }

    fn sheets(&self) -> Vec<PySheet> {
        self.inner
            .sheets()
            .into_iter()
            .map(PySheet::from_inner)
            .collect()
    }

    fn solids(&self) -> Vec<PySolid> {
        self.inner
            .solids()
            .into_iter()
            .map(PySolid::from_inner)
            .collect()
    }

    fn vertex(&self, dart: usize) -> PyResult<Option<PyVertex>> {
        Ok(self
            .inner
            .vertex_at(dart)
            .map_err(py_err)?
            .map(PyVertex::from_inner))
    }

    fn edge(&self, dart: usize) -> PyResult<Option<PyEdge>> {
        Ok(self
            .inner
            .edge_at(dart)
            .map_err(py_err)?
            .map(PyEdge::from_inner))
    }

    fn profile(&self, dart: usize) -> PyResult<Option<PyProfile>> {
        Ok(self
            .inner
            .profile_at(dart)
            .map_err(py_err)?
            .map(PyProfile::from_inner))
    }

    fn face(&self, dart: usize) -> PyResult<Option<PyFace>> {
        Ok(self
            .inner
            .face_at(dart)
            .map_err(py_err)?
            .map(PyFace::from_inner))
    }

    fn sheet(&self, dart: usize) -> PyResult<Option<PySheet>> {
        Ok(self
            .inner
            .sheet_at(dart)
            .map_err(py_err)?
            .map(PySheet::from_inner))
    }

    fn solid(&self, dart: usize) -> PyResult<Option<PySolid>> {
        Ok(self
            .inner
            .solid_at(dart)
            .map_err(py_err)?
            .map(PySolid::from_inner))
    }

    fn __richcmp__(&self, other: PyRef<'_, PyGMap>, op: CompareOp) -> PyResult<bool> {
        match op {
            CompareOp::Eq => Ok(self.inner.ptr_eq(&other.inner)),
            CompareOp::Ne => Ok(!self.inner.ptr_eq(&other.inner)),
            _ => Err(PyValueError::new_err(
                "GMap ordering is not defined; use == or !=",
            )),
        }
    }

    fn __hash__(&self) -> isize {
        self.inner.identity() as isize
    }

    fn __repr__(&self) -> String {
        format!("GMap(darts={})", self.inner.dart_count())
    }
}
