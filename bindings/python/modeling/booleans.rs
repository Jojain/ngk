use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate::binding_common::explore::SharedModel;
use crate::builders::boolean::BooleanOperation;
use crate::modeling;

use super::super::topology::{PyFace, PyModel, PySolid};
use super::common::py_solid;

/// Registers the current solid Boolean operations.
///
/// They live in the Boolean namespace rather than the solids namespace so
/// later dimension-generic operations keep the same Python import path.
pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_function(wrap_pyfunction!(fuse, module)?)?;
    module.add_function(wrap_pyfunction!(cut, module)?)?;
    module.add_function(wrap_pyfunction!(intersect, module)?)?;
    module.add_class::<PyFaceBooleanResult>()?;
    module.add_function(wrap_pyfunction!(fuse_faces, module)?)?;
    module.add_function(wrap_pyfunction!(cut_faces, module)?)?;
    module.add_function(wrap_pyfunction!(intersect_faces, module)?)?;
    Ok(())
}

/// A face Boolean result whose handles share one model.
#[pyclass(name = "FaceBooleanResult", module = "ngk")]
pub struct PyFaceBooleanResult {
    model: SharedModel,
    faces: Vec<crate::topology::shape_keys::FaceKey>,
}

#[pymethods]
impl PyFaceBooleanResult {
    #[getter]
    fn model(&self) -> PyModel {
        PyModel::from_inner(self.model.clone())
    }

    fn faces(&self) -> PyResult<Vec<PyFace>> {
        self.faces
            .iter()
            .map(|key| {
                self.model
                    .face_by_key(*key)
                    .map(PyFace::from_inner)
                    .ok_or_else(|| PyValueError::new_err(format!("missing face {key:?}")))
            })
            .collect()
    }
}

#[pyfunction]
pub(crate) fn fuse_faces(first: &PyFace, second: &PyFace) -> PyResult<PyFaceBooleanResult> {
    combine_faces(first, second, BooleanOperation::Union)
}

#[pyfunction]
pub(crate) fn cut_faces(first: &PyFace, second: &PyFace) -> PyResult<PyFaceBooleanResult> {
    combine_faces(first, second, BooleanOperation::Difference)
}

#[pyfunction]
pub(crate) fn intersect_faces(first: &PyFace, second: &PyFace) -> PyResult<PyFaceBooleanResult> {
    combine_faces(first, second, BooleanOperation::Intersection)
}

fn combine_faces(
    first: &PyFace,
    second: &PyFace,
    operation: BooleanOperation,
) -> PyResult<PyFaceBooleanResult> {
    let result = modeling::faces::combine_views(
        first
            .inner
            .view()
            .map_err(|error| PyValueError::new_err(error.to_string()))?,
        second
            .inner
            .view()
            .map_err(|error| PyValueError::new_err(error.to_string()))?,
        operation,
    )
    .map_err(|error| PyValueError::new_err(error.to_string()))?;
    let (model, faces) = result.into_model();
    Ok(PyFaceBooleanResult {
        model: SharedModel::from_model(model),
        faces,
    })
}

#[pyfunction]
pub(crate) fn fuse(first: &PySolid, second: &PySolid) -> PyResult<PySolid> {
    combine(first, second, BooleanOperation::Union)
}

#[pyfunction]
pub(crate) fn cut(target: &PySolid, tool: &PySolid) -> PyResult<PySolid> {
    combine(target, tool, BooleanOperation::Difference)
}

#[pyfunction]
pub(crate) fn intersect(first: &PySolid, second: &PySolid) -> PyResult<PySolid> {
    combine(first, second, BooleanOperation::Intersection)
}

fn combine(first: &PySolid, second: &PySolid, operation: BooleanOperation) -> PyResult<PySolid> {
    modeling::solids::combine_views(
        first
            .inner
            .view()
            .map_err(|error| PyValueError::new_err(error.to_string()))?,
        second
            .inner
            .view()
            .map_err(|error| PyValueError::new_err(error.to_string()))?,
        operation,
    )
    .map_err(|error| PyValueError::new_err(error.to_string()))
    .and_then(py_solid)
}
