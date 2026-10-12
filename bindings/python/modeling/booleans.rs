use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate::binding_common::explore::SharedModel;
use crate::binding_common::modeling::{face_boolean, solid_boolean};
use crate::builders::boolean::BooleanOperation;

use super::super::topology::{PyFace, PyModel, PySolid};
use super::common::py_solid;

/// Registers the solid and face Boolean operations.
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

/// One tool, or a sequence of them.
#[derive(FromPyObject)]
pub(crate) enum Tools<T> {
    One(T),
    Many(Vec<T>),
}

impl<T> Tools<T> {
    fn into_vec(self) -> Vec<T> {
        match self {
            Self::One(tool) => vec![tool],
            Self::Many(tools) => tools,
        }
    }
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
pub(crate) fn fuse_faces(target: &PyFace, tools: Tools<PyFace>) -> PyResult<PyFaceBooleanResult> {
    combine_faces(target, tools, BooleanOperation::Union)
}

#[pyfunction]
pub(crate) fn cut_faces(target: &PyFace, tools: Tools<PyFace>) -> PyResult<PyFaceBooleanResult> {
    combine_faces(target, tools, BooleanOperation::Difference)
}

#[pyfunction]
pub(crate) fn intersect_faces(
    target: &PyFace,
    tools: Tools<PyFace>,
) -> PyResult<PyFaceBooleanResult> {
    combine_faces(target, tools, BooleanOperation::Intersection)
}

fn combine_faces(
    target: &PyFace,
    tools: Tools<PyFace>,
    operation: BooleanOperation,
) -> PyResult<PyFaceBooleanResult> {
    let tools = tools
        .into_vec()
        .into_iter()
        .map(|tool| tool.inner)
        .collect::<Vec<_>>();
    let result = face_boolean(&target.inner, &tools, operation)
        .map_err(|error| PyValueError::new_err(error.to_string()))?;
    let (model, faces) = result.into_model();
    Ok(PyFaceBooleanResult {
        model: SharedModel::from_model(model),
        faces,
    })
}

#[pyfunction]
pub(crate) fn fuse(target: &PySolid, tools: Tools<PySolid>) -> PyResult<PySolid> {
    combine(target, tools, BooleanOperation::Union)
}

#[pyfunction]
pub(crate) fn cut(target: &PySolid, tools: Tools<PySolid>) -> PyResult<PySolid> {
    combine(target, tools, BooleanOperation::Difference)
}

#[pyfunction]
pub(crate) fn intersect(target: &PySolid, tools: Tools<PySolid>) -> PyResult<PySolid> {
    combine(target, tools, BooleanOperation::Intersection)
}

fn combine(
    target: &PySolid,
    tools: Tools<PySolid>,
    operation: BooleanOperation,
) -> PyResult<PySolid> {
    let tools = tools
        .into_vec()
        .into_iter()
        .map(|tool| tool.inner)
        .collect::<Vec<_>>();
    solid_boolean(&target.inner, &tools, operation)
        .map_err(|error| PyValueError::new_err(error.to_string()))
        .and_then(py_solid)
}
