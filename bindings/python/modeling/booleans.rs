use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate::binding_common::explore::SharedModel;
use crate::binding_common::modeling::{
    BindingBooleanError, face_boolean, solid_boolean, split_alone, split_in_place,
};
use crate::builders::boolean::{BooleanOperation, Split};
use crate::model::Model;
use crate::topology::shape_keys::{EdgeKey, FaceKey, ProfileKey, SheetKey, SolidKey};

use super::super::topology::{PyEdge, PyFace, PyModel, PyProfile, PySheet, PySolid};
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
    module.add_class::<PySplitResult>()?;
    module.add_function(wrap_pyfunction!(split, module)?)?;
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

/// What a split can cut.
#[derive(FromPyObject)]
pub(crate) enum SplitTarget {
    Solid(PySolid),
    Sheet(PySheet),
    Face(PyFace),
    Profile(PyProfile),
    Edge(PyEdge),
}

/// One part of a split, of the target's own kind.
#[derive(Debug, Clone, Copy)]
enum SplitPart {
    Solid(SolidKey),
    Sheet(SheetKey),
    Face(FaceKey),
    Profile(ProfileKey),
    Edge(EdgeKey),
}

/// The two parts a split produced, sharing one model.
#[pyclass(name = "SplitResult", module = "ngk")]
pub struct PySplitResult {
    model: SharedModel,
    front: SplitPart,
    back: SplitPart,
}

impl PySplitResult {
    fn new<K>(model: Model, parts: Split<K>, part: fn(K) -> SplitPart) -> Self {
        Self {
            model: SharedModel::from_model(model),
            front: part(parts.front),
            back: part(parts.back),
        }
    }

    fn part(&self, py: Python<'_>, part: SplitPart) -> PyResult<PyObject> {
        let missing = || PyValueError::new_err(format!("missing part {part:?}"));
        Ok(match part {
            SplitPart::Solid(key) => {
                let solid = self.model.solid_by_key(key).ok_or_else(missing)?;
                PySolid::from_inner(solid).into_py(py)
            }
            SplitPart::Sheet(key) => {
                let sheet = self.model.sheet_by_key(key).ok_or_else(missing)?;
                PySheet::from_inner(sheet).into_py(py)
            }
            SplitPart::Face(key) => {
                let face = self.model.face_by_key(key).ok_or_else(missing)?;
                PyFace::from_inner(face).into_py(py)
            }
            SplitPart::Profile(key) => {
                let profile = self.model.profile_by_key(key).ok_or_else(missing)?;
                PyProfile::from_inner(profile).into_py(py)
            }
            SplitPart::Edge(key) => {
                let edge = self.model.edge_by_key(key).ok_or_else(missing)?;
                PyEdge::from_inner(edge).into_py(py)
            }
        })
    }
}

#[pymethods]
impl PySplitResult {
    #[getter]
    fn model(&self) -> PyModel {
        PyModel::from_inner(self.model.clone())
    }

    /// The part on the side the sheet's normals point to.
    #[getter]
    fn front(&self, py: Python<'_>) -> PyResult<PyObject> {
        self.part(py, self.front)
    }

    /// The part behind the sheet.
    #[getter]
    fn back(&self, py: Python<'_>) -> PyResult<PyObject> {
        self.part(py, self.back)
    }
}

/// Splits a shape along a sheet into the parts in front of and behind it,
/// each of the target's own kind.
#[pyfunction]
pub(crate) fn split(target: SplitTarget, tool: &PySheet) -> PyResult<PySplitResult> {
    let raise = |error: BindingBooleanError| PyValueError::new_err(error.to_string());
    let tool = &tool.inner;
    Ok(match target {
        SplitTarget::Solid(target) => {
            let target = &target.inner;
            target.view().map_err(|error| raise(error.into()))?;
            let (model, parts) =
                split_in_place(&target.model(), target.key(), tool).map_err(raise)?;
            PySplitResult::new(model, parts, SplitPart::Solid)
        }
        SplitTarget::Sheet(target) => {
            let target = &target.inner;
            target.view().map_err(|error| raise(error.into()))?;
            let (model, parts) =
                split_in_place(&target.model(), target.key(), tool).map_err(raise)?;
            PySplitResult::new(model, parts, SplitPart::Sheet)
        }
        SplitTarget::Face(target) => {
            let shape = target
                .inner
                .isolated_shape()
                .map_err(|error| raise(error.into()))?;
            let (model, parts) = split_alone(shape, tool).map_err(raise)?;
            PySplitResult::new(model, parts, SplitPart::Face)
        }
        SplitTarget::Profile(target) => {
            let shape = target
                .inner
                .isolated_shape()
                .map_err(|error| raise(error.into()))?;
            let (model, parts) = split_alone(shape, tool).map_err(raise)?;
            PySplitResult::new(model, parts, SplitPart::Profile)
        }
        SplitTarget::Edge(target) => {
            let shape = target
                .inner
                .isolated_shape()
                .map_err(|error| raise(error.into()))?;
            let (model, parts) = split_alone(shape, tool).map_err(raise)?;
            PySplitResult::new(model, parts, SplitPart::Edge)
        }
    })
}
