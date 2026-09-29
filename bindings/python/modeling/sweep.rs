use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate::binding_common::modeling::sweep_options;
use crate::builders::sweep::SweepOptions;
use crate::modeling;

use super::super::geometry::{PyAxis3, PyVector3};
use super::super::topology::{PyEdge, PyFace, PyProfile, PySheet, PySolid};
use super::common::{py_sheet, py_solid};

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_class::<PySweepOptions>()?;
    module.add_function(wrap_pyfunction!(extrude_profile, module)?)?;
    module.add_function(wrap_pyfunction!(extrude_face, module)?)?;
    module.add_function(wrap_pyfunction!(face_along_edge, module)?)?;
    module.add_function(wrap_pyfunction!(face_along_profile, module)?)?;
    Ok(())
}

/// Frame transport, junction transition, and curved-spine resolution.
#[pyclass(name = "SweepOptions", module = "ngk.modeling.sweep")]
#[derive(Clone)]
pub(crate) struct PySweepOptions {
    inner: SweepOptions,
}

#[pymethods]
impl PySweepOptions {
    #[new]
    #[pyo3(signature = (frame="parallel", axis=None, transition="smooth", samples_per_segment=8))]
    fn new(
        frame: &str,
        axis: Option<&PyAxis3>,
        transition: &str,
        samples_per_segment: usize,
    ) -> PyResult<Self> {
        let inner = sweep_options(
            frame,
            axis.map(|value| value.axis),
            transition,
            samples_per_segment,
        )
        .map_err(|error| PyValueError::new_err(error.to_string()))?;
        Ok(Self { inner })
    }
}

/// Extrudes a profile by a displacement vector to make a sheet.
#[pyfunction]
pub(crate) fn extrude_profile(profile: &PyProfile, direction: &PyVector3) -> PyResult<PySheet> {
    let shape = profile.inner.isolated_shape().map_err(py_err)?;
    modeling::sweep::extrude_profile(shape.profile(), direction.vector)
        .map_err(py_err)
        .and_then(py_sheet)
}

/// Extrudes a face by a displacement vector to make a solid.
#[pyfunction]
pub(crate) fn extrude_face(face: &PyFace, direction: &PyVector3) -> PyResult<PySolid> {
    let shape = face.inner.isolated_shape().map_err(py_err)?;
    modeling::sweep::extrude_face(shape, direction.vector)
        .map_err(py_err)
        .and_then(py_solid)
}

/// Sweeps a face along one edge.
#[pyfunction]
#[pyo3(signature = (face, spine, options=None))]
pub(crate) fn face_along_edge(
    face: &PyFace,
    spine: &PyEdge,
    options: Option<&PySweepOptions>,
) -> PyResult<PySolid> {
    let face = face.inner.isolated_shape().map_err(py_err)?;
    let spine = spine.inner.isolated_shape().map_err(py_err)?;
    modeling::sweep::sweep_face(
        face,
        &spine.edge(),
        options.map_or_else(SweepOptions::default, |value| value.inner),
    )
    .map_err(py_err)
    .and_then(py_solid)
}

/// Sweeps a face along an open or closed profile.
#[pyfunction]
#[pyo3(signature = (face, spine, options=None))]
pub(crate) fn face_along_profile(
    face: &PyFace,
    spine: &PyProfile,
    options: Option<&PySweepOptions>,
) -> PyResult<PySolid> {
    let face = face.inner.isolated_shape().map_err(py_err)?;
    let spine = spine.inner.isolated_shape().map_err(py_err)?;
    modeling::sweep::sweep_face(
        face,
        &spine.profile(),
        options.map_or_else(SweepOptions::default, |value| value.inner),
    )
    .map_err(py_err)
    .and_then(py_solid)
}

fn py_err(error: impl ToString) -> PyErr {
    PyValueError::new_err(error.to_string())
}
