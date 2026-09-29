use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate::binding_common::modeling::SharedBlendTarget;
use crate::modeling;
use crate::topology::shape::Shape;

use super::super::topology::{PyEdge, PyFace, PyProfile, PySolid, PyVertex};
use super::common::{py_face, py_profile, py_solid};

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_class::<PyBlendTarget>()?;
    module.add_function(wrap_pyfunction!(filleted_profile, module)?)?;
    module.add_function(wrap_pyfunction!(filleted_face, module)?)?;
    module.add_function(wrap_pyfunction!(filleted_solid, module)?)?;
    module.add_function(wrap_pyfunction!(chamfered_profile, module)?)?;
    module.add_function(wrap_pyfunction!(chamfered_face, module)?)?;
    module.add_function(wrap_pyfunction!(chamfered_solid, module)?)?;
    Ok(())
}

/// A mixed selection of vertices, edges, profiles, and faces from one model.
#[pyclass(name = "BlendTarget", module = "ngk.modeling.blend")]
#[derive(Clone)]
pub(crate) struct PyBlendTarget {
    inner: SharedBlendTarget,
}

#[pymethods]
impl PyBlendTarget {
    #[new]
    fn new() -> Self {
        Self {
            inner: SharedBlendTarget::new(),
        }
    }

    fn add_vertex(&mut self, vertex: &PyVertex) -> PyResult<()> {
        self.inner
            .add(vertex.inner.model(), vertex.inner.key().into())
            .map_err(|error| PyValueError::new_err(error.to_string()))
    }

    fn add_edge(&mut self, edge: &PyEdge) -> PyResult<()> {
        self.inner
            .add(edge.inner.model(), edge.inner.key().into())
            .map_err(|error| PyValueError::new_err(error.to_string()))
    }

    fn add_profile(&mut self, profile: &PyProfile) -> PyResult<()> {
        self.inner
            .add(profile.inner.model(), profile.inner.key().into())
            .map_err(|error| PyValueError::new_err(error.to_string()))
    }

    fn add_face(&mut self, face: &PyFace) -> PyResult<()> {
        self.inner
            .add(face.inner.model(), face.inner.key().into())
            .map_err(|error| PyValueError::new_err(error.to_string()))
    }
}

macro_rules! blend_operation {
    ($name:ident, $input:ty, $output:ident, $operation:path) => {
        #[pyfunction]
        pub(crate) fn $name(
            shape: &$input,
            target: &PyBlendTarget,
            amount: f64,
        ) -> PyResult<$input> {
            let selection = target
                .inner
                .for_model(&shape.inner.model())
                .map_err(|error| PyValueError::new_err(error.to_string()))?;
            let source = shape.inner.model();
            let owned = Shape::new(source.model().clone(), shape.inner.key());
            $operation(owned, selection, amount)
                .map_err(|error| PyValueError::new_err(error.to_string()))
                .and_then($output)
        }
    };
}

blend_operation!(
    filleted_profile,
    PyProfile,
    py_profile,
    modeling::blend::filleted
);
blend_operation!(filleted_face, PyFace, py_face, modeling::blend::filleted);
blend_operation!(filleted_solid, PySolid, py_solid, modeling::blend::filleted);
blend_operation!(
    chamfered_profile,
    PyProfile,
    py_profile,
    modeling::blend::chamfered
);
blend_operation!(chamfered_face, PyFace, py_face, modeling::blend::chamfered);
blend_operation!(
    chamfered_solid,
    PySolid,
    py_solid,
    modeling::blend::chamfered
);
