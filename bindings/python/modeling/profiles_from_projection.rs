use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate::binding_common::explore::SharedModel;
use crate::modeling;

use super::super::projection::{PyCurveProjection, chain_message};
use super::super::topology::{PyEdge, PyProfile};

/// A projected chain as the profile it became.
#[pyclass(module = "ngk.modeling.profiles", frozen)]
pub(crate) struct ProjectionProfile {
    profile: PyProfile,
    edges: Vec<PyEdge>,
}

#[pymethods]
impl ProjectionProfile {
    #[getter]
    fn profile(&self) -> PyProfile {
        self.profile.clone()
    }

    /// The edge of each segment of the chain, in the chain's order.
    #[getter]
    fn edges(&self) -> Vec<PyEdge> {
        self.edges.clone()
    }
}

/// What `from_projection` drew: one profile per chain, in the chain order.
#[pyclass(module = "ngk.modeling.profiles", frozen)]
pub(crate) struct ProjectionProfiles {
    profiles: Vec<Py<ProjectionProfile>>,
}

#[pymethods]
impl ProjectionProfiles {
    #[getter]
    fn profiles(&self, py: Python<'_>) -> Vec<Py<ProjectionProfile>> {
        self.profiles
            .iter()
            .map(|profile| profile.clone_ref(py))
            .collect()
    }
}

/// Draws the profiles of a curve projection, in a fresh model.
///
/// Each chain becomes one profile, closed when the chain is, with one edge per
/// segment. A projection without chains, or with a chain whose segments do not
/// continue one another, is refused with a `ValueError`.
#[pyfunction]
pub(super) fn from_projection(
    py: Python<'_>,
    projection: PyRef<'_, PyCurveProjection>,
) -> PyResult<ProjectionProfiles> {
    let (model, built) = modeling::profiles::from_projection(&projection.without_hits())
        .map_err(|error| PyValueError::new_err(chain_message(&error)))?;
    let model = SharedModel::from_model(model);
    let missing = |what: String| PyValueError::new_err(format!("missing {what}"));
    let profiles = built
        .profiles
        .iter()
        .map(|added| {
            let profile = model
                .profile_by_key(added.profile)
                .ok_or_else(|| missing(format!("profile {:?}", added.profile)))?;
            let edges = added
                .edges
                .iter()
                .map(|key| {
                    model
                        .edge_by_key(*key)
                        .map(PyEdge::from_inner)
                        .ok_or_else(|| missing(format!("edge {key:?}")))
                })
                .collect::<PyResult<Vec<_>>>()?;
            Py::new(
                py,
                ProjectionProfile {
                    profile: PyProfile::from_inner(profile),
                    edges,
                },
            )
        })
        .collect::<PyResult<Vec<_>>>()?;
    Ok(ProjectionProfiles { profiles })
}

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_function(wrap_pyfunction!(from_projection, module)?)?;
    module.add_class::<ProjectionProfile>()?;
    module.add_class::<ProjectionProfiles>()?;
    Ok(())
}
