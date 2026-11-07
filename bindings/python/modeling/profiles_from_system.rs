use pyo3::create_exception;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyModule};

use crate::binding_common::explore::SharedModel;
use crate::builders::profiles::SketchError;
use crate::modeling;
use crate::solver::CurveId;

use super::super::geometry::{PyPlane, PyPoint2};
use super::super::solver::{CurveArg, System2d};
use super::super::topology::{PyEdge, PyProfile};

create_exception!(
    ngk.modeling.profiles,
    NotSolvedError,
    PyValueError,
    "The system has no solution for its current revision."
);
create_exception!(
    ngk.modeling.profiles,
    BranchingError,
    PyValueError,
    "More than two curve ends are joined: `curves` names them."
);
create_exception!(
    ngk.modeling.profiles,
    JointGapError,
    PyValueError,
    "`first` and `second` are joined but `gap` apart."
);
create_exception!(
    ngk.modeling.profiles,
    CrossingError,
    PyValueError,
    "`first` and `second` meet at `point` without being joined there."
);
create_exception!(
    ngk.modeling.profiles,
    OverlappingError,
    PyValueError,
    "`first` and `second` overlap."
);
create_exception!(
    ngk.modeling.profiles,
    UncertainCrossingError,
    PyValueError,
    "Whether `first` and `second` cross could not be established."
);

create_exception!(
    ngk.modeling.faces,
    CutOffCurveError,
    PyValueError,
    "`curve` is cut at `point`, `gap` away from it."
);
create_exception!(
    ngk.modeling.faces,
    PinchedRegionError,
    PyValueError,
    "A region's boundary runs through `point` twice."
);

/// An exception carrying the attributes that name what is at fault.
fn with_attributes(py: Python<'_>, error: PyErr, attributes: &[(&str, PyObject)]) -> PyErr {
    for (name, value) in attributes {
        if let Err(failure) = error.value_bound(py).setattr(*name, value) {
            return failure;
        }
    }
    error
}

/// The named exception for a refusal to draw a system.
pub(super) fn from_system_error(py: Python<'_>, error: SketchError) -> PyErr {
    let message = error.to_string();
    let curve = |id: CurveId| CurveArg::from(id).into_py(py);
    match error {
        SketchError::NotSolved => NotSolvedError::new_err(message),
        SketchError::Branching { curves } => {
            let curves: Vec<PyObject> = curves.into_iter().map(curve).collect();
            with_attributes(
                py,
                BranchingError::new_err(message),
                &[("curves", curves.into_py(py))],
            )
        }
        SketchError::JointGap { first, second, gap } => with_attributes(
            py,
            JointGapError::new_err(message),
            &[
                ("first", curve(first)),
                ("second", curve(second)),
                ("gap", gap.into_py(py)),
            ],
        ),
        SketchError::Crossing {
            first,
            second,
            point,
        } => with_attributes(
            py,
            CrossingError::new_err(message),
            &[
                ("first", curve(first)),
                ("second", curve(second)),
                ("point", PyPoint2::from(point).into_py(py)),
            ],
        ),
        SketchError::Overlapping { first, second } => with_attributes(
            py,
            OverlappingError::new_err(message),
            &[("first", curve(first)), ("second", curve(second))],
        ),
        SketchError::UncertainCrossing { first, second } => with_attributes(
            py,
            UncertainCrossingError::new_err(message),
            &[("first", curve(first)), ("second", curve(second))],
        ),
        SketchError::CutOffCurve {
            curve: cut,
            point,
            gap,
        } => with_attributes(
            py,
            CutOffCurveError::new_err(message),
            &[
                ("curve", curve(cut)),
                ("point", PyPoint2::from(point).into_py(py)),
                ("gap", gap.into_py(py)),
            ],
        ),
        SketchError::PinchedRegion { point } => with_attributes(
            py,
            PinchedRegionError::new_err(message),
            &[("point", PyPoint2::from(point).into_py(py))],
        ),
        _ => PyValueError::new_err(message),
    }
}

/// One connected run of a system's curves.
#[pyclass(module = "ngk.modeling.profiles", frozen)]
pub(crate) struct SystemProfile {
    profile: PyProfile,
    curves: Vec<CurveArg>,
}

#[pymethods]
impl SystemProfile {
    #[getter]
    fn profile(&self) -> PyProfile {
        self.profile.clone()
    }

    #[getter]
    fn curves(&self, py: Python<'_>) -> Vec<PyObject> {
        self.curves
            .iter()
            .map(|curve| (*curve).into_py(py))
            .collect()
    }
}

/// What `from_system` drew: the edge of each curve, and the profiles they form.
#[pyclass(module = "ngk.modeling.profiles", frozen)]
pub(crate) struct SystemProfiles {
    edges: Vec<(CurveArg, PyEdge)>,
    profiles: Vec<Py<SystemProfile>>,
}

#[pymethods]
impl SystemProfiles {
    /// The edge each curve drew, by the curve's id.
    #[getter]
    fn edges<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyDict>> {
        let edges = PyDict::new_bound(py);
        for (curve, edge) in &self.edges {
            edges.set_item((*curve).into_py(py), edge.clone().into_py(py))?;
        }
        Ok(edges)
    }

    #[getter]
    fn profiles(&self, py: Python<'_>) -> Vec<Py<SystemProfile>> {
        self.profiles
            .iter()
            .map(|profile| profile.clone_ref(py))
            .collect()
    }
}

/// Draws a solved system on `plane`, in a fresh model.
#[pyfunction]
pub(super) fn from_system(
    py: Python<'_>,
    system: PyRef<'_, System2d>,
    plane: PyPlane,
    curves: Vec<CurveArg>,
) -> PyResult<SystemProfiles> {
    let curves: Vec<CurveId> = curves.into_iter().map(Into::into).collect();
    let (model, built) = modeling::profiles::from_system(&system.0, plane.plane, &curves)
        .map_err(|error| from_system_error(py, error))?;
    let model = SharedModel::from_model(model);
    let missing = |what: String| PyValueError::new_err(format!("missing {what}"));
    let edges = built
        .edges
        .iter()
        .map(|(curve, key)| {
            let edge = model
                .edge_by_key(*key)
                .ok_or_else(|| missing(format!("edge {key:?}")))?;
            Ok((CurveArg::from(*curve), PyEdge::from_inner(edge)))
        })
        .collect::<PyResult<Vec<_>>>()?;
    let profiles = built
        .profiles
        .iter()
        .map(|run| {
            let profile = model
                .profile_by_key(run.profile)
                .ok_or_else(|| missing(format!("profile {:?}", run.profile)))?;
            Py::new(
                py,
                SystemProfile {
                    profile: PyProfile::from_inner(profile),
                    curves: run.curves.iter().copied().map(CurveArg::from).collect(),
                },
            )
        })
        .collect::<PyResult<Vec<_>>>()?;
    Ok(SystemProfiles { edges, profiles })
}

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    let py = module.py();
    module.add_function(wrap_pyfunction!(from_system, module)?)?;
    module.add_class::<SystemProfile>()?;
    module.add_class::<SystemProfiles>()?;
    module.add("NotSolvedError", py.get_type_bound::<NotSolvedError>())?;
    module.add("BranchingError", py.get_type_bound::<BranchingError>())?;
    module.add("JointGapError", py.get_type_bound::<JointGapError>())?;
    module.add("CrossingError", py.get_type_bound::<CrossingError>())?;
    module.add("OverlappingError", py.get_type_bound::<OverlappingError>())?;
    module.add(
        "UncertainCrossingError",
        py.get_type_bound::<UncertainCrossingError>(),
    )?;
    Ok(())
}
