use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate::binding_common::explore::SharedModel;
use crate::modeling;
use crate::solver::CurveId;

use super::super::geometry::PyPlane;
use super::super::solver::{CurveArg, System2d};
use super::super::topology::{PyEdge, PyFace};
use super::profiles_from_system::{
    CutOffCurveError, JointGapError, NotSolvedError, OverlappingError, PinchedRegionError,
    UncertainCrossingError, from_system_error,
};

/// One closed area a system's curves enclose, as a loose face.
#[pyclass(module = "ngk.modeling.faces", frozen)]
pub(crate) struct SystemRegion {
    face: PyFace,
    boundary: Vec<(CurveArg, PyEdge)>,
}

#[pymethods]
impl SystemRegion {
    #[getter]
    fn face(&self) -> PyFace {
        self.face.clone()
    }

    /// The edges this face runs, each with the curve it lies on.
    #[getter]
    fn boundary(&self, py: Python<'_>) -> Vec<(PyObject, PyObject)> {
        self.boundary
            .iter()
            .map(|(curve, edge)| ((*curve).into_py(py), edge.clone().into_py(py)))
            .collect()
    }
}

/// What `from_system` drew: one loose face per region, all in one model.
#[pyclass(module = "ngk.modeling.faces", frozen)]
pub(crate) struct SystemRegions {
    regions: Vec<Py<SystemRegion>>,
}

#[pymethods]
impl SystemRegions {
    #[getter]
    fn regions(&self, py: Python<'_>) -> Vec<Py<SystemRegion>> {
        self.regions
            .iter()
            .map(|region| region.clone_ref(py))
            .collect()
    }
}

/// Draws a loose face for each closed area a solved system's curves enclose on `plane`, in
/// a fresh model.
#[pyfunction]
pub(super) fn from_system(
    py: Python<'_>,
    system: PyRef<'_, System2d>,
    plane: PyPlane,
    curves: Vec<CurveArg>,
) -> PyResult<SystemRegions> {
    let curves: Vec<CurveId> = curves.into_iter().map(Into::into).collect();
    let (model, sketch) = modeling::faces::from_system(&system.0, plane.plane, &curves)
        .map_err(|error| from_system_error(py, error))?;
    let model = SharedModel::from_model(model);
    let missing = |what: String| PyValueError::new_err(format!("missing {what}"));
    let regions = sketch
        .regions
        .iter()
        .map(|region| {
            let face = model
                .face_by_key(region.face)
                .ok_or_else(|| missing(format!("face {:?}", region.face)))?;
            let boundary = region
                .boundary
                .iter()
                .map(|(curve, key)| {
                    let edge = model
                        .edge_by_key(*key)
                        .ok_or_else(|| missing(format!("edge {key:?}")))?;
                    Ok((CurveArg::from(*curve), PyEdge::from_inner(edge)))
                })
                .collect::<PyResult<Vec<_>>>()?;
            Py::new(
                py,
                SystemRegion {
                    face: PyFace::from_inner(face),
                    boundary,
                },
            )
        })
        .collect::<PyResult<Vec<_>>>()?;
    Ok(SystemRegions { regions })
}

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    let py = module.py();
    module.add_function(wrap_pyfunction!(from_system, module)?)?;
    module.add_class::<SystemRegion>()?;
    module.add_class::<SystemRegions>()?;
    module.add("NotSolvedError", py.get_type_bound::<NotSolvedError>())?;
    module.add("JointGapError", py.get_type_bound::<JointGapError>())?;
    module.add("OverlappingError", py.get_type_bound::<OverlappingError>())?;
    module.add(
        "UncertainCrossingError",
        py.get_type_bound::<UncertainCrossingError>(),
    )?;
    module.add("CutOffCurveError", py.get_type_bound::<CutOffCurveError>())?;
    module.add(
        "PinchedRegionError",
        py.get_type_bound::<PinchedRegionError>(),
    )?;
    Ok(())
}
