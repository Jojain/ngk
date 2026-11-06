use pyo3::prelude::*;
use pyo3::types::{PyDict, PyModule};

use crate::solver;

use super::super::geometry::PyPoint2;
use super::constraints::TangencyMode;
use super::ids::{ConstraintId, GeometryArg};

/// Why a relation is reported as redundant.
#[pyclass(module = "ngk.solver", eq, eq_int, frozen)]
#[derive(Clone, Copy, PartialEq)]
pub(crate) enum RedundancyKind {
    Duplicate,
    LocalDependency,
    PartialLocalDependency,
}

impl From<solver::RedundancyKind> for RedundancyKind {
    fn from(kind: solver::RedundancyKind) -> Self {
        match kind {
            solver::RedundancyKind::Duplicate => Self::Duplicate,
            solver::RedundancyKind::LocalDependency => Self::LocalDependency,
            solver::RedundancyKind::PartialLocalDependency => Self::PartialLocalDependency,
        }
    }
}

#[pyclass(module = "ngk.solver", frozen, get_all)]
#[derive(Clone)]
pub(crate) struct Redundancy {
    constraint: ConstraintId,
    kind: RedundancyKind,
}

/// A relation's residual in geometric units, and its largest ratio to the tolerance.
#[pyclass(module = "ngk.solver", frozen, get_all)]
#[derive(Clone)]
pub(crate) struct ConstraintResidual {
    constraint: ConstraintId,
    values: Vec<f64>,
    max_tolerance_ratio: f64,
}

impl From<&solver::ConstraintResidual> for ConstraintResidual {
    fn from(residual: &solver::ConstraintResidual) -> Self {
        Self {
            constraint: ConstraintId(residual.constraint),
            values: residual.values.clone(),
            max_tolerance_ratio: residual.max_tolerance_ratio,
        }
    }
}

#[pyclass(module = "ngk.solver", frozen, get_all)]
#[derive(Clone)]
pub(crate) struct TargetReport {
    index: usize,
    achieved: PyPoint2,
    distance: f64,
    reached: bool,
}

/// Diagnostics of the permanent relations; targets consume no freedom.
#[pyclass(module = "ngk.solver", frozen)]
pub(crate) struct SolveReport(pub(crate) solver::SolveReport);

#[pymethods]
impl SolveReport {
    #[getter]
    fn degrees_of_freedom(&self) -> usize {
        self.0.degrees_of_freedom
    }

    /// The linearized freedom of one primitive's own parameters.
    fn freedom(&self, primitive: GeometryArg) -> Option<usize> {
        let primitive = solver::GeometryId::from(primitive);
        self.0
            .freedoms
            .iter()
            .find(|freedom| freedom.geometry == primitive)
            .map(|freedom| freedom.degrees)
    }

    /// The linearized freedom of every primitive. A motion shared by several counts in
    /// each, so these do not add up to `degrees_of_freedom`.
    #[getter]
    fn freedoms<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyDict>> {
        let freedoms = PyDict::new_bound(py);
        for freedom in &self.0.freedoms {
            freedoms.set_item(
                GeometryArg::from(freedom.geometry).into_py(py),
                freedom.degrees,
            )?;
        }
        Ok(freedoms)
    }

    #[getter]
    fn redundancies(&self) -> Vec<Redundancy> {
        self.0
            .redundancies
            .iter()
            .map(|redundancy| Redundancy {
                constraint: ConstraintId(redundancy.constraint),
                kind: redundancy.kind.into(),
            })
            .collect()
    }

    #[getter]
    fn residuals(&self) -> Vec<ConstraintResidual> {
        self.0.residuals.iter().map(Into::into).collect()
    }

    #[getter]
    fn targets(&self) -> Vec<TargetReport> {
        self.0
            .targets
            .iter()
            .map(|target| TargetReport {
                index: target.index,
                achieved: target.achieved.into(),
                distance: target.distance,
                reached: target.reached,
            })
            .collect()
    }

    /// The branch chosen for each circle tangency, to keep when rebuilding the system.
    #[getter]
    fn tangencies(&self) -> Vec<(ConstraintId, TangencyMode)> {
        self.0
            .tangencies
            .iter()
            .map(|(constraint, mode)| (ConstraintId(*constraint), (*mode).into()))
            .collect()
    }

    #[getter]
    fn iterations(&self) -> usize {
        self.0.iterations
    }

    #[getter]
    fn target_iterations(&self) -> usize {
        self.0.target_iterations
    }

    #[getter]
    fn target_budget_exhausted(&self) -> bool {
        self.0.target_budget_exhausted
    }
}

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_class::<RedundancyKind>()?;
    module.add_class::<Redundancy>()?;
    module.add_class::<ConstraintResidual>()?;
    module.add_class::<TargetReport>()?;
    module.add_class::<SolveReport>()?;
    Ok(())
}
