use pyo3::create_exception;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate::solver::{EditError, SolveError};

use super::ids::ConstraintId;
use super::report::ConstraintResidual;

create_exception!(
    ngk.solver,
    ConflictError,
    PyValueError,
    "Dimensions or fixed geometry contradict each other: `constraints` names them."
);
create_exception!(
    ngk.solver,
    NonConvergenceError,
    PyValueError,
    "Local solving did not converge: `constraints` names the connected group, not a minimal conflict."
);

fn ids(constraints: &[crate::solver::ConstraintId]) -> Vec<ConstraintId> {
    constraints.iter().copied().map(ConstraintId).collect()
}

pub(super) fn solve_error(py: Python<'_>, error: SolveError) -> PyErr {
    let message = error.to_string();
    let raised = |exception: PyErr, constraints: &[crate::solver::ConstraintId]| {
        exception
            .value_bound(py)
            .setattr("constraints", ids(constraints).into_py(py))
            .map_or_else(|failure| failure, |()| exception)
    };
    match &error {
        SolveError::Conflict { constraints } => {
            raised(ConflictError::new_err(message), constraints)
        }
        SolveError::NonConvergence {
            constraints,
            residuals,
        } => {
            let exception = raised(NonConvergenceError::new_err(message), constraints);
            let residuals: Vec<ConstraintResidual> = residuals.iter().map(Into::into).collect();
            exception
                .value_bound(py)
                .setattr("residuals", residuals.into_py(py))
                .map_or_else(|failure| failure, |()| exception)
        }
        _ => PyValueError::new_err(message),
    }
}

pub(super) fn edit_error(error: EditError) -> PyErr {
    PyValueError::new_err(error.to_string())
}

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    let py = module.py();
    module.add("ConflictError", py.get_type_bound::<ConflictError>())?;
    module.add(
        "NonConvergenceError",
        py.get_type_bound::<NonConvergenceError>(),
    )?;
    Ok(())
}
