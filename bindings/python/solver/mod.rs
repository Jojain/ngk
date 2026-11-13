//! The constraint solvers: `ngk.solver`, `ngk.solver.constraints` and
//! `ngk.solver.assembly`.

mod assembly;
mod constraints;
mod errors;
mod ids;
mod report;
mod system;
mod targets;
mod values;

use pyo3::prelude::*;
use pyo3::types::PyModule;

use super::modeling::{EXTENSION_QUALNAME, new_submodule};

pub(crate) use ids::CurveArg;
pub(crate) use system::System2d;

pub(super) fn register(parent: &Bound<'_, PyModule>) -> PyResult<()> {
    let py = parent.py();
    let solver = new_submodule(py, "solver", EXTENSION_QUALNAME)?;
    let solver_qualname = format!("{EXTENSION_QUALNAME}.solver");

    ids::register(&solver)?;
    values::register(&solver)?;
    report::register(&solver)?;
    system::register(&solver)?;
    targets::register(&solver)?;
    errors::register(&solver)?;

    let constraints_module = new_submodule(py, "constraints", &solver_qualname)?;
    constraints::register(&constraints_module)?;
    solver.add_submodule(&constraints_module)?;

    let assembly_module = new_submodule(py, "assembly", &solver_qualname)?;
    assembly::register(&assembly_module)?;
    solver.add_submodule(&assembly_module)?;

    parent.add_submodule(&solver)?;
    Ok(())
}
