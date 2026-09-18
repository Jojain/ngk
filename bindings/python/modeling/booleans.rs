use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate::builders::boolean::BooleanOperation;
use crate::modeling;

use super::super::topology::PySolid;
use super::common::py_solid;

/// Registers the current solid Boolean operations.
///
/// They live in the Boolean namespace rather than the solids namespace so
/// later dimension-generic operations keep the same Python import path.
pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_function(wrap_pyfunction!(fuse, module)?)?;
    module.add_function(wrap_pyfunction!(cut, module)?)?;
    module.add_function(wrap_pyfunction!(intersect, module)?)?;
    Ok(())
}

#[pyfunction]
pub(crate) fn fuse(first: &PySolid, second: &PySolid) -> PyResult<PySolid> {
    combine(first, second, BooleanOperation::Union)
}

#[pyfunction]
pub(crate) fn cut(target: &PySolid, tool: &PySolid) -> PyResult<PySolid> {
    combine(target, tool, BooleanOperation::Difference)
}

#[pyfunction]
pub(crate) fn intersect(first: &PySolid, second: &PySolid) -> PyResult<PySolid> {
    combine(first, second, BooleanOperation::Intersection)
}

fn combine(first: &PySolid, second: &PySolid, operation: BooleanOperation) -> PyResult<PySolid> {
    modeling::solids::combine_views(
        first
            .inner
            .view()
            .map_err(|error| PyValueError::new_err(error.to_string()))?,
        second
            .inner
            .view()
            .map_err(|error| PyValueError::new_err(error.to_string()))?,
        operation,
    )
    .map_err(|error| PyValueError::new_err(error.to_string()))
    .and_then(py_solid)
}
