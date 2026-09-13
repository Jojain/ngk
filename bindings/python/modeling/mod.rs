mod common;
mod edges;
mod faces;
mod profiles;
mod solids;

use pyo3::prelude::*;
use pyo3::types::PyModule;

pub(super) fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    edges::register(m)?;
    profiles::register(m)?;
    faces::register(m)?;
    solids::register(m)?;
    Ok(())
}
