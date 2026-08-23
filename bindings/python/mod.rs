#![allow(unsafe_op_in_unsafe_fn)]
#![allow(clippy::useless_conversion)]

mod geometry;
mod modeling;
mod topology;
mod visualization;

use pyo3::prelude::*;
use pyo3::types::PyModule;

#[pymodule]
pub fn _ngk(m: &Bound<'_, PyModule>) -> PyResult<()> {
    geometry::register(m)?;
    modeling::register(m)?;
    topology::register(m)?;
    visualization::register(m)?;
    Ok(())
}
