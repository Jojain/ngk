#![allow(unsafe_op_in_unsafe_fn)]
#![allow(clippy::useless_conversion)]
// `create_exception!` expands to a check of a pyo3 feature this crate cannot declare.
#![allow(unexpected_cfgs)]

mod exchange;
mod geometry;
mod measurement;
mod modeling;
mod solver;
mod tessellation;
mod topology;
mod visualization;

use pyo3::prelude::*;
use pyo3::types::PyModule;

#[pymodule]
pub fn core(m: &Bound<'_, PyModule>) -> PyResult<()> {
    exchange::register(m)?;
    geometry::register(m)?;
    measurement::register(m)?;
    modeling::register(m)?;
    solver::register(m)?;
    tessellation::register(m)?;
    topology::register(m)?;
    visualization::register(m)?;
    Ok(())
}
