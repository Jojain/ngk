mod primitive;

use pyo3::prelude::*;
use pyo3::types::PyModule;

pub(super) fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    primitive::register(m)
}
