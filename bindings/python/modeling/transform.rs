use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate::modeling;

use super::super::geometry::PyRigid;
use super::super::topology::{PyEdge, PyFace, PyProfile, PySheet, PySolid};
use super::common::{py_edge, py_face, py_profile, py_sheet, py_solid};

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_function(wrap_pyfunction!(moved_edge, module)?)?;
    module.add_function(wrap_pyfunction!(moved_profile, module)?)?;
    module.add_function(wrap_pyfunction!(moved_face, module)?)?;
    module.add_function(wrap_pyfunction!(moved_sheet, module)?)?;
    module.add_function(wrap_pyfunction!(moved_solid, module)?)?;
    Ok(())
}

macro_rules! moved_shape {
    ($name:ident, $shape:ty, $convert:ident) => {
        #[pyfunction]
        pub(crate) fn $name(shape: &$shape, motion: &PyRigid) -> PyResult<$shape> {
            let owned = shape
                .inner
                .isolated_shape()
                .map_err(|error| PyValueError::new_err(error.to_string()))?;
            $convert(modeling::transform::moved(owned, motion.inner))
        }
    };
}

moved_shape!(moved_edge, PyEdge, py_edge);
moved_shape!(moved_profile, PyProfile, py_profile);
moved_shape!(moved_face, PyFace, py_face);
moved_shape!(moved_sheet, PySheet, py_sheet);
moved_shape!(moved_solid, PySolid, py_solid);
