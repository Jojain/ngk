use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate::binding_common::measurement::{self, MeasuredProperties};

use super::geometry::{PyPoint3, point};
use super::topology::{PyEdge, PyFace, PyProfile, PySheet, PySolid};

#[pyclass(name = "MeasuredProperties", module = "ngk")]
pub struct PyMeasuredProperties {
    inner: MeasuredProperties,
}

#[pymethods]
impl PyMeasuredProperties {
    #[getter]
    fn amount(&self) -> f64 {
        self.inner.amount
    }
    #[getter]
    fn centroid(&self) -> PyPoint3 {
        point(self.inner.centroid)
    }
    #[getter]
    fn inertia(&self) -> [f64; 9] {
        self.inner.inertia
    }
}

pub(super) fn register(parent: &Bound<'_, PyModule>) -> PyResult<()> {
    let module = PyModule::new_bound(parent.py(), "measurement")?;
    let qualified = "ngk.core.measurement";
    parent
        .py()
        .import_bound("sys")?
        .getattr("modules")?
        .set_item(qualified, &module)?;
    module.add_class::<PyMeasuredProperties>()?;
    module.add_function(wrap_pyfunction!(edge_properties, &module)?)?;
    module.add_function(wrap_pyfunction!(profile_properties, &module)?)?;
    module.add_function(wrap_pyfunction!(face_properties, &module)?)?;
    module.add_function(wrap_pyfunction!(sheet_properties, &module)?)?;
    module.add_function(wrap_pyfunction!(solid_properties, &module)?)?;
    parent.add_submodule(&module)?;
    Ok(())
}

fn wrap(value: Result<MeasuredProperties, String>) -> PyResult<PyMeasuredProperties> {
    value
        .map(|inner| PyMeasuredProperties { inner })
        .map_err(PyValueError::new_err)
}

#[pyfunction]
fn edge_properties(value: &PyEdge) -> PyResult<PyMeasuredProperties> {
    wrap(measurement::edge(&value.inner))
}
#[pyfunction]
fn profile_properties(value: &PyProfile) -> PyResult<PyMeasuredProperties> {
    wrap(measurement::profile(&value.inner))
}
#[pyfunction]
fn face_properties(value: &PyFace) -> PyResult<PyMeasuredProperties> {
    wrap(measurement::face(&value.inner))
}
#[pyfunction]
fn sheet_properties(value: &PySheet) -> PyResult<PyMeasuredProperties> {
    wrap(measurement::sheet(&value.inner))
}
#[pyfunction]
fn solid_properties(value: &PySolid) -> PyResult<PyMeasuredProperties> {
    wrap(measurement::solid(&value.inner))
}
