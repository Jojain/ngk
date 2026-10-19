use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate::binding_common::measurement::tensor;
use crate::measure::{Area, Length, LinearProperties, SurfaceProperties, Volume, VolumeProperties};

use super::geometry::{PyPoint3, point};

#[pyclass(name = "LinearProperties", module = "ngk")]
pub struct PyLinearProperties {
    inner: LinearProperties,
}

#[pymethods]
impl PyLinearProperties {
    #[getter]
    fn length(&self) -> f64 {
        self.inner.length
    }
    #[getter]
    fn centroid(&self) -> PyPoint3 {
        point(self.inner.centroid)
    }
    #[getter]
    fn inertia(&self) -> [f64; 9] {
        tensor::<Length>(self.inner.inertia)
    }
}

#[pyclass(name = "SurfaceProperties", module = "ngk")]
pub struct PySurfaceProperties {
    inner: SurfaceProperties,
}

#[pymethods]
impl PySurfaceProperties {
    #[getter]
    fn area(&self) -> f64 {
        self.inner.area
    }
    #[getter]
    fn centroid(&self) -> PyPoint3 {
        point(self.inner.centroid)
    }
    #[getter]
    fn inertia(&self) -> [f64; 9] {
        tensor::<Area>(self.inner.inertia)
    }
}

#[pyclass(name = "VolumeProperties", module = "ngk")]
pub struct PyVolumeProperties {
    inner: VolumeProperties,
}

#[pymethods]
impl PyVolumeProperties {
    #[getter]
    fn volume(&self) -> f64 {
        self.inner.volume
    }
    #[getter]
    fn centroid(&self) -> PyPoint3 {
        point(self.inner.centroid)
    }
    #[getter]
    fn inertia(&self) -> [f64; 9] {
        tensor::<Volume>(self.inner.inertia)
    }
}

pub(super) fn linear(value: Result<LinearProperties, String>) -> PyResult<PyLinearProperties> {
    value
        .map(|inner| PyLinearProperties { inner })
        .map_err(PyValueError::new_err)
}

pub(super) fn surface(value: Result<SurfaceProperties, String>) -> PyResult<PySurfaceProperties> {
    value
        .map(|inner| PySurfaceProperties { inner })
        .map_err(PyValueError::new_err)
}

pub(super) fn volume(value: Result<VolumeProperties, String>) -> PyResult<PyVolumeProperties> {
    value
        .map(|inner| PyVolumeProperties { inner })
        .map_err(PyValueError::new_err)
}

pub(super) fn register(parent: &Bound<'_, PyModule>) -> PyResult<()> {
    let module = PyModule::new_bound(parent.py(), "measurement")?;
    let qualified = "ngk.core.measurement";
    parent
        .py()
        .import_bound("sys")?
        .getattr("modules")?
        .set_item(qualified, &module)?;
    module.add_class::<PyLinearProperties>()?;
    module.add_class::<PySurfaceProperties>()?;
    module.add_class::<PyVolumeProperties>()?;
    parent.add_submodule(&module)?;
    Ok(())
}
