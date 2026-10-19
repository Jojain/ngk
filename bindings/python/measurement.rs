use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate::binding_common::measurement::tensor;
use crate::geometry::{BBox, Bound as BoxBound};
use crate::measure::{Area, Length, LinearProperties, SurfaceProperties, Volume, VolumeProperties};

use super::geometry::{PyFrame, PyPoint3, PyVector3, point, vector};

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

/// One end of a box's range along a frame axis, for picking a corner.
#[pyclass(name = "Bound", module = "ngk", eq, eq_int)]
#[derive(Clone, Copy, PartialEq)]
pub enum PyBound {
    Min,
    Max,
}

impl From<PyBound> for BoxBound {
    fn from(bound: PyBound) -> Self {
        match bound {
            PyBound::Min => BoxBound::Min,
            PyBound::Max => BoxBound::Max,
        }
    }
}

/// A box given by a frame and a range along each of its axes.
#[pyclass(name = "BoundingBox", module = "ngk")]
pub struct PyBoundingBox {
    inner: BBox,
}

#[pymethods]
impl PyBoundingBox {
    #[getter]
    fn frame(&self) -> PyFrame {
        PyFrame {
            frame: self.inner.frame().clone(),
        }
    }
    #[getter]
    fn min(&self) -> PyPoint3 {
        point(self.inner.min())
    }
    #[getter]
    fn max(&self) -> PyPoint3 {
        point(self.inner.max())
    }
    #[getter]
    fn center(&self) -> PyPoint3 {
        point(self.inner.center())
    }
    #[getter]
    fn size(&self) -> PyVector3 {
        vector(self.inner.size())
    }
    #[getter]
    fn x_size(&self) -> f64 {
        self.inner.x_size()
    }
    #[getter]
    fn y_size(&self) -> f64 {
        self.inner.y_size()
    }
    #[getter]
    fn z_size(&self) -> f64 {
        self.inner.z_size()
    }
    #[getter]
    fn diagonal_length(&self) -> f64 {
        self.inner.diagonal_length()
    }
    fn corner(&self, x: PyBound, y: PyBound, z: PyBound) -> PyPoint3 {
        point(self.inner.corner(x.into(), y.into(), z.into()))
    }
    fn corners(&self) -> Vec<PyPoint3> {
        self.inner.corners().into_iter().map(point).collect()
    }
    fn __repr__(&self) -> String {
        let (min, max) = (self.inner.min(), self.inner.max());
        format!(
            "BoundingBox(min=({}, {}, {}), max=({}, {}, {}))",
            min.x, min.y, min.z, max.x, max.y, max.z
        )
    }
}

pub(super) fn bounding_box(value: Result<BBox, String>) -> PyResult<PyBoundingBox> {
    value
        .map(|inner| PyBoundingBox { inner })
        .map_err(PyValueError::new_err)
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
    module.add_class::<PyBoundingBox>()?;
    module.add_class::<PyBound>()?;
    parent.add_submodule(&module)?;
    Ok(())
}
