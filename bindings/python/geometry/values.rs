use nalgebra::{UnitVector3, Vector3};
use pyo3::prelude::*;

use crate::geometry::Point3;

pub(crate) fn point(point: Point3) -> PyPoint3 {
    PyPoint3 { point }
}

pub(crate) fn vector(vector: Vector3<f64>) -> PyVector3 {
    PyVector3 { vector }
}

pub(crate) fn unit_vector(vector: UnitVector3<f64>) -> PyVector3 {
    PyVector3 {
        vector: vector.into_inner(),
    }
}

#[pyclass(name = "Point3", module = "ngk")]
#[derive(Clone)]
pub(crate) struct PyPoint3 {
    pub(crate) point: Point3,
}

#[pymethods]
impl PyPoint3 {
    #[getter]
    fn x(&self) -> f64 {
        self.point.x
    }

    #[getter]
    fn y(&self) -> f64 {
        self.point.y
    }

    #[getter]
    fn z(&self) -> f64 {
        self.point.z
    }

    fn as_tuple(&self) -> (f64, f64, f64) {
        (self.point.x, self.point.y, self.point.z)
    }

    fn __repr__(&self) -> String {
        format!(
            "Point3({}, {}, {})",
            self.point.x, self.point.y, self.point.z
        )
    }
}

#[pyclass(name = "Vector3", module = "ngk")]
#[derive(Clone)]
pub(crate) struct PyVector3 {
    vector: Vector3<f64>,
}

#[pymethods]
impl PyVector3 {
    #[getter]
    fn x(&self) -> f64 {
        self.vector.x
    }

    #[getter]
    fn y(&self) -> f64 {
        self.vector.y
    }

    #[getter]
    fn z(&self) -> f64 {
        self.vector.z
    }

    fn as_tuple(&self) -> (f64, f64, f64) {
        (self.vector.x, self.vector.y, self.vector.z)
    }

    fn __repr__(&self) -> String {
        format!(
            "Vector3({}, {}, {})",
            self.vector.x, self.vector.y, self.vector.z
        )
    }
}
