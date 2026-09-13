use nalgebra::{Point2, UnitVector3, Vector2, Vector3};
use pyo3::prelude::*;

use crate::geometry::axis::Axis3;
use crate::geometry::{Frame, Point3};

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

#[pyclass(name = "Point2", module = "ngk.geometry")]
#[derive(Clone)]
pub(crate) struct PyPoint2 {
    point: Point2<f64>,
}

#[pymethods]
impl PyPoint2 {
    #[new]
    fn new(x: f64, y: f64) -> Self {
        Self {
            point: Point2::new(x, y),
        }
    }

    #[getter]
    fn x(&self) -> f64 {
        self.point.x
    }

    #[getter]
    fn y(&self) -> f64 {
        self.point.y
    }

    fn as_tuple(&self) -> (f64, f64) {
        (self.point.x, self.point.y)
    }
}

#[pyclass(name = "Vector2", module = "ngk.geometry")]
#[derive(Clone)]
pub(crate) struct PyVector2 {
    vector: Vector2<f64>,
}

#[pymethods]
impl PyVector2 {
    #[new]
    fn new(x: f64, y: f64) -> Self {
        Self {
            vector: Vector2::new(x, y),
        }
    }

    #[getter]
    fn x(&self) -> f64 {
        self.vector.x
    }

    #[getter]
    fn y(&self) -> f64 {
        self.vector.y
    }

    fn as_tuple(&self) -> (f64, f64) {
        (self.vector.x, self.vector.y)
    }
}

#[pyclass(name = "Point", module = "ngk.geometry")]
#[derive(Clone)]
pub(crate) struct PyPoint3 {
    pub(crate) point: Point3,
}

#[pymethods]
impl PyPoint3 {
    #[new]
    fn new(x: f64, y: f64, z: f64) -> Self {
        point(Point3::new(x, y, z))
    }
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
            "Point({}, {}, {})",
            self.point.x, self.point.y, self.point.z
        )
    }
}

#[pyclass(name = "Vector", module = "ngk.geometry")]
#[derive(Clone)]
pub(crate) struct PyVector3 {
    vector: Vector3<f64>,
}

#[pymethods]
impl PyVector3 {
    #[new]
    fn new(x: f64, y: f64, z: f64) -> Self {
        vector(Vector3::new(x, y, z))
    }
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
            "Vector({}, {}, {})",
            self.vector.x, self.vector.y, self.vector.z
        )
    }
}

/// A directed three-dimensional line used to place and orient geometry.
#[pyclass(name = "Axis", module = "ngk.geometry")]
#[derive(Clone)]
pub(crate) struct PyAxis3 {
    pub(crate) axis: Axis3,
}

#[pymethods]
impl PyAxis3 {
    #[new]
    fn new(origin: PyPoint3, direction: PyVector3) -> Self {
        Self {
            axis: Axis3::new(origin.point, direction.vector),
        }
    }

    #[staticmethod]
    fn from_points(start: PyPoint3, end: PyPoint3) -> Self {
        Self {
            axis: Axis3::from_points(start.point, end.point),
        }
    }

    #[getter]
    fn origin(&self) -> PyPoint3 {
        point(self.axis.origin)
    }

    #[getter]
    fn direction(&self) -> PyVector3 {
        unit_vector(self.axis.direction)
    }

    fn project(&self, point_to_project: PyPoint3) -> PyPoint3 {
        point(self.axis.project(point_to_project.point))
    }
}

/// An orthonormal three-dimensional placement frame.
#[pyclass(name = "Frame", module = "ngk.geometry")]
#[derive(Clone)]
pub(crate) struct PyFrame {
    pub(crate) frame: Frame,
}

#[pymethods]
impl PyFrame {
    #[new]
    #[pyo3(signature = (origin, x_dir, y_dir))]
    fn new(origin: PyPoint3, x_dir: PyVector3, y_dir: PyVector3) -> Self {
        Self {
            frame: Frame::from_xy(origin.point, x_dir.vector, y_dir.vector),
        }
    }

    #[staticmethod]
    fn xyz() -> Self {
        Self {
            frame: Frame::xyz(),
        }
    }

    #[staticmethod]
    fn from_xy(origin: PyPoint3, x_dir: PyVector3, y_dir: PyVector3) -> Self {
        Self::new(origin, x_dir, y_dir)
    }

    #[staticmethod]
    fn from_xz(origin: PyPoint3, x_dir: PyVector3, z_dir: PyVector3) -> Self {
        Self {
            frame: Frame::from_xz(origin.point, x_dir.vector, z_dir.vector),
        }
    }

    #[getter]
    fn origin(&self) -> PyPoint3 {
        point(self.frame.origin)
    }

    #[getter]
    fn x_dir(&self) -> PyVector3 {
        unit_vector(self.frame.x_dir)
    }

    #[getter]
    fn y_dir(&self) -> PyVector3 {
        unit_vector(self.frame.y_dir)
    }

    #[getter]
    fn z_dir(&self) -> PyVector3 {
        unit_vector(self.frame.z_dir)
    }

    #[getter]
    fn x_axis(&self) -> PyAxis3 {
        PyAxis3 {
            axis: self.frame.x_axis(),
        }
    }

    #[getter]
    fn y_axis(&self) -> PyAxis3 {
        PyAxis3 {
            axis: self.frame.y_axis(),
        }
    }

    #[getter]
    fn z_axis(&self) -> PyAxis3 {
        PyAxis3 {
            axis: self.frame.z_axis(),
        }
    }

    fn point_at(&self, coordinates: PyVector3) -> PyPoint3 {
        point(self.frame.point_at(coordinates.vector))
    }

    fn coordinates_of(&self, point_to_locate: PyPoint3) -> PyVector3 {
        vector(self.frame.coordinates_of(point_to_locate.point))
    }
}
