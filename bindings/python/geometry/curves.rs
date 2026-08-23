use pyo3::prelude::*;

use crate::geometry::{Circle, Line};

use super::{PyPlane, PyPoint3, point};

#[pyclass(name = "Line", module = "ngk")]
#[derive(Clone)]
pub(crate) struct PyLine {
    pub(super) line: Line,
}

#[pymethods]
impl PyLine {
    #[getter]
    fn start(&self) -> PyPoint3 {
        point(self.line.origin())
    }

    #[getter]
    fn end(&self) -> PyPoint3 {
        point(self.line.point_at(1.0))
    }

    fn point_at(&self, t: f64) -> PyPoint3 {
        point(self.line.point_at(t))
    }

    fn __repr__(&self) -> &'static str {
        "Line()"
    }
}

#[pyclass(name = "Circle", module = "ngk")]
#[derive(Clone)]
pub(crate) struct PyCircle {
    pub(super) circle: Circle,
}

#[pymethods]
impl PyCircle {
    #[getter]
    fn plane(&self) -> PyPlane {
        PyPlane {
            plane: self.circle.plane().clone(),
        }
    }

    #[getter]
    fn radius(&self) -> f64 {
        self.circle.radius()
    }

    fn point_at(&self, t: f64) -> PyPoint3 {
        point(self.circle.point_at(t))
    }

    fn __repr__(&self) -> String {
        format!("Circle(radius={})", self.circle.radius())
    }
}
