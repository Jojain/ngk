use pyo3::prelude::*;

use crate::geometry::parameter::NativeParam;
use crate::geometry::{Circle, Ellipse, Helix, Line};

use super::{PyAxis3, PyPlane, PyPoint3, point};

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
        point(self.line.point_at(NativeParam::new(1.0)))
    }

    fn point_at(&self, t: f64) -> PyPoint3 {
        point(self.line.point_at(NativeParam::new(t)))
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
        point(self.circle.point_at(NativeParam::new(t)))
    }

    fn __repr__(&self) -> String {
        format!("Circle(radius={})", self.circle.radius())
    }
}

#[pyclass(name = "Ellipse", module = "ngk")]
#[derive(Clone)]
pub(crate) struct PyEllipse {
    pub(super) ellipse: Ellipse,
}

#[pymethods]
impl PyEllipse {
    #[getter]
    fn major_radius(&self) -> f64 {
        self.ellipse.major_radius()
    }

    #[getter]
    fn minor_radius(&self) -> f64 {
        self.ellipse.minor_radius()
    }

    fn point_at(&self, t: f64) -> PyPoint3 {
        point(self.ellipse.point_at(NativeParam::new(t)))
    }

    fn __repr__(&self) -> String {
        format!(
            "Ellipse(major_radius={}, minor_radius={})",
            self.ellipse.major_radius(),
            self.ellipse.minor_radius()
        )
    }
}

#[pyclass(name = "Helix", module = "ngk")]
#[derive(Clone)]
pub(crate) struct PyHelix {
    pub(super) helix: Helix,
}

#[pymethods]
impl PyHelix {
    #[new]
    fn new(axis: PyAxis3, radius: f64, pitch: f64) -> Self {
        Self {
            helix: Helix::from_axis(axis.axis, radius, pitch),
        }
    }

    #[getter]
    fn axis(&self) -> PyAxis3 {
        PyAxis3 {
            axis: self.helix.axis(),
        }
    }

    #[getter]
    fn radius(&self) -> f64 {
        self.helix.radius()
    }

    #[getter]
    fn pitch(&self) -> f64 {
        self.helix.pitch()
    }

    fn point_at(&self, t: f64) -> PyPoint3 {
        point(self.helix.point_at(NativeParam::new(t)))
    }

    fn __repr__(&self) -> String {
        format!(
            "Helix(radius={}, pitch={})",
            self.helix.radius(),
            self.helix.pitch()
        )
    }
}
