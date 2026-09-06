use pyo3::prelude::*;

use crate::geometry::{Cone, Cylinder, Plane, RuledSurface, Sphere, SurfaceOfRevolution};

use super::{PyPoint3, PyVector3, curve_to_py, point, unit_vector, vector};

#[pyclass(name = "Plane", module = "ngk")]
#[derive(Clone)]
pub(crate) struct PyPlane {
    pub(super) plane: Plane,
}

#[pymethods]
impl PyPlane {
    #[getter]
    fn origin(&self) -> PyPoint3 {
        point(self.plane.origin())
    }

    #[getter]
    fn x_dir(&self) -> PyVector3 {
        unit_vector(self.plane.x_dir())
    }

    #[getter]
    fn y_dir(&self) -> PyVector3 {
        unit_vector(self.plane.y_dir())
    }

    #[getter]
    fn normal(&self) -> PyVector3 {
        unit_vector(self.plane.normal())
    }

    fn point_at(&self, u: f64, v: f64) -> PyPoint3 {
        point(self.plane.point_at(u, v))
    }

    fn normal_at(&self, _u: f64, _v: f64) -> PyVector3 {
        unit_vector(self.plane.normal())
    }

    fn __repr__(&self) -> &'static str {
        "Plane()"
    }
}

#[pyclass(name = "Cylinder", module = "ngk")]
#[derive(Clone)]
pub(crate) struct PyCylinder {
    pub(super) cylinder: Cylinder,
}

#[pymethods]
impl PyCylinder {
    #[getter]
    fn origin(&self) -> PyPoint3 {
        point(self.cylinder.origin())
    }

    #[getter]
    fn x_dir(&self) -> PyVector3 {
        unit_vector(self.cylinder.x_dir())
    }

    #[getter]
    fn axis(&self) -> PyVector3 {
        unit_vector(self.cylinder.axis())
    }

    #[getter]
    fn radius(&self) -> f64 {
        self.cylinder.radius
    }

    fn point_at(&self, u: f64, v: f64) -> PyPoint3 {
        point(self.cylinder.point_at(u, v))
    }

    fn normal_at(&self, u: f64, v: f64) -> PyVector3 {
        unit_vector(self.cylinder.normal_at(u, v))
    }

    fn __repr__(&self) -> String {
        format!("Cylinder(radius={})", self.cylinder.radius)
    }
}

#[pyclass(name = "Sphere", module = "ngk")]
#[derive(Clone)]
pub(crate) struct PySphere {
    pub(super) sphere: Sphere,
}

#[pymethods]
impl PySphere {
    #[getter]
    fn origin(&self) -> PyPoint3 {
        point(self.sphere.frame().origin)
    }

    #[getter]
    fn x_dir(&self) -> PyVector3 {
        unit_vector(self.sphere.frame().x_dir)
    }

    #[getter]
    fn axis(&self) -> PyVector3 {
        unit_vector(self.sphere.frame().z_dir)
    }

    #[getter]
    fn radius(&self) -> f64 {
        self.sphere.radius()
    }

    fn point_at(&self, u: f64, v: f64) -> PyPoint3 {
        point(self.sphere.point_at(u, v))
    }

    fn normal_at(&self, u: f64, v: f64) -> PyVector3 {
        unit_vector(self.sphere.normal_at(u, v))
    }

    fn __repr__(&self) -> String {
        format!("Sphere(radius={})", self.sphere.radius())
    }
}

#[pyclass(name = "Cone", module = "ngk")]
#[derive(Clone)]
pub(crate) struct PyCone {
    pub(super) cone: Cone,
}

#[pymethods]
impl PyCone {
    #[getter]
    fn origin(&self) -> PyPoint3 {
        point(self.cone.frame().origin)
    }

    #[getter]
    fn x_dir(&self) -> PyVector3 {
        unit_vector(self.cone.frame().x_dir)
    }

    #[getter]
    fn axis(&self) -> PyVector3 {
        unit_vector(self.cone.frame().z_dir)
    }

    #[getter]
    fn reference_radius(&self) -> f64 {
        self.cone.reference_radius()
    }

    #[getter]
    fn half_angle(&self) -> f64 {
        self.cone.half_angle()
    }

    #[getter]
    fn apex_parameter(&self) -> Option<f64> {
        self.cone.apex_parameter()
    }

    fn point_at(&self, u: f64, v: f64) -> PyPoint3 {
        point(self.cone.point_at(u, v))
    }

    fn normal_at(&self, u: f64, v: f64) -> PyVector3 {
        unit_vector(self.cone.normal_at(u, v))
    }

    fn __repr__(&self) -> String {
        format!(
            "Cone(reference_radius={}, half_angle={})",
            self.cone.reference_radius(),
            self.cone.half_angle()
        )
    }
}

#[pyclass(name = "RuledSurface", module = "ngk")]
#[derive(Clone)]
pub(crate) struct PyRuledSurface {
    pub(super) surface: RuledSurface,
}

#[pymethods]
impl PyRuledSurface {
    #[getter]
    fn curve(&self, py: Python<'_>) -> PyResult<PyObject> {
        curve_to_py(py, self.surface.curve().clone())
    }

    #[getter]
    fn direction(&self) -> PyVector3 {
        vector(self.surface.direction())
    }

    fn point_at(&self, u: f64, v: f64) -> PyPoint3 {
        point(self.surface.point_at(u, v))
    }

    fn normal_at(&self, u: f64, v: f64) -> PyVector3 {
        unit_vector(self.surface.normal_at(u, v))
    }

    fn __repr__(&self) -> &'static str {
        "RuledSurface()"
    }
}

#[pyclass(name = "SurfaceOfRevolution", module = "ngk")]
#[derive(Clone)]
pub(crate) struct PySurfaceOfRevolution {
    pub(super) surface: SurfaceOfRevolution,
}

#[pymethods]
impl PySurfaceOfRevolution {
    #[getter]
    fn curve(&self, py: Python<'_>) -> PyResult<PyObject> {
        curve_to_py(py, self.surface.curve().clone())
    }

    #[getter]
    fn origin(&self) -> PyPoint3 {
        point(self.surface.origin())
    }

    #[getter]
    fn axis(&self) -> PyVector3 {
        unit_vector(self.surface.axis.direction)
    }

    fn point_at(&self, u: f64, v: f64) -> PyPoint3 {
        point(self.surface.point_at(u, v))
    }

    fn normal_at(&self, u: f64, v: f64) -> PyVector3 {
        unit_vector(self.surface.normal_at(u, v))
    }

    fn __repr__(&self) -> &'static str {
        "SurfaceOfRevolution()"
    }
}
