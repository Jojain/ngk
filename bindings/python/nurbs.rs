use pyo3::prelude::*;

use crate::geometry::{NurbsCurve, NurbsSurface};

use super::{PyPoint3, PyVector3, point, unit_vector};

#[pyclass(name = "NurbsCurve", module = "ngk")]
#[derive(Clone)]
pub(super) struct PyNurbsCurve {
    pub(super) curve: NurbsCurve,
}

#[pymethods]
impl PyNurbsCurve {
    #[getter]
    fn degree(&self) -> usize {
        self.curve.degree().get()
    }

    #[getter]
    fn domain(&self) -> (f64, f64) {
        let domain = self.curve.domain();
        (domain.start, domain.end)
    }

    #[getter]
    fn knots(&self) -> Vec<f64> {
        self.curve.knots().as_slice().to_vec()
    }

    #[getter]
    fn control_points(&self) -> Vec<(PyPoint3, f64)> {
        self.curve
            .control_points()
            .iter()
            .map(|p| (point(p.to_cartesian()), p.weight()))
            .collect()
    }

    #[getter]
    fn kind(&self) -> &'static str {
        "nurbs_curve"
    }

    fn point_at(&self, u: f64) -> PyPoint3 {
        point(self.curve.point_at(u))
    }

    fn __repr__(&self) -> String {
        format!("NurbsCurve(degree={})", self.curve.degree().get())
    }
}

#[pyclass(name = "NurbsSurface", module = "ngk")]
#[derive(Clone)]
pub(super) struct PyNurbsSurface {
    pub(super) surface: NurbsSurface,
}

#[pymethods]
impl PyNurbsSurface {
    #[getter]
    fn degree_u(&self) -> usize {
        self.surface.degree_u().get()
    }

    #[getter]
    fn degree_v(&self) -> usize {
        self.surface.degree_v().get()
    }

    #[getter]
    fn domain_u(&self) -> (f64, f64) {
        let domain = self.surface.domain_u();
        (domain.start, domain.end)
    }

    #[getter]
    fn domain_v(&self) -> (f64, f64) {
        let domain = self.surface.domain_v();
        (domain.start, domain.end)
    }

    #[getter]
    fn knots_u(&self) -> Vec<f64> {
        self.surface.knots_u().as_slice().to_vec()
    }

    #[getter]
    fn knots_v(&self) -> Vec<f64> {
        self.surface.knots_v().as_slice().to_vec()
    }

    #[getter]
    fn control_points(&self) -> Vec<Vec<(PyPoint3, f64)>> {
        let points = self.surface.control_points();
        (0..points.nv())
            .map(|v| {
                (0..points.nu())
                    .map(|u| {
                        let point_ = points.get(u, v);
                        (point(point_.to_cartesian()), point_.weight())
                    })
                    .collect()
            })
            .collect()
    }

    #[getter]
    fn kind(&self) -> &'static str {
        "nurbs_surface"
    }

    fn point_at(&self, u: f64, v: f64) -> PyPoint3 {
        point(self.surface.point_at(u, v))
    }

    fn normal_at(&self, u: f64, v: f64) -> PyVector3 {
        unit_vector(self.surface.normal_at(u, v))
    }

    fn __repr__(&self) -> String {
        format!(
            "NurbsSurface(degree_u={}, degree_v={})",
            self.surface.degree_u().get(),
            self.surface.degree_v().get()
        )
    }
}
