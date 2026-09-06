use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use crate::geometry::{Curve, Surface};

use super::{
    PyCircle, PyCylinder, PyEllipse, PyLine, PyNurbsCurve, PyNurbsSurface, PyPlane, PyRuledSurface,
    PySurfaceOfRevolution,
};

pub(crate) fn curve_to_py(py: Python<'_>, curve: Curve) -> PyResult<PyObject> {
    match curve {
        Curve::Line(line) => Ok(Py::new(py, PyLine { line })?.into_py(py)),
        Curve::Circle(circle) => Ok(Py::new(py, PyCircle { circle })?.into_py(py)),
        Curve::Ellipse(ellipse) => Ok(Py::new(py, PyEllipse { ellipse })?.into_py(py)),
        Curve::Nurbs(curve) => Ok(Py::new(py, PyNurbsCurve { curve })?.into_py(py)),
        Curve::Bounded(curve) => {
            let curve = curve.to_nurbs().map_err(|err| {
                PyValueError::new_err(format!("failed to convert bounded curve to nurbs: {err}"))
            })?;
            Ok(Py::new(py, PyNurbsCurve { curve })?.into_py(py))
        }
    }
}

pub(crate) fn surface_to_py(py: Python<'_>, surface: Surface) -> PyResult<PyObject> {
    match surface {
        Surface::Plane(plane) => Ok(Py::new(py, PyPlane { plane })?.into_py(py)),
        Surface::Cylinder(cylinder) => Ok(Py::new(py, PyCylinder { cylinder })?.into_py(py)),
        Surface::Ruled(surface) => Ok(Py::new(py, PyRuledSurface { surface })?.into_py(py)),
        Surface::Revolution(surface) => {
            Ok(Py::new(py, PySurfaceOfRevolution { surface })?.into_py(py))
        }
        Surface::Nurbs(surface) => Ok(Py::new(py, PyNurbsSurface { surface })?.into_py(py)),
    }
}
