use pyo3::prelude::*;

use crate::geometry::{Curve, Surface};

use super::{
    PyCircle, PyCone, PyCylinder, PyEllipse, PyHelix, PyLine, PyNurbsCurve, PyNurbsSurface,
    PyPlane, PyRuledSurface, PySphere, PySurfaceOfRevolution, PyTorus,
};

pub(crate) fn curve_to_py(py: Python<'_>, curve: Curve) -> PyResult<PyObject> {
    match curve {
        Curve::Line(line) => Ok(Py::new(py, PyLine { line })?.into_py(py)),
        Curve::Circle(circle) => Ok(Py::new(py, PyCircle { circle })?.into_py(py)),
        Curve::Ellipse(ellipse) => Ok(Py::new(py, PyEllipse { ellipse })?.into_py(py)),
        Curve::Helix(helix) => Ok(Py::new(py, PyHelix { helix })?.into_py(py)),
        Curve::Nurbs(curve) => Ok(Py::new(py, PyNurbsCurve { curve })?.into_py(py)),
    }
}

pub(crate) fn surface_to_py(py: Python<'_>, surface: Surface) -> PyResult<PyObject> {
    match surface {
        Surface::Plane(plane) => Ok(Py::new(py, PyPlane { plane })?.into_py(py)),
        Surface::Cylinder(cylinder) => Ok(Py::new(py, PyCylinder { cylinder })?.into_py(py)),
        Surface::Sphere(sphere) => Ok(Py::new(py, PySphere { sphere })?.into_py(py)),
        Surface::Cone(cone) => Ok(Py::new(py, PyCone { cone })?.into_py(py)),
        Surface::Torus(torus) => Ok(Py::new(py, PyTorus { torus })?.into_py(py)),
        Surface::Ruled(surface) => Ok(Py::new(py, PyRuledSurface { surface })?.into_py(py)),
        Surface::Revolution(surface) => {
            Ok(Py::new(py, PySurfaceOfRevolution { surface })?.into_py(py))
        }
        Surface::Nurbs(surface) => Ok(Py::new(py, PyNurbsSurface { surface })?.into_py(py)),
    }
}

/// The curve a Python curve object holds, or `None` when the object is not one.
pub(crate) fn curve_from_py(object: &Bound<'_, PyAny>) -> Option<Curve> {
    if let Ok(line) = object.extract::<PyRef<'_, PyLine>>() {
        Some(Curve::Line(line.line.clone()))
    } else if let Ok(circle) = object.extract::<PyRef<'_, PyCircle>>() {
        Some(Curve::Circle(circle.circle.clone()))
    } else if let Ok(ellipse) = object.extract::<PyRef<'_, PyEllipse>>() {
        Some(Curve::Ellipse(ellipse.ellipse.clone()))
    } else if let Ok(helix) = object.extract::<PyRef<'_, PyHelix>>() {
        Some(Curve::Helix(helix.helix.clone()))
    } else if let Ok(curve) = object.extract::<PyRef<'_, PyNurbsCurve>>() {
        Some(Curve::Nurbs(curve.curve.clone()))
    } else {
        None
    }
}

/// The surface a Python surface object holds, or `None` when the object is not one.
pub(crate) fn surface_from_py(object: &Bound<'_, PyAny>) -> Option<Surface> {
    if let Ok(plane) = object.extract::<PyRef<'_, PyPlane>>() {
        Some(Surface::Plane(plane.plane.clone()))
    } else if let Ok(cylinder) = object.extract::<PyRef<'_, PyCylinder>>() {
        Some(Surface::Cylinder(cylinder.cylinder.clone()))
    } else if let Ok(sphere) = object.extract::<PyRef<'_, PySphere>>() {
        Some(Surface::Sphere(sphere.sphere.clone()))
    } else if let Ok(cone) = object.extract::<PyRef<'_, PyCone>>() {
        Some(Surface::Cone(cone.cone.clone()))
    } else if let Ok(torus) = object.extract::<PyRef<'_, PyTorus>>() {
        Some(Surface::Torus(torus.torus.clone()))
    } else if let Ok(surface) = object.extract::<PyRef<'_, PyRuledSurface>>() {
        Some(Surface::Ruled(surface.surface.clone()))
    } else if let Ok(surface) = object.extract::<PyRef<'_, PySurfaceOfRevolution>>() {
        Some(Surface::Revolution(surface.surface.clone()))
    } else if let Ok(surface) = object.extract::<PyRef<'_, PyNurbsSurface>>() {
        Some(Surface::Nurbs(surface.surface.clone()))
    } else {
        None
    }
}
