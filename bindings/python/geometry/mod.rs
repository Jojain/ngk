mod convert;
mod curves;
mod nurbs;
mod surfaces;
mod values;

use pyo3::prelude::*;
use pyo3::types::PyModule;

pub(crate) use convert::{curve_to_py, surface_to_py};
pub(crate) use curves::{PyCircle, PyEllipse, PyLine};
pub(crate) use nurbs::{PyNurbsCurve, PyNurbsSurface};
pub(crate) use surfaces::{
    PyCone, PyCylinder, PyPlane, PyRuledSurface, PySphere, PySurfaceOfRevolution, PyTorus,
};
pub(crate) use values::{
    PyAxis3, PyFrame, PyPoint2, PyPoint3, PyVector2, PyVector3, point, unit_vector, vector,
};

pub(super) fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyPoint2>()?;
    m.add_class::<PyVector2>()?;
    m.add_class::<PyPoint3>()?;
    m.add_class::<PyVector3>()?;
    m.add_class::<PyAxis3>()?;
    m.add_class::<PyFrame>()?;
    m.add_class::<PyLine>()?;
    m.add_class::<PyCircle>()?;
    m.add_class::<PyEllipse>()?;
    m.add_class::<PyNurbsCurve>()?;
    m.add_class::<PyPlane>()?;
    m.add_class::<PyCylinder>()?;
    m.add_class::<PySphere>()?;
    m.add_class::<PyCone>()?;
    m.add_class::<PyTorus>()?;
    m.add_class::<PyRuledSurface>()?;
    m.add_class::<PySurfaceOfRevolution>()?;
    m.add_class::<PyNurbsSurface>()?;
    Ok(())
}
