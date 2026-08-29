mod common;
mod edge;
mod face;
mod gmap;
mod profile;
mod sheet;
mod solid;
mod vertex;

use pyo3::prelude::*;
use pyo3::types::PyModule;

pub(crate) use edge::PyEdge;
pub(crate) use face::PyFace;
pub(crate) use gmap::PyGMap;
pub(crate) use profile::{PyLoop, PyProfile};
pub(crate) use sheet::{PySheet, PyShell};
pub(crate) use solid::PySolid;
pub(crate) use vertex::PyVertex;

pub(super) fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyGMap>()?;
    m.add_class::<PySolid>()?;
    m.add_class::<PyShell>()?;
    m.add_class::<PySheet>()?;
    m.add_class::<PyProfile>()?;
    m.add_class::<PyFace>()?;
    m.add_class::<PyLoop>()?;
    m.add_class::<PyEdge>()?;
    m.add_class::<PyVertex>()?;
    Ok(())
}
