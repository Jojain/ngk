use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use crate::StandardPayload;
use crate::binding_common::explore::SharedModel;
use crate::topology::shape::{EdgeTag, FaceTag, ProfileTag, Shape, SolidTag};

use super::super::topology::{PyEdge, PyFace, PyProfile, PySolid};

pub(super) fn py_edge(shape: Shape<EdgeTag, StandardPayload>) -> PyResult<PyEdge> {
    let (map, key) = shape.into_model();
    let map = SharedModel::from_model(map);
    let inner = map
        .edge_by_key(key)
        .ok_or_else(|| PyValueError::new_err(format!("missing edge {key:?}")))?;
    Ok(PyEdge::from_inner(inner))
}

pub(super) fn py_profile(shape: Shape<ProfileTag, StandardPayload>) -> PyResult<PyProfile> {
    let (map, key) = shape.into_model();
    let map = SharedModel::from_model(map);
    let inner = map
        .profile_by_key(key)
        .ok_or_else(|| PyValueError::new_err(format!("missing profile {key:?}")))?;
    Ok(PyProfile::from_inner(inner))
}

pub(super) fn py_face(shape: Shape<FaceTag, StandardPayload>) -> PyResult<PyFace> {
    let (map, key) = shape.into_model();
    let map = SharedModel::from_model(map);
    let inner = map
        .face_by_key(key)
        .ok_or_else(|| PyValueError::new_err(format!("missing face {key:?}")))?;
    Ok(PyFace::from_inner(inner))
}

pub(super) fn py_solid(shape: Shape<SolidTag, StandardPayload>) -> PyResult<PySolid> {
    let (map, key) = shape.into_model();
    let map = SharedModel::from_model(map);
    let inner = map
        .solid_by_key(key)
        .ok_or_else(|| PyValueError::new_err(format!("missing solid {key:?}")))?;
    Ok(PySolid::from_inner(inner))
}
