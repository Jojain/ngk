use pyo3::exceptions::{PyTypeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate::tcv::{TcvOptions, to_tcv};
use crate::topology::shape::{EdgeTag, FaceTag, ProfileTag, Shape, SolidTag};

use super::super::topology::{PyEdge, PyFace, PyProfile, PySolid};

pub(super) fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(_to_tcv_json, m)?)?;
    Ok(())
}

#[pyfunction]
#[pyo3(signature = (obj, name=None, color="#e8b024", alpha=1.0))]
fn _to_tcv_json(
    obj: &Bound<'_, PyAny>,
    name: Option<String>,
    color: &str,
    alpha: f64,
) -> PyResult<String> {
    let opts = TcvOptions {
        name: name.unwrap_or_else(|| "shape".to_string()),
        color: color.to_string(),
        alpha,
        ..TcvOptions::default()
    };

    if let Ok(edge) = obj.extract::<PyRef<'_, PyEdge>>() {
        let map = edge.inner.gmap();
        return tcv_json(
            &Shape::<EdgeTag, _>::new(map.map().clone(), edge.inner.key()),
            opts,
        );
    }
    if let Ok(profile) = obj.extract::<PyRef<'_, PyProfile>>() {
        let map = profile.inner.gmap();
        return tcv_json(
            &Shape::<ProfileTag, _>::new(map.map().clone(), profile.inner.key()),
            opts,
        );
    }
    if let Ok(face) = obj.extract::<PyRef<'_, PyFace>>() {
        let map = face.inner.gmap();
        return tcv_json(
            &Shape::<FaceTag, _>::new(map.map().clone(), face.inner.key()),
            opts,
        );
    }
    if let Ok(solid) = obj.extract::<PyRef<'_, PySolid>>() {
        let map = solid.inner.gmap();
        return tcv_json(
            &Shape::<SolidTag, _>::new(map.map().clone(), solid.inner.key()),
            opts,
        );
    }

    Err(PyTypeError::new_err(
        "ngk.to_tcv supports Edge, Profile, Face, and Solid objects",
    ))
}

fn tcv_json<T: crate::tcv::ToTcv>(shape: &T, opts: TcvOptions) -> PyResult<String> {
    let tcv = to_tcv(shape, opts).map_err(|err| PyValueError::new_err(err.to_string()))?;
    serde_json::to_string(&tcv).map_err(|err| PyValueError::new_err(err.to_string()))
}
