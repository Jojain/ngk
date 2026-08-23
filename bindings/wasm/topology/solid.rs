use js_sys::Array;
use wasm_bindgen::prelude::*;

use super::common::{entity_common, js_err, values};
use super::gmap::WasmGMap;

use crate::binding_common::explore::SharedSolid;
use crate::topology::StandardPayload;

use super::edge::WasmEdge;
use super::face::WasmFace;
use super::sheet::WasmShell;
use super::vertex::WasmVertex;

#[wasm_bindgen(js_name = Solid)]
#[derive(Clone)]
pub struct WasmSolid {
    pub(crate) inner: SharedSolid<StandardPayload>,
}

impl WasmSolid {
    pub(crate) fn from_inner(inner: SharedSolid<StandardPayload>) -> Self {
        Self { inner }
    }
}

entity_common!(WasmSolid);

#[wasm_bindgen]
impl WasmSolid {
    /// Returns the outer shell.
    #[wasm_bindgen(getter, js_name = outerShell)]
    pub fn outer_shell(&self) -> Result<WasmShell, JsValue> {
        Ok(WasmShell::from_inner(
            self.inner.outer_shell().map_err(js_err)?,
        ))
    }

    /// Returns inner shells.
    #[wasm_bindgen(js_name = innerShells)]
    pub fn inner_shells(&self) -> Result<Array, JsValue> {
        Ok(values(
            self.inner
                .inner_shells()
                .map_err(js_err)?
                .into_iter()
                .map(|inner| WasmShell::from_inner(inner).into()),
        ))
    }

    /// Returns all shells, outer first.
    pub fn shells(&self) -> Result<Array, JsValue> {
        Ok(values(
            self.inner
                .shells()
                .map_err(js_err)?
                .into_iter()
                .map(|inner| WasmShell::from_inner(inner).into()),
        ))
    }

    /// Returns faces belonging to this solid.
    pub fn faces(&self) -> Result<Array, JsValue> {
        Ok(values(
            self.inner
                .faces()
                .map_err(js_err)?
                .into_iter()
                .map(|inner| WasmFace::from_inner(inner).into()),
        ))
    }

    /// Returns edges belonging to this solid.
    pub fn edges(&self) -> Result<Array, JsValue> {
        Ok(values(
            self.inner
                .edges()
                .map_err(js_err)?
                .into_iter()
                .map(|inner| WasmEdge::from_inner(inner).into()),
        ))
    }

    /// Returns vertices belonging to this solid.
    pub fn vertices(&self) -> Result<Array, JsValue> {
        Ok(values(
            self.inner
                .vertices()
                .map_err(js_err)?
                .into_iter()
                .map(|inner| WasmVertex::from_inner(inner).into()),
        ))
    }

    /// Returns the number of faces belonging to this solid.
    #[wasm_bindgen(getter, js_name = faceCount)]
    pub fn face_count(&self) -> Result<usize, JsValue> {
        Ok(self.inner.faces().map_err(js_err)?.len())
    }

    /// Returns the number of edges belonging to this solid.
    #[wasm_bindgen(getter, js_name = edgeCount)]
    pub fn edge_count(&self) -> Result<usize, JsValue> {
        Ok(self.inner.edges().map_err(js_err)?.len())
    }

    /// Returns the number of vertices belonging to this solid.
    #[wasm_bindgen(getter, js_name = vertexCount)]
    pub fn vertex_count(&self) -> Result<usize, JsValue> {
        Ok(self.inner.vertices().map_err(js_err)?.len())
    }
}
