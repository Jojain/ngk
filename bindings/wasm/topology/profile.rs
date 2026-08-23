use js_sys::Array;
use wasm_bindgen::prelude::*;

use super::common::{entity_common, js_err, values};
use super::gmap::WasmGMap;

use crate::binding_common::explore::{SharedLoop, SharedProfile};
use crate::topology::StandardPayload;

use super::edge::WasmEdge;
use super::vertex::WasmVertex;

#[wasm_bindgen(js_name = Loop)]
#[derive(Clone)]
pub struct WasmLoop {
    inner: SharedLoop<StandardPayload>,
}

impl WasmLoop {
    pub(crate) fn from_inner(inner: SharedLoop<StandardPayload>) -> Self {
        Self { inner }
    }
}

entity_common!(WasmLoop);

#[wasm_bindgen]
impl WasmLoop {
    /// Returns traversal-order dart ids.
    pub fn darts(&self) -> Result<Vec<usize>, JsValue> {
        self.inner.darts().map_err(js_err)
    }

    /// Returns traversal-order edges.
    pub fn edges(&self) -> Result<Array, JsValue> {
        Ok(values(
            self.inner
                .edges()
                .map_err(js_err)?
                .into_iter()
                .map(|inner| WasmEdge::from_inner(inner).into()),
        ))
    }

    /// Returns traversal-order vertices.
    pub fn vertices(&self) -> Result<Array, JsValue> {
        Ok(values(
            self.inner
                .vertices()
                .map_err(js_err)?
                .into_iter()
                .map(|inner| WasmVertex::from_inner(inner).into()),
        ))
    }

    /// Returns the same loop in opposite orientation.
    pub fn reversed(&self) -> Result<WasmLoop, JsValue> {
        Ok(Self::from_inner(self.inner.reversed().map_err(js_err)?))
    }
}

/// Read-only profile handle preserving contextual orientation.
#[wasm_bindgen(js_name = Profile)]
#[derive(Clone)]
pub struct WasmProfile {
    inner: SharedProfile<StandardPayload>,
}

impl WasmProfile {
    pub(crate) fn from_inner(inner: SharedProfile<StandardPayload>) -> Self {
        Self { inner }
    }
}

entity_common!(WasmProfile);

#[wasm_bindgen]
impl WasmProfile {
    /// Reports whether this profile is closed.
    #[wasm_bindgen(getter, js_name = isClosed)]
    pub fn is_closed(&self) -> Result<bool, JsValue> {
        self.inner.is_closed().map_err(js_err)
    }

    /// Returns the first traversal vertex.
    #[wasm_bindgen(getter)]
    pub fn start(&self) -> Result<WasmVertex, JsValue> {
        Ok(WasmVertex::from_inner(self.inner.start().map_err(js_err)?))
    }

    /// Returns the final traversal vertex.
    #[wasm_bindgen(getter)]
    pub fn end(&self) -> Result<WasmVertex, JsValue> {
        Ok(WasmVertex::from_inner(self.inner.end().map_err(js_err)?))
    }

    /// Returns traversal-order dart ids.
    pub fn darts(&self) -> Result<Vec<usize>, JsValue> {
        self.inner.darts().map_err(js_err)
    }

    /// Returns traversal-order edges.
    pub fn edges(&self) -> Result<Array, JsValue> {
        Ok(values(
            self.inner
                .edges()
                .map_err(js_err)?
                .into_iter()
                .map(|inner| WasmEdge::from_inner(inner).into()),
        ))
    }

    /// Returns traversal-order vertices.
    pub fn vertices(&self) -> Result<Array, JsValue> {
        Ok(values(
            self.inner
                .vertices()
                .map_err(js_err)?
                .into_iter()
                .map(|inner| WasmVertex::from_inner(inner).into()),
        ))
    }

    /// Returns the same profile in opposite orientation.
    pub fn reversed(&self) -> Result<WasmProfile, JsValue> {
        Ok(Self::from_inner(self.inner.reversed().map_err(js_err)?))
    }
}
