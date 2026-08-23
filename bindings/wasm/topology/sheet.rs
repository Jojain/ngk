use js_sys::Array;
use wasm_bindgen::prelude::*;

use super::common::{entity_common, js_err, values};
use super::gmap::WasmGMap;

use crate::binding_common::explore::{SharedSheet, SharedShell};
use crate::topology::StandardPayload;

use super::edge::WasmEdge;
use super::face::WasmFace;
use super::vertex::WasmVertex;

#[wasm_bindgen(js_name = Shell)]
#[derive(Clone)]
pub struct WasmShell {
    inner: SharedShell<StandardPayload>,
}

impl WasmShell {
    pub(crate) fn from_inner(inner: SharedShell<StandardPayload>) -> Self {
        Self { inner }
    }
}

entity_common!(WasmShell);

#[wasm_bindgen]
impl WasmShell {
    /// Returns faces belonging to this shell.
    pub fn faces(&self) -> Result<Array, JsValue> {
        Ok(values(
            self.inner
                .faces()
                .map_err(js_err)?
                .into_iter()
                .map(|inner| WasmFace::from_inner(inner).into()),
        ))
    }

    /// Returns edges belonging to this shell.
    pub fn edges(&self) -> Result<Array, JsValue> {
        Ok(values(
            self.inner
                .edges()
                .map_err(js_err)?
                .into_iter()
                .map(|inner| WasmEdge::from_inner(inner).into()),
        ))
    }

    /// Returns vertices belonging to this shell.
    pub fn vertices(&self) -> Result<Array, JsValue> {
        Ok(values(
            self.inner
                .vertices()
                .map_err(js_err)?
                .into_iter()
                .map(|inner| WasmVertex::from_inner(inner).into()),
        ))
    }

    /// Returns the same shell in opposite orientation.
    pub fn reversed(&self) -> Result<WasmShell, JsValue> {
        Ok(Self::from_inner(self.inner.reversed().map_err(js_err)?))
    }
}

/// Read-only sheet handle.
#[wasm_bindgen(js_name = Sheet)]
#[derive(Clone)]
pub struct WasmSheet {
    inner: SharedSheet<StandardPayload>,
}

impl WasmSheet {
    pub(crate) fn from_inner(inner: SharedSheet<StandardPayload>) -> Self {
        Self { inner }
    }
}

entity_common!(WasmSheet);

#[wasm_bindgen]
impl WasmSheet {
    /// Reports whether this sheet is closed.
    #[wasm_bindgen(getter, js_name = isClosed)]
    pub fn is_closed(&self) -> Result<bool, JsValue> {
        self.inner.is_closed().map_err(js_err)
    }

    /// Returns all sheet darts.
    pub fn darts(&self) -> Result<Vec<usize>, JsValue> {
        self.inner.darts().map_err(js_err)
    }

    /// Returns faces belonging to this sheet.
    pub fn faces(&self) -> Result<Array, JsValue> {
        Ok(values(
            self.inner
                .faces()
                .map_err(js_err)?
                .into_iter()
                .map(|inner| WasmFace::from_inner(inner).into()),
        ))
    }

    /// Returns edges belonging to this sheet.
    pub fn edges(&self) -> Result<Array, JsValue> {
        Ok(values(
            self.inner
                .edges()
                .map_err(js_err)?
                .into_iter()
                .map(|inner| WasmEdge::from_inner(inner).into()),
        ))
    }

    /// Returns vertices belonging to this sheet.
    pub fn vertices(&self) -> Result<Array, JsValue> {
        Ok(values(
            self.inner
                .vertices()
                .map_err(js_err)?
                .into_iter()
                .map(|inner| WasmVertex::from_inner(inner).into()),
        ))
    }

    /// Returns the same sheet in opposite orientation.
    pub fn reversed(&self) -> Result<WasmSheet, JsValue> {
        Ok(Self::from_inner(self.inner.reversed().map_err(js_err)?))
    }
}
