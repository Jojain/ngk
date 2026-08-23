use js_sys::Array;
use wasm_bindgen::prelude::*;

use super::common::{entity_common, js_err, values};
use super::gmap::WasmGMap;

use crate::binding_common::explore::SharedEdge;
use crate::topology::StandardPayload;

use super::super::geometry::curve_to_js;
use super::face::WasmFace;
use super::sheet::WasmSheet;
use super::vertex::WasmVertex;

#[wasm_bindgen(js_name = Edge)]
#[derive(Clone)]
pub struct WasmEdge {
    inner: SharedEdge<StandardPayload>,
}

impl WasmEdge {
    pub(crate) fn from_inner(inner: SharedEdge<StandardPayload>) -> Self {
        Self { inner }
    }
}

entity_common!(WasmEdge);

#[wasm_bindgen]
impl WasmEdge {
    /// Returns the oriented start vertex.
    #[wasm_bindgen(getter)]
    pub fn start(&self) -> Result<WasmVertex, JsValue> {
        Ok(WasmVertex::from_inner(self.inner.start().map_err(js_err)?))
    }

    /// Returns the oriented end vertex.
    #[wasm_bindgen(getter)]
    pub fn end(&self) -> Result<WasmVertex, JsValue> {
        Ok(WasmVertex::from_inner(self.inner.end().map_err(js_err)?))
    }

    /// Returns geometric length when available.
    #[wasm_bindgen(getter)]
    pub fn length(&self) -> Result<Option<f64>, JsValue> {
        self.inner.length().map_err(js_err)
    }

    /// Returns the attached curve as its concrete geometry class.
    #[wasm_bindgen(getter)]
    pub fn curve(&self) -> Result<JsValue, JsValue> {
        match self.inner.curve().map_err(js_err)? {
            Some(curve) => curve_to_js(curve),
            None => Ok(JsValue::NULL),
        }
    }

    /// Returns all darts in this edge cell.
    pub fn darts(&self) -> Result<Vec<usize>, JsValue> {
        self.inner.darts().map_err(js_err)
    }

    /// Returns incident vertices.
    pub fn vertices(&self) -> Result<Array, JsValue> {
        Ok(values(
            self.inner
                .vertices()
                .map_err(js_err)?
                .into_iter()
                .map(|inner| WasmVertex::from_inner(inner).into()),
        ))
    }

    /// Returns incident faces.
    pub fn faces(&self) -> Result<Array, JsValue> {
        Ok(values(
            self.inner
                .faces()
                .map_err(js_err)?
                .into_iter()
                .map(|inner| WasmFace::from_inner(inner).into()),
        ))
    }

    /// Returns incident sheets.
    pub fn sheets(&self) -> Result<Array, JsValue> {
        Ok(values(
            self.inner
                .sheets()
                .map_err(js_err)?
                .into_iter()
                .map(|inner| WasmSheet::from_inner(inner).into()),
        ))
    }

    /// Returns the same edge in opposite orientation.
    pub fn reversed(&self) -> Result<WasmEdge, JsValue> {
        Ok(Self::from_inner(self.inner.reversed().map_err(js_err)?))
    }
}
