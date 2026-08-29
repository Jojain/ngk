use js_sys::Array;
use wasm_bindgen::prelude::*;

use super::common::{entity_common, js_err, values};
use super::gmap::WasmGMap;

use crate::binding_common::explore::SharedVertex;
use crate::topology::StandardPayload;

use super::super::geometry::{WasmPoint3, point};
use super::edge::WasmEdge;
use super::face::WasmFace;
use super::sheet::WasmSheet;

#[wasm_bindgen(js_name = Vertex)]
#[derive(Clone)]
pub struct WasmVertex {
    inner: SharedVertex<StandardPayload>,
}

impl WasmVertex {
    pub(crate) fn from_inner(inner: SharedVertex<StandardPayload>) -> Self {
        Self { inner }
    }
}

entity_common!(WasmVertex);

#[wasm_bindgen]
impl WasmVertex {
    /// Returns stored point geometry when available.
    #[wasm_bindgen(getter)]
    pub fn point(&self) -> Result<Option<WasmPoint3>, JsValue> {
        Ok(self.inner.point().map_err(js_err)?.map(point))
    }

    /// Returns incident edges.
    #[wasm_bindgen(unchecked_return_type = "Edge[]")]
    pub fn edges(&self) -> Result<Array, JsValue> {
        Ok(values(
            self.inner
                .edges()
                .map_err(js_err)?
                .into_iter()
                .map(|inner| WasmEdge::from_inner(inner).into()),
        ))
    }

    /// Returns incident faces.
    #[wasm_bindgen(unchecked_return_type = "Face[]")]
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
    #[wasm_bindgen(unchecked_return_type = "Sheet[]")]
    pub fn sheets(&self) -> Result<Array, JsValue> {
        Ok(values(
            self.inner
                .sheets()
                .map_err(js_err)?
                .into_iter()
                .map(|inner| WasmSheet::from_inner(inner).into()),
        ))
    }
}
