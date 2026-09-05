use js_sys::Array;
use js_sys::{Object, Reflect};
use wasm_bindgen::prelude::*;

use super::common::{entity_common, js_err, values};
use super::gmap::WasmGMap;

use crate::binding_common::explore::SharedFace;
use crate::topology::StandardPayload;

use super::super::geometry::{curve2_to_js, surface_to_js};
use super::edge::WasmEdge;
use super::profile::WasmLoop;
use super::vertex::WasmVertex;

#[wasm_bindgen(js_name = Face)]
#[derive(Clone)]
pub struct WasmFace {
    inner: SharedFace<StandardPayload>,
}

impl WasmFace {
    pub(crate) fn from_inner(inner: SharedFace<StandardPayload>) -> Self {
        Self { inner }
    }
}

entity_common!(WasmFace);

#[wasm_bindgen]
impl WasmFace {
    /// Returns the support surface as its concrete geometry class.
    #[wasm_bindgen(getter)]
    pub fn surface(&self) -> Result<JsValue, JsValue> {
        Ok(surface_to_js(self.inner.surface().map_err(js_err)?))
    }

    /// Returns the outer loop.
    #[wasm_bindgen(getter, js_name = outerLoop)]
    pub fn outer_loop(&self) -> Result<WasmLoop, JsValue> {
        Ok(WasmLoop::from_inner(
            self.inner.outer_loop().map_err(js_err)?,
        ))
    }

    /// Returns inner loops.
    #[wasm_bindgen(js_name = innerLoops)]
    #[wasm_bindgen(unchecked_return_type = "Loop[]")]
    pub fn inner_loops(&self) -> Result<Array, JsValue> {
        Ok(values(
            self.inner
                .inner_loops()
                .map_err(js_err)?
                .into_iter()
                .map(|inner| WasmLoop::from_inner(inner).into()),
        ))
    }

    /// Returns all loops, outer first.
    #[wasm_bindgen(unchecked_return_type = "Loop[]")]
    pub fn loops(&self) -> Result<Array, JsValue> {
        Ok(values(
            self.inner
                .loops()
                .map_err(js_err)?
                .into_iter()
                .map(|inner| WasmLoop::from_inner(inner).into()),
        ))
    }

    /// Returns boundary edges in loop order.
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

    /// Returns boundary vertices in loop order.
    #[wasm_bindgen(unchecked_return_type = "Vertex[]")]
    pub fn vertices(&self) -> Result<Array, JsValue> {
        Ok(values(
            self.inner
                .vertices()
                .map_err(js_err)?
                .into_iter()
                .map(|inner| WasmVertex::from_inner(inner).into()),
        ))
    }

    /// Returns the exact pcurves attached to the face boundary darts.
    #[wasm_bindgen(unchecked_return_type = "FacePcurve[]", js_name = pcurves)]
    pub fn pcurves(&self) -> Result<Array, JsValue> {
        let curves = Array::new();
        for (loop_index, boundary_loop) in self.inner.loops().map_err(js_err)?.into_iter().enumerate()
        {
            for edge in boundary_loop.edges().map_err(js_err)? {
                let Some(pcurve) = self.inner.pcurve(edge.dart_id()).map_err(js_err)? else {
                    continue;
                };
                let record = Object::new();
                Reflect::set(
                    record.as_ref(),
                    &JsValue::from_str("loopIndex"),
                    &JsValue::from_f64(loop_index as f64),
                )?;
                Reflect::set(
                    record.as_ref(),
                    &JsValue::from_str("dartId"),
                    &JsValue::from_f64(edge.dart_id() as f64),
                )?;
                Reflect::set(
                    record.as_ref(),
                    &JsValue::from_str("edgeKey"),
                    &JsValue::from_str(&format!("{:?}", edge.key())),
                )?;
                Reflect::set(
                    record.as_ref(),
                    &JsValue::from_str("startVertexKey"),
                    &JsValue::from_str(&format!("{:?}", edge.start().map_err(js_err)?.key())),
                )?;
                Reflect::set(
                    record.as_ref(),
                    &JsValue::from_str("endVertexKey"),
                    &JsValue::from_str(&format!("{:?}", edge.end().map_err(js_err)?.key())),
                )?;
                Reflect::set(
                    record.as_ref(),
                    &JsValue::from_str("curve"),
                    &curve2_to_js(pcurve),
                )?;
                curves.push(&record);
            }
        }
        Ok(curves)
    }

    /// Returns the same face in opposite orientation.
    pub fn reversed(&self) -> Result<WasmFace, JsValue> {
        Ok(Self::from_inner(self.inner.reversed().map_err(js_err)?))
    }
}
