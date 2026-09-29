//! Length, area, volume and centroidal inertia of topology entities.

use wasm_bindgen::prelude::*;

use crate::binding_common::measurement::{self, MeasuredProperties};

use super::geometry::{WasmPoint3, point};
use super::topology::{WasmEdge, WasmFace, WasmProfile, WasmSheet, WasmSolid};

fn js_err(error: impl ToString) -> JsValue {
    JsValue::from_str(&error.to_string())
}

#[wasm_bindgen(js_name = MeasuredProperties)]
pub struct WasmMeasuredProperties {
    inner: MeasuredProperties,
}

#[wasm_bindgen]
impl WasmMeasuredProperties {
    #[wasm_bindgen(getter)]
    pub fn amount(&self) -> f64 {
        self.inner.amount
    }
    #[wasm_bindgen(getter)]
    pub fn centroid(&self) -> WasmPoint3 {
        point(self.inner.centroid)
    }
    #[wasm_bindgen(getter)]
    pub fn inertia(&self) -> Vec<f64> {
        self.inner.inertia.to_vec()
    }
}

fn wrap(value: Result<MeasuredProperties, String>) -> Result<WasmMeasuredProperties, JsValue> {
    value
        .map(|inner| WasmMeasuredProperties { inner })
        .map_err(js_err)
}

#[wasm_bindgen(js_name = edgeProperties)]
pub fn edge_properties(value: &WasmEdge) -> Result<WasmMeasuredProperties, JsValue> {
    wrap(measurement::edge(&value.inner))
}
#[wasm_bindgen(js_name = profileProperties)]
pub fn profile_properties(value: &WasmProfile) -> Result<WasmMeasuredProperties, JsValue> {
    wrap(measurement::profile(&value.inner))
}
#[wasm_bindgen(js_name = faceProperties)]
pub fn face_properties(value: &WasmFace) -> Result<WasmMeasuredProperties, JsValue> {
    wrap(measurement::face(&value.inner))
}
#[wasm_bindgen(js_name = sheetProperties)]
pub fn sheet_properties(value: &WasmSheet) -> Result<WasmMeasuredProperties, JsValue> {
    wrap(measurement::sheet(&value.inner))
}
#[wasm_bindgen(js_name = solidProperties)]
pub fn solid_properties(value: &WasmSolid) -> Result<WasmMeasuredProperties, JsValue> {
    wrap(measurement::solid(&value.inner))
}
