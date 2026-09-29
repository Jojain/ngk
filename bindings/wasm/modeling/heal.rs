use js_sys::Array;
use wasm_bindgen::prelude::*;

use crate::healing::{HealingOptions, HealingReport};
use crate::modeling;

use super::super::topology::WasmSolid;
use super::common::{js_err, wasm_solid};

#[wasm_bindgen(js_name = HealingOptions)]
#[derive(Clone)]
pub struct WasmHealingOptions {
    inner: HealingOptions,
}

#[wasm_bindgen]
impl WasmHealingOptions {
    #[wasm_bindgen(constructor)]
    pub fn new(
        remove_redundant_vertices: Option<bool>,
        remove_redundant_edges: Option<bool>,
        remove_seams: Option<bool>,
        remove_filled_inner_loops: Option<bool>,
        linear_tolerance: Option<f64>,
        angular_tolerance: Option<f64>,
        max_iterations: Option<usize>,
    ) -> Self {
        let mut inner = HealingOptions::default();
        if let Some(value) = remove_redundant_vertices {
            inner.remove_redundant_vertices = value;
        }
        if let Some(value) = remove_redundant_edges {
            inner.remove_redundant_edges = value;
        }
        if let Some(value) = remove_seams {
            inner.remove_seams = value;
        }
        if let Some(value) = remove_filled_inner_loops {
            inner.remove_filled_inner_loops = value;
        }
        if let Some(value) = linear_tolerance {
            inner.linear_tolerance = value;
        }
        if let Some(value) = angular_tolerance {
            inner.angular_tolerance = value;
        }
        if let Some(value) = max_iterations {
            inner.max_iterations = value;
        }
        Self { inner }
    }

    #[wasm_bindgen(js_name = seamsOnly)]
    pub fn seams_only() -> Self {
        Self {
            inner: HealingOptions::seams_only(),
        }
    }
}

#[wasm_bindgen(js_name = HealingReport)]
pub struct WasmHealingReport {
    inner: HealingReport,
}

#[wasm_bindgen]
impl WasmHealingReport {
    #[wasm_bindgen(getter)]
    pub fn changes(&self) -> usize {
        self.inner.changes()
    }
    #[wasm_bindgen(getter)]
    pub fn iterations(&self) -> usize {
        self.inner.iterations
    }
    #[wasm_bindgen(getter, unchecked_return_type = "string[]")]
    pub fn skipped(&self) -> Array {
        self.inner
            .skipped
            .iter()
            .map(|skip| JsValue::from_str(&format!("{:?}: {:?}", skip.cell, skip.reason)))
            .collect()
    }
}

#[wasm_bindgen(js_name = HealingResult)]
pub struct WasmHealingResult {
    solid: WasmSolid,
    report: HealingReport,
}

#[wasm_bindgen]
impl WasmHealingResult {
    #[wasm_bindgen(getter)]
    pub fn solid(&self) -> WasmSolid {
        self.solid.clone()
    }
    #[wasm_bindgen(getter)]
    pub fn report(&self) -> WasmHealingReport {
        WasmHealingReport {
            inner: self.report.clone(),
        }
    }
}

#[wasm_bindgen(js_name = healSolid)]
pub fn solid(
    shape: &WasmSolid,
    options: Option<WasmHealingOptions>,
) -> Result<WasmHealingResult, JsValue> {
    let shape = shape.isolated_shape()?;
    let healed = modeling::heal::solid(
        shape,
        options.map_or_else(HealingOptions::default, |value| value.inner),
    )
    .map_err(js_err)?;
    Ok(WasmHealingResult {
        solid: wasm_solid(healed.shape)?,
        report: healed.report,
    })
}
