//! STEP text and binary STL exchange for browser callers.

use js_sys::Array;
use wasm_bindgen::prelude::*;

use crate::binding_common::explore::SharedModel;
use crate::exchange::step::part21::exchange_to_string;
use crate::exchange::step::{StepReadOptions, StepWriteOptions, map_to_exchange, read_step};

use super::topology::WasmSolid;

/// Exports a closed solid as binary STL in model coordinates.
#[wasm_bindgen(js_name = stlToBytes)]
pub fn stl_to_bytes(solid: &WasmSolid, segments: Option<u32>) -> Result<Vec<u8>, JsValue> {
    use crate::exchange::stl::{StlWriteOptions, stl_to_bytes as export_bytes};
    use crate::tessellate::{CurveOpts, SurfaceOpts, TessellateOpts};

    let segments = segments.unwrap_or(64) as usize;
    let options = StlWriteOptions {
        tessellation: TessellateOpts {
            curve: CurveOpts { segments },
            surface: SurfaceOpts {
                nu: segments,
                nv: segments / 2,
            },
        },
        ..Default::default()
    };
    let model = solid.inner.model();
    let view = model
        .model()
        .solid(solid.inner.key())
        .ok_or_else(|| js_err("missing solid"))?;
    export_bytes(&view, &options).map_err(js_err)
}

fn js_err(error: impl ToString) -> JsValue {
    JsValue::from_str(&error.to_string())
}

/// Writes one solid as STEP text.
#[wasm_bindgen(js_name = stepToString)]
pub fn step_to_string(solid: &WasmSolid, name: Option<String>) -> Result<String, JsValue> {
    let options = name.map(StepWriteOptions::named).unwrap_or_default();
    let model = solid.inner.model();
    let exchange =
        map_to_exchange(model.model(), &[solid.inner.key()], &options).map_err(js_err)?;
    exchange_to_string(&exchange).map_err(js_err)
}

/// One STEP import's solids and the entities skipped during recovery.
#[wasm_bindgen(js_name = StepImport)]
pub struct WasmStepImport {
    solids: Vec<WasmSolid>,
    skipped: Vec<String>,
}

#[wasm_bindgen]
impl WasmStepImport {
    #[wasm_bindgen(getter, unchecked_return_type = "Solid[]")]
    pub fn solids(&self) -> Array {
        self.solids.iter().cloned().map(JsValue::from).collect()
    }

    #[wasm_bindgen(getter, unchecked_return_type = "string[]")]
    pub fn skipped(&self) -> Array {
        self.skipped
            .iter()
            .map(|value| JsValue::from_str(value))
            .collect()
    }
}

/// Reads STEP text. Strict mode rejects any skipped entity.
#[wasm_bindgen(js_name = stepFromString)]
pub fn step_from_string(text: &str, strict: Option<bool>) -> Result<WasmStepImport, JsValue> {
    let options = if strict.unwrap_or(false) {
        StepReadOptions::strict()
    } else {
        StepReadOptions::default()
    };
    let import = read_step(text, &options).map_err(js_err)?;
    let solids = import
        .shapes
        .into_iter()
        .map(|shape| {
            let (model, key) = shape.into_model();
            let model = SharedModel::from_model(model);
            model
                .solid_by_key(key)
                .map(WasmSolid::from_inner)
                .ok_or_else(|| js_err(format!("missing solid {key:?}")))
        })
        .collect::<Result<Vec<_>, _>>()?;
    let skipped = import
        .report
        .skipped
        .iter()
        .map(|skip| {
            let entity = skip
                .entity
                .map(|key| key.to_string())
                .unwrap_or_else(|| "the document".to_string());
            format!("{} on line {}: {:?}", entity, skip.line, skip.reason)
        })
        .collect();
    Ok(WasmStepImport { solids, skipped })
}
