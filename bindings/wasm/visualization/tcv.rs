use wasm_bindgen::prelude::*;

use crate::modeling::solids;
use crate::viz::{ScriptResult, Style, VizHints};

fn js_err(error: impl ToString) -> JsValue {
    JsValue::from_str(&error.to_string())
}

/// Builds the visualization-only block scene used by the frontend experiment.
#[wasm_bindgen(js_name = blockScene)]
pub fn block_scene(x_size: f64, y_size: f64, z_size: f64) -> Result<JsValue, JsValue> {
    let shape = solids::block(x_size, y_size, z_size).map_err(js_err)?;

    let mut hints = VizHints::new();
    for (key, _) in shape.map().iter_faces() {
        hints.face(
            key,
            Style::default()
                .color("#5aa9e6")
                .opacity(0.78)
                .double_sided(true),
        );
    }

    let result = ScriptResult::from_gmap_with_hints(shape.map(), &hints);
    serde_wasm_bindgen::to_value(&result).map_err(js_err)
}
