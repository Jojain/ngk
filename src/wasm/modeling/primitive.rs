use wasm_bindgen::prelude::*;

use crate::{
    modeling::primitives,
    viz::{ScriptResult, Style, VizHints},
};

fn js_err(e: impl ToString) -> JsValue {
    JsValue::from_str(&e.to_string())
}

/// Builds an axis-aligned block from the modeling primitives API.
#[wasm_bindgen(js_name = createBlock)]
pub fn create_block(x_size: f64, y_size: f64, z_size: f64) -> Result<JsValue, JsValue> {
    let shape = primitives::block(x_size, y_size, z_size).map_err(js_err)?;

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
