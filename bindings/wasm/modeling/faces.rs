use wasm_bindgen::prelude::*;

use crate::geometry::Plane;
use crate::modeling;

use super::super::topology::WasmFace;
use super::common::{js_err, wasm_face};

/// Builds a rectangular face on the XY plane.
#[wasm_bindgen(js_name = rectangleFace)]
pub fn rectangle(x_size: f64, y_size: f64) -> Result<WasmFace, JsValue> {
    modeling::faces::rectangle(Plane::xy(), x_size, y_size)
        .map_err(js_err)
        .and_then(wasm_face)
}
