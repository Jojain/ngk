use wasm_bindgen::prelude::*;

use crate::geometry::Plane;
use crate::modeling;

use super::super::topology::WasmProfile;
use super::common::{js_err, wasm_profile};

/// Builds a rectangular profile on the XY plane.
#[wasm_bindgen(js_name = rectangleProfile)]
pub fn rectangle(x_size: f64, y_size: f64) -> Result<WasmProfile, JsValue> {
    modeling::profiles::rectangle(Plane::xy(), x_size, y_size)
        .map_err(js_err)
        .and_then(wasm_profile)
}
