pub mod nurbs;

use wasm_bindgen::prelude::*;

use crate::scripts;
use nalgebra::Vector3;

#[wasm_bindgen(start)]
pub fn init() {
    console_error_panic_hook::set_once();
}

/// IDs of all scripts registered in [`crate::scripts`].
#[wasm_bindgen(js_name = listScripts)]
pub fn list_scripts() -> Vec<String> {
    scripts::list().into_iter().map(String::from).collect()
}

/// Runs a script by id and returns its [`crate::viz::VizScene`] as a plain JS
/// object (see `visualization/src/kernel/viz.ts` for the TS mirror).
#[wasm_bindgen(js_name = runScript)]
pub fn run_script(name: &str) -> Result<JsValue, JsValue> {
    let scene = scripts::run(name).map_err(|e| JsValue::from_str(&e))?;
    serde_wasm_bindgen::to_value(&scene).map_err(|e| JsValue::from_str(&e.to_string()))
}

/// Builds an extruded regular polygon scene from live UI parameters.
#[wasm_bindgen(js_name = extrudePolygon)]
pub fn extrude_polygon(
    point_count: usize,
    extrusion_x: f64,
    extrusion_y: f64,
    extrusion_z: f64,
) -> Result<JsValue, JsValue> {
    let result = scripts::interactive_extrusion::build(
        point_count,
        Vector3::new(extrusion_x, extrusion_y, extrusion_z),
    )
    .map_err(|e| JsValue::from_str(&e))?;
    serde_wasm_bindgen::to_value(&result).map_err(|e| JsValue::from_str(&e.to_string()))
}
