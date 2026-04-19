pub mod nurbs;

use wasm_bindgen::prelude::*;

use crate::scripts;

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
