use wasm_bindgen::prelude::*;

#[wasm_bindgen(start)]
/// Installs the panic hook when the WASM module starts.
pub fn init() {
    console_error_panic_hook::set_once();
}
