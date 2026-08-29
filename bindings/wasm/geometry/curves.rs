use wasm_bindgen::prelude::*;

use crate::geometry::{Circle, Line};

use super::surfaces::WasmPlane;
use super::values::{WasmPoint3, point};

#[wasm_bindgen(js_name = Line)]
pub struct WasmLine {
    pub(crate) inner: Line,
}

#[wasm_bindgen]
impl WasmLine {
    /// Returns the start point.
    #[wasm_bindgen(getter)]
    pub fn start(&self) -> WasmPoint3 {
        point(self.inner.origin())
    }

    /// Returns the end point.
    #[wasm_bindgen(getter)]
    pub fn end(&self) -> WasmPoint3 {
        point(self.inner.point_at(1.0))
    }

    /// Evaluates the line.
    #[wasm_bindgen(js_name = pointAt)]
    pub fn point_at(&self, parameter: f64) -> WasmPoint3 {
        point(self.inner.point_at(parameter))
    }
}

/// Analytical circle curve.
#[wasm_bindgen(js_name = Circle)]
pub struct WasmCircle {
    pub(crate) inner: Circle,
}

#[wasm_bindgen]
impl WasmCircle {
    /// Returns the support plane.
    #[wasm_bindgen(getter)]
    pub fn plane(&self) -> WasmPlane {
        WasmPlane {
            inner: self.inner.plane().clone(),
        }
    }

    /// Returns the circle radius.
    #[wasm_bindgen(getter)]
    pub fn radius(&self) -> f64 {
        self.inner.radius()
    }

    /// Evaluates the circle.
    #[wasm_bindgen(js_name = pointAt)]
    pub fn point_at(&self, parameter: f64) -> WasmPoint3 {
        point(self.inner.point_at(parameter))
    }
}
