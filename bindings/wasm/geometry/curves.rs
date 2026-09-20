use wasm_bindgen::prelude::*;

use crate::geometry::parameter::NativeParam;
use crate::geometry::{Circle, Ellipse, Helix, Line};

use super::surfaces::WasmPlane;
use super::values::WasmFrame;
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
        point(self.inner.point_at(NativeParam::new(1.0)))
    }

    /// Evaluates the line.
    #[wasm_bindgen(js_name = pointAt)]
    pub fn point_at(&self, parameter: f64) -> WasmPoint3 {
        point(self.inner.point_at(NativeParam::new(parameter)))
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
        point(self.inner.point_at(NativeParam::new(parameter)))
    }
}

/// Analytical ellipse curve.
#[wasm_bindgen(js_name = Ellipse)]
pub struct WasmEllipse {
    pub(crate) inner: Ellipse,
}

#[wasm_bindgen]
impl WasmEllipse {
    #[wasm_bindgen(getter, js_name = majorRadius)]
    pub fn major_radius(&self) -> f64 {
        self.inner.major_radius()
    }

    #[wasm_bindgen(getter, js_name = minorRadius)]
    pub fn minor_radius(&self) -> f64 {
        self.inner.minor_radius()
    }

    #[wasm_bindgen(js_name = pointAt)]
    pub fn point_at(&self, parameter: f64) -> WasmPoint3 {
        point(self.inner.point_at(NativeParam::new(parameter)))
    }
}

/// Analytical helical curve.
#[wasm_bindgen(js_name = Helix)]
pub struct WasmHelix {
    pub(crate) inner: Helix,
}

#[wasm_bindgen]
impl WasmHelix {
    /// Creates a helix around the frame's z axis.
    #[wasm_bindgen(constructor)]
    pub fn new(frame: &WasmFrame, radius: f64, pitch: f64) -> Self {
        Self {
            inner: Helix::new(frame.inner.clone(), radius, pitch),
        }
    }

    #[wasm_bindgen(getter)]
    pub fn radius(&self) -> f64 {
        self.inner.radius()
    }

    #[wasm_bindgen(getter)]
    pub fn pitch(&self) -> f64 {
        self.inner.pitch()
    }

    #[wasm_bindgen(js_name = pointAt)]
    pub fn point_at(&self, parameter: f64) -> WasmPoint3 {
        point(self.inner.point_at(NativeParam::new(parameter)))
    }
}
