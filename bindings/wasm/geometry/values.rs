use nalgebra::{UnitVector3, Vector3};
use wasm_bindgen::prelude::*;

use crate::geometry::Point3;

pub(crate) fn point(value: Point3) -> WasmPoint3 {
    WasmPoint3 { inner: value }
}

/// Converts a kernel vector to its JavaScript exploration value.
pub(crate) fn vector(value: Vector3<f64>) -> WasmVector3 {
    WasmVector3 { inner: value }
}

pub(crate) fn unit_vector(value: UnitVector3<f64>) -> WasmVector3 {
    vector(value.into_inner())
}

#[wasm_bindgen(js_name = Point3)]
#[derive(Clone)]
pub struct WasmPoint3 {
    pub(crate) inner: Point3,
}

#[wasm_bindgen]
impl WasmPoint3 {
    /// Returns the x coordinate.
    #[wasm_bindgen(getter)]
    pub fn x(&self) -> f64 {
        self.inner.x
    }

    /// Returns the y coordinate.
    #[wasm_bindgen(getter)]
    pub fn y(&self) -> f64 {
        self.inner.y
    }

    /// Returns the z coordinate.
    #[wasm_bindgen(getter)]
    pub fn z(&self) -> f64 {
        self.inner.z
    }

    /// Returns `[x, y, z]`.
    #[wasm_bindgen(js_name = toArray)]
    pub fn to_array(&self) -> Vec<f64> {
        vec![self.inner.x, self.inner.y, self.inner.z]
    }
}

/// Three-dimensional vector returned by JavaScript exploration methods.
#[wasm_bindgen(js_name = Vector3)]
#[derive(Clone)]
pub struct WasmVector3 {
    pub(crate) inner: Vector3<f64>,
}

#[wasm_bindgen]
impl WasmVector3 {
    /// Returns the x component.
    #[wasm_bindgen(getter)]
    pub fn x(&self) -> f64 {
        self.inner.x
    }

    /// Returns the y component.
    #[wasm_bindgen(getter)]
    pub fn y(&self) -> f64 {
        self.inner.y
    }

    /// Returns the z component.
    #[wasm_bindgen(getter)]
    pub fn z(&self) -> f64 {
        self.inner.z
    }

    /// Returns `[x, y, z]`.
    #[wasm_bindgen(js_name = toArray)]
    pub fn to_array(&self) -> Vec<f64> {
        vec![self.inner.x, self.inner.y, self.inner.z]
    }
}
