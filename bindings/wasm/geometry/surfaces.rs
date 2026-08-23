use wasm_bindgen::prelude::*;

use crate::geometry::{Cylinder, Plane, RuledSurface, SurfaceOfRevolution};

use super::convert::curve_to_js;
use super::values::{WasmPoint3, WasmVector3, point, unit_vector, vector};

#[wasm_bindgen(js_name = Plane)]
pub struct WasmPlane {
    pub(crate) inner: Plane,
}

#[wasm_bindgen]
impl WasmPlane {
    /// Returns the plane origin.
    #[wasm_bindgen(getter)]
    pub fn origin(&self) -> WasmPoint3 {
        point(self.inner.origin())
    }

    /// Returns the local x direction.
    #[wasm_bindgen(getter, js_name = xDir)]
    pub fn x_dir(&self) -> WasmVector3 {
        unit_vector(self.inner.x_dir())
    }

    /// Returns the local y direction.
    #[wasm_bindgen(getter, js_name = yDir)]
    pub fn y_dir(&self) -> WasmVector3 {
        unit_vector(self.inner.y_dir())
    }

    /// Returns the support normal.
    #[wasm_bindgen(getter)]
    pub fn normal(&self) -> WasmVector3 {
        unit_vector(self.inner.normal())
    }

    /// Evaluates the plane.
    #[wasm_bindgen(js_name = pointAt)]
    pub fn point_at(&self, u: f64, v: f64) -> WasmPoint3 {
        point(self.inner.point_at(u, v))
    }

    /// Evaluates the plane normal.
    #[wasm_bindgen(js_name = normalAt)]
    pub fn normal_at(&self, _u: f64, _v: f64) -> WasmVector3 {
        unit_vector(self.inner.normal())
    }
}

/// Analytical cylinder surface.
#[wasm_bindgen(js_name = Cylinder)]
pub struct WasmCylinder {
    pub(crate) inner: Cylinder,
}

#[wasm_bindgen]
impl WasmCylinder {
    /// Returns the cylinder origin.
    #[wasm_bindgen(getter)]
    pub fn origin(&self) -> WasmPoint3 {
        point(self.inner.origin())
    }

    /// Returns the local x direction.
    #[wasm_bindgen(getter, js_name = xDir)]
    pub fn x_dir(&self) -> WasmVector3 {
        unit_vector(self.inner.x_dir())
    }

    /// Returns the cylinder axis.
    #[wasm_bindgen(getter)]
    pub fn axis(&self) -> WasmVector3 {
        unit_vector(self.inner.axis())
    }

    /// Returns the radius.
    #[wasm_bindgen(getter)]
    pub fn radius(&self) -> f64 {
        self.inner.radius
    }

    /// Evaluates the cylinder.
    #[wasm_bindgen(js_name = pointAt)]
    pub fn point_at(&self, u: f64, v: f64) -> WasmPoint3 {
        point(self.inner.point_at(u, v))
    }

    /// Evaluates the cylinder normal.
    #[wasm_bindgen(js_name = normalAt)]
    pub fn normal_at(&self, u: f64, v: f64) -> WasmVector3 {
        unit_vector(self.inner.normal_at(u, v))
    }
}

/// Ruled support surface.
#[wasm_bindgen(js_name = RuledSurface)]
pub struct WasmRuledSurface {
    pub(crate) inner: RuledSurface,
}

#[wasm_bindgen]
impl WasmRuledSurface {
    /// Returns the generating curve.
    #[wasm_bindgen(getter)]
    pub fn curve(&self) -> Result<JsValue, JsValue> {
        curve_to_js(self.inner.curve().clone())
    }

    /// Returns the ruling direction.
    #[wasm_bindgen(getter)]
    pub fn direction(&self) -> WasmVector3 {
        vector(self.inner.direction())
    }

    /// Evaluates the surface.
    #[wasm_bindgen(js_name = pointAt)]
    pub fn point_at(&self, u: f64, v: f64) -> WasmPoint3 {
        point(self.inner.point_at(u, v))
    }

    /// Evaluates the surface normal.
    #[wasm_bindgen(js_name = normalAt)]
    pub fn normal_at(&self, u: f64, v: f64) -> WasmVector3 {
        unit_vector(self.inner.normal_at(u, v))
    }
}

/// Surface produced by revolving a curve around an axis.
#[wasm_bindgen(js_name = SurfaceOfRevolution)]
pub struct WasmSurfaceOfRevolution {
    pub(crate) inner: SurfaceOfRevolution,
}

#[wasm_bindgen]
impl WasmSurfaceOfRevolution {
    /// Returns the generating curve.
    #[wasm_bindgen(getter)]
    pub fn curve(&self) -> Result<JsValue, JsValue> {
        curve_to_js(self.inner.curve().clone())
    }

    /// Returns the axis origin.
    #[wasm_bindgen(getter)]
    pub fn origin(&self) -> WasmPoint3 {
        point(self.inner.origin())
    }

    /// Returns the axis direction.
    #[wasm_bindgen(getter)]
    pub fn axis(&self) -> WasmVector3 {
        unit_vector(self.inner.axis.direction)
    }

    /// Evaluates the surface.
    #[wasm_bindgen(js_name = pointAt)]
    pub fn point_at(&self, u: f64, v: f64) -> WasmPoint3 {
        point(self.inner.point_at(u, v))
    }

    /// Evaluates the surface normal.
    #[wasm_bindgen(js_name = normalAt)]
    pub fn normal_at(&self, u: f64, v: f64) -> WasmVector3 {
        unit_vector(self.inner.normal_at(u, v))
    }
}
