//! Length, area, volume and centroidal inertia of topology entities.

use wasm_bindgen::prelude::*;

use crate::binding_common::measurement::{self, tensor};
use crate::measure::{Area, Length, LinearProperties, SurfaceProperties, Volume, VolumeProperties};

use super::geometry::{WasmPoint3, point};
use super::topology::{WasmEdge, WasmFace, WasmProfile, WasmSheet, WasmSolid};

fn js_err(error: impl ToString) -> JsValue {
    JsValue::from_str(&error.to_string())
}

#[wasm_bindgen(js_name = LinearProperties)]
pub struct WasmLinearProperties {
    inner: LinearProperties,
}

#[wasm_bindgen]
impl WasmLinearProperties {
    #[wasm_bindgen(getter)]
    pub fn length(&self) -> f64 {
        self.inner.length
    }
    #[wasm_bindgen(getter)]
    pub fn centroid(&self) -> WasmPoint3 {
        point(self.inner.centroid)
    }
    #[wasm_bindgen(getter)]
    pub fn inertia(&self) -> Vec<f64> {
        tensor::<Length>(self.inner.inertia).to_vec()
    }
}

#[wasm_bindgen(js_name = SurfaceProperties)]
pub struct WasmSurfaceProperties {
    inner: SurfaceProperties,
}

#[wasm_bindgen]
impl WasmSurfaceProperties {
    #[wasm_bindgen(getter)]
    pub fn area(&self) -> f64 {
        self.inner.area
    }
    #[wasm_bindgen(getter)]
    pub fn centroid(&self) -> WasmPoint3 {
        point(self.inner.centroid)
    }
    #[wasm_bindgen(getter)]
    pub fn inertia(&self) -> Vec<f64> {
        tensor::<Area>(self.inner.inertia).to_vec()
    }
}

#[wasm_bindgen(js_name = VolumeProperties)]
pub struct WasmVolumeProperties {
    inner: VolumeProperties,
}

#[wasm_bindgen]
impl WasmVolumeProperties {
    #[wasm_bindgen(getter)]
    pub fn volume(&self) -> f64 {
        self.inner.volume
    }
    #[wasm_bindgen(getter)]
    pub fn centroid(&self) -> WasmPoint3 {
        point(self.inner.centroid)
    }
    #[wasm_bindgen(getter)]
    pub fn inertia(&self) -> Vec<f64> {
        tensor::<Volume>(self.inner.inertia).to_vec()
    }
}

#[wasm_bindgen]
impl WasmEdge {
    #[wasm_bindgen(getter, js_name = linearProperties)]
    pub fn linear_properties(&self) -> Result<WasmLinearProperties, JsValue> {
        let inner = measurement::edge(&self.inner).map_err(js_err)?;
        Ok(WasmLinearProperties { inner })
    }
}

#[wasm_bindgen]
impl WasmProfile {
    #[wasm_bindgen(getter)]
    pub fn length(&self) -> Result<f64, JsValue> {
        measurement::profile_length(&self.inner).map_err(js_err)
    }

    #[wasm_bindgen(getter, js_name = linearProperties)]
    pub fn linear_properties(&self) -> Result<WasmLinearProperties, JsValue> {
        let inner = measurement::profile(&self.inner).map_err(js_err)?;
        Ok(WasmLinearProperties { inner })
    }
}

#[wasm_bindgen]
impl WasmFace {
    #[wasm_bindgen(getter)]
    pub fn area(&self) -> Result<f64, JsValue> {
        Ok(self.surface_properties()?.area())
    }

    #[wasm_bindgen(getter, js_name = surfaceProperties)]
    pub fn surface_properties(&self) -> Result<WasmSurfaceProperties, JsValue> {
        let inner = measurement::face(&self.inner).map_err(js_err)?;
        Ok(WasmSurfaceProperties { inner })
    }
}

#[wasm_bindgen]
impl WasmSheet {
    #[wasm_bindgen(getter)]
    pub fn area(&self) -> Result<f64, JsValue> {
        Ok(self.surface_properties()?.area())
    }

    #[wasm_bindgen(getter, js_name = surfaceProperties)]
    pub fn surface_properties(&self) -> Result<WasmSurfaceProperties, JsValue> {
        let inner = measurement::sheet(&self.inner).map_err(js_err)?;
        Ok(WasmSurfaceProperties { inner })
    }
}

#[wasm_bindgen]
impl WasmSolid {
    #[wasm_bindgen(getter)]
    pub fn area(&self) -> Result<f64, JsValue> {
        Ok(self.surface_properties()?.area())
    }

    #[wasm_bindgen(getter, js_name = surfaceProperties)]
    pub fn surface_properties(&self) -> Result<WasmSurfaceProperties, JsValue> {
        let inner = measurement::solid_surface(&self.inner).map_err(js_err)?;
        Ok(WasmSurfaceProperties { inner })
    }

    #[wasm_bindgen(getter)]
    pub fn volume(&self) -> Result<f64, JsValue> {
        Ok(self.volume_properties()?.volume())
    }

    #[wasm_bindgen(getter, js_name = volumeProperties)]
    pub fn volume_properties(&self) -> Result<WasmVolumeProperties, JsValue> {
        let inner = measurement::solid(&self.inner).map_err(js_err)?;
        Ok(WasmVolumeProperties { inner })
    }
}
