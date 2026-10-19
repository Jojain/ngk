//! Length, area, volume, centroidal inertia and bounding boxes of topology
//! entities.

use js_sys::Array;
use wasm_bindgen::prelude::*;

use crate::binding_common::measurement::{self, BoxRequest, tensor};
use crate::geometry::{BBox, Bound};
use crate::measure::{Area, Length, LinearProperties, SurfaceProperties, Volume, VolumeProperties};

use super::geometry::{WasmFrame, WasmPoint3, WasmVector3, point, vector};
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

/// One end of a box's range along a frame axis, for picking a corner.
#[wasm_bindgen(js_name = Bound)]
#[derive(Clone, Copy)]
pub enum WasmBound {
    Min,
    Max,
}

impl From<WasmBound> for Bound {
    fn from(bound: WasmBound) -> Self {
        match bound {
            WasmBound::Min => Bound::Min,
            WasmBound::Max => Bound::Max,
        }
    }
}

/// A box given by a frame and a range along each of its axes.
#[wasm_bindgen(js_name = BoundingBox)]
pub struct WasmBoundingBox {
    inner: BBox,
}

#[wasm_bindgen]
impl WasmBoundingBox {
    #[wasm_bindgen(getter)]
    pub fn frame(&self) -> WasmFrame {
        WasmFrame {
            inner: self.inner.frame().clone(),
        }
    }
    #[wasm_bindgen(getter)]
    pub fn min(&self) -> WasmPoint3 {
        point(self.inner.min())
    }
    #[wasm_bindgen(getter)]
    pub fn max(&self) -> WasmPoint3 {
        point(self.inner.max())
    }
    #[wasm_bindgen(getter)]
    pub fn center(&self) -> WasmPoint3 {
        point(self.inner.center())
    }
    #[wasm_bindgen(getter)]
    pub fn size(&self) -> WasmVector3 {
        vector(self.inner.size())
    }
    #[wasm_bindgen(getter, js_name = xSize)]
    pub fn x_size(&self) -> f64 {
        self.inner.x_size()
    }
    #[wasm_bindgen(getter, js_name = ySize)]
    pub fn y_size(&self) -> f64 {
        self.inner.y_size()
    }
    #[wasm_bindgen(getter, js_name = zSize)]
    pub fn z_size(&self) -> f64 {
        self.inner.z_size()
    }
    #[wasm_bindgen(getter, js_name = diagonalLength)]
    pub fn diagonal_length(&self) -> f64 {
        self.inner.diagonal_length()
    }
    pub fn corner(&self, x: WasmBound, y: WasmBound, z: WasmBound) -> WasmPoint3 {
        point(self.inner.corner(x.into(), y.into(), z.into()))
    }
    #[wasm_bindgen(unchecked_return_type = "Point3[]")]
    pub fn corners(&self) -> Array {
        self.inner
            .corners()
            .into_iter()
            .map(|corner| JsValue::from(point(corner)))
            .collect()
    }
}

fn bounding_box(value: Result<BBox, String>) -> Result<WasmBoundingBox, JsValue> {
    value.map(|inner| WasmBoundingBox { inner }).map_err(js_err)
}

fn aligned(frame: Option<&WasmFrame>) -> BoxRequest<'_> {
    BoxRequest::Aligned(frame.map(|frame| &frame.inner))
}

#[wasm_bindgen]
impl WasmEdge {
    #[wasm_bindgen(getter, js_name = linearProperties)]
    pub fn linear_properties(&self) -> Result<WasmLinearProperties, JsValue> {
        let inner = measurement::edge(&self.inner).map_err(js_err)?;
        Ok(WasmLinearProperties { inner })
    }

    #[wasm_bindgen(js_name = boundingBox)]
    pub fn bounding_box(&self, frame: Option<WasmFrame>) -> Result<WasmBoundingBox, JsValue> {
        bounding_box(measurement::edge_box(&self.inner, aligned(frame.as_ref())))
    }

    #[wasm_bindgen(js_name = orientedBoundingBox)]
    pub fn oriented_bounding_box(&self) -> Result<WasmBoundingBox, JsValue> {
        bounding_box(measurement::edge_box(&self.inner, BoxRequest::Oriented))
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

    #[wasm_bindgen(js_name = boundingBox)]
    pub fn bounding_box(&self, frame: Option<WasmFrame>) -> Result<WasmBoundingBox, JsValue> {
        bounding_box(measurement::profile_box(
            &self.inner,
            aligned(frame.as_ref()),
        ))
    }

    #[wasm_bindgen(js_name = orientedBoundingBox)]
    pub fn oriented_bounding_box(&self) -> Result<WasmBoundingBox, JsValue> {
        bounding_box(measurement::profile_box(&self.inner, BoxRequest::Oriented))
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

    #[wasm_bindgen(js_name = boundingBox)]
    pub fn bounding_box(&self, frame: Option<WasmFrame>) -> Result<WasmBoundingBox, JsValue> {
        bounding_box(measurement::face_box(&self.inner, aligned(frame.as_ref())))
    }

    #[wasm_bindgen(js_name = orientedBoundingBox)]
    pub fn oriented_bounding_box(&self) -> Result<WasmBoundingBox, JsValue> {
        bounding_box(measurement::face_box(&self.inner, BoxRequest::Oriented))
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

    #[wasm_bindgen(js_name = boundingBox)]
    pub fn bounding_box(&self, frame: Option<WasmFrame>) -> Result<WasmBoundingBox, JsValue> {
        bounding_box(measurement::sheet_box(&self.inner, aligned(frame.as_ref())))
    }

    #[wasm_bindgen(js_name = orientedBoundingBox)]
    pub fn oriented_bounding_box(&self) -> Result<WasmBoundingBox, JsValue> {
        bounding_box(measurement::sheet_box(&self.inner, BoxRequest::Oriented))
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

    #[wasm_bindgen(js_name = boundingBox)]
    pub fn bounding_box(&self, frame: Option<WasmFrame>) -> Result<WasmBoundingBox, JsValue> {
        bounding_box(measurement::solid_box(&self.inner, aligned(frame.as_ref())))
    }

    #[wasm_bindgen(js_name = orientedBoundingBox)]
    pub fn oriented_bounding_box(&self) -> Result<WasmBoundingBox, JsValue> {
        bounding_box(measurement::solid_box(&self.inner, BoxRequest::Oriented))
    }
}
