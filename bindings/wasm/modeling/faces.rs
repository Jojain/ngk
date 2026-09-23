use wasm_bindgen::prelude::*;

use crate::geometry::{Plane, Point3};
use crate::modeling;

use super::super::geometry::WasmPoint3;
use super::super::topology::{WasmFace, WasmProfile};
use super::common::{js_err, wasm_face};

/// Builds a rectangular face on the XY plane.
#[wasm_bindgen(js_name = rectangleFace)]
pub fn rectangle(x_size: f64, y_size: f64) -> Result<WasmFace, JsValue> {
    modeling::faces::rectangle(Plane::xy(), x_size, y_size)
        .map_err(js_err)
        .and_then(wasm_face)
}

/// Builds a planar face from ordered three-dimensional polygon corners.
#[wasm_bindgen(js_name = polygonFace)]
pub fn polygon(points: Vec<WasmPoint3>) -> Result<WasmFace, JsValue> {
    let points: Vec<Point3> = points.into_iter().map(|point| point.inner).collect();
    modeling::faces::polygon(&points)
        .map_err(js_err)
        .and_then(wasm_face)
}

/// Builds a planar face bounded by an existing profile's loop.
#[wasm_bindgen(js_name = faceFromProfile)]
pub fn from_profile(profile: &WasmProfile) -> Result<WasmFace, JsValue> {
    let shape = profile.isolated_shape()?;
    modeling::faces::from_profile(&shape)
        .map_err(js_err)
        .and_then(wasm_face)
}
