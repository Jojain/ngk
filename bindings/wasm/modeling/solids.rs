use wasm_bindgen::prelude::*;

use crate::builders::boolean::BooleanOperation;
use crate::modeling;

use super::super::geometry::WasmFrame;
use super::super::topology::WasmSolid;
use super::common::{js_err, wasm_solid};

/// Builds an axis-aligned block and returns its read-only solid handle.
#[wasm_bindgen(js_name = block)]
pub fn block(x_size: f64, y_size: f64, z_size: f64) -> Result<WasmSolid, JsValue> {
    modeling::solids::block(x_size, y_size, z_size)
        .map_err(js_err)
        .and_then(wasm_solid)
}

/// Builds a block at an explicitly supplied placement frame.
#[wasm_bindgen(js_name = blockAt)]
pub fn block_at(
    frame: &WasmFrame,
    x_size: f64,
    y_size: f64,
    z_size: f64,
) -> Result<WasmSolid, JsValue> {
    modeling::solids::block_at(frame.inner.clone(), x_size, y_size, z_size)
        .map_err(js_err)
        .and_then(wasm_solid)
}

/// Builds a cylinder and returns its read-only solid handle.
#[wasm_bindgen(js_name = cylinder)]
pub fn cylinder(radius: f64, height: f64) -> Result<WasmSolid, JsValue> {
    modeling::solids::cylinder(radius, height)
        .map_err(js_err)
        .and_then(wasm_solid)
}

/// Builds a cylinder at an explicitly supplied placement frame.
#[wasm_bindgen(js_name = cylinderAt)]
pub fn cylinder_at(frame: &WasmFrame, radius: f64, height: f64) -> Result<WasmSolid, JsValue> {
    modeling::solids::cylinder_at(frame.inner.clone(), radius, height)
        .map_err(js_err)
        .and_then(wasm_solid)
}

/// Builds a sphere and returns its read-only solid handle.
#[wasm_bindgen(js_name = sphere)]
pub fn sphere(radius: f64) -> Result<WasmSolid, JsValue> {
    modeling::solids::sphere(radius)
        .map_err(js_err)
        .and_then(wasm_solid)
}

/// Builds a sphere at an explicitly supplied placement frame.
#[wasm_bindgen(js_name = sphereAt)]
pub fn sphere_at(frame: &WasmFrame, radius: f64) -> Result<WasmSolid, JsValue> {
    modeling::solids::sphere_at(frame.inner.clone(), radius)
        .map_err(js_err)
        .and_then(wasm_solid)
}

/// Builds a torus and returns its read-only solid handle.
#[wasm_bindgen(js_name = torus)]
pub fn torus(major: f64, minor: f64) -> Result<WasmSolid, JsValue> {
    modeling::solids::torus(major, minor)
        .map_err(js_err)
        .and_then(wasm_solid)
}

/// Builds a torus at an explicitly supplied placement frame.
#[wasm_bindgen(js_name = torusAt)]
pub fn torus_at(frame: &WasmFrame, major: f64, minor: f64) -> Result<WasmSolid, JsValue> {
    modeling::solids::torus_at(frame.inner.clone(), major, minor)
        .map_err(js_err)
        .and_then(wasm_solid)
}

/// Fuses two solids into a new result without mutating either input.
#[wasm_bindgen(js_name = fuse)]
pub fn fuse(first: &WasmSolid, second: &WasmSolid) -> Result<WasmSolid, JsValue> {
    combine(first, second, BooleanOperation::Union)
}

/// Subtracts `tool` from `target` into a new result.
#[wasm_bindgen(js_name = cut)]
pub fn cut(target: &WasmSolid, tool: &WasmSolid) -> Result<WasmSolid, JsValue> {
    combine(target, tool, BooleanOperation::Difference)
}

/// Returns the common volume of two solids as a new result.
#[wasm_bindgen(js_name = intersect)]
pub fn intersect(first: &WasmSolid, second: &WasmSolid) -> Result<WasmSolid, JsValue> {
    combine(first, second, BooleanOperation::Intersection)
}

fn combine(
    first: &WasmSolid,
    second: &WasmSolid,
    operation: BooleanOperation,
) -> Result<WasmSolid, JsValue> {
    modeling::solids::combine_views(
        first.inner.view().map_err(js_err)?,
        second.inner.view().map_err(js_err)?,
        operation,
    )
    .map_err(js_err)
    .and_then(wasm_solid)
}
