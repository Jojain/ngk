use wasm_bindgen::prelude::*;

use crate::{
    binding_common::explore::SharedGMap,
    geometry::{Plane, Point3},
    modeling::{edges, faces, profiles, solids},
};

use super::super::topology::{WasmEdge, WasmFace, WasmProfile, WasmSolid};

fn js_err(e: impl ToString) -> JsValue {
    JsValue::from_str(&e.to_string())
}

/// Builds an axis-aligned block and returns its read-only solid handle.
#[wasm_bindgen(js_name = block)]
pub fn block(x_size: f64, y_size: f64, z_size: f64) -> Result<WasmSolid, JsValue> {
    let shape = solids::block(x_size, y_size, z_size).map_err(js_err)?;
    let (map, key) = shape.into_map();
    let map = SharedGMap::from_map(map);
    let inner = map
        .solid_by_key(key)
        .ok_or_else(|| js_err(format!("missing solid {key:?}")))?;
    Ok(WasmSolid::from_inner(inner))
}

/// Builds a straight edge from two three-component point arrays.
#[wasm_bindgen(js_name = line)]
pub fn line(start: &[f64], end: &[f64]) -> Result<WasmEdge, JsValue> {
    if start.len() != 3 || end.len() != 3 {
        return Err(js_err(
            "line endpoints must each contain exactly three coordinates",
        ));
    }
    let shape = edges::line(
        Point3::new(start[0], start[1], start[2]),
        Point3::new(end[0], end[1], end[2]),
    )
    .map_err(js_err)?;
    let (map, key) = shape.into_map();
    let map = SharedGMap::from_map(map);
    let inner = map
        .edge_by_key(key)
        .ok_or_else(|| js_err(format!("missing edge {key:?}")))?;
    Ok(WasmEdge::from_inner(inner))
}

/// Builds a rectangular profile on the XY plane.
#[wasm_bindgen(js_name = rectangleProfile)]
pub fn rectangle_profile(x_size: f64, y_size: f64) -> Result<WasmProfile, JsValue> {
    let shape = profiles::rectangle(Plane::xy(), x_size, y_size).map_err(js_err)?;
    let (map, key) = shape.into_map();
    let map = SharedGMap::from_map(map);
    let inner = map
        .profile_by_key(key)
        .ok_or_else(|| js_err(format!("missing profile {key:?}")))?;
    Ok(WasmProfile::from_inner(inner))
}

/// Builds a rectangular face on the XY plane.
#[wasm_bindgen(js_name = rectangleFace)]
pub fn rectangle_face(x_size: f64, y_size: f64) -> Result<WasmFace, JsValue> {
    let shape = faces::rectangle(Plane::xy(), x_size, y_size).map_err(js_err)?;
    let (map, key) = shape.into_map();
    let map = SharedGMap::from_map(map);
    let inner = map
        .face_by_key(key)
        .ok_or_else(|| js_err(format!("missing face {key:?}")))?;
    Ok(WasmFace::from_inner(inner))
}
