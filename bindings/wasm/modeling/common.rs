use wasm_bindgen::prelude::*;

use crate::StandardPayload;
use crate::binding_common::explore::SharedGMap;
use crate::topology::shape::{EdgeTag, FaceTag, ProfileTag, Shape, SolidTag};

use super::super::topology::{WasmEdge, WasmFace, WasmProfile, WasmSolid};

pub(super) fn js_err(error: impl ToString) -> JsValue {
    JsValue::from_str(&error.to_string())
}

pub(super) fn wasm_edge(shape: Shape<EdgeTag, StandardPayload>) -> Result<WasmEdge, JsValue> {
    let (map, key) = shape.into_map();
    let map = SharedGMap::from_map(map);
    let inner = map
        .edge_by_key(key)
        .ok_or_else(|| js_err(format!("missing edge {key:?}")))?;
    Ok(WasmEdge::from_inner(inner))
}

pub(super) fn wasm_profile(
    shape: Shape<ProfileTag, StandardPayload>,
) -> Result<WasmProfile, JsValue> {
    let (map, key) = shape.into_map();
    let map = SharedGMap::from_map(map);
    let inner = map
        .profile_by_key(key)
        .ok_or_else(|| js_err(format!("missing profile {key:?}")))?;
    Ok(WasmProfile::from_inner(inner))
}

pub(super) fn wasm_face(shape: Shape<FaceTag, StandardPayload>) -> Result<WasmFace, JsValue> {
    let (map, key) = shape.into_map();
    let map = SharedGMap::from_map(map);
    let inner = map
        .face_by_key(key)
        .ok_or_else(|| js_err(format!("missing face {key:?}")))?;
    Ok(WasmFace::from_inner(inner))
}

pub(super) fn wasm_solid(shape: Shape<SolidTag, StandardPayload>) -> Result<WasmSolid, JsValue> {
    let (map, key) = shape.into_map();
    let map = SharedGMap::from_map(map);
    let inner = map
        .solid_by_key(key)
        .ok_or_else(|| js_err(format!("missing solid {key:?}")))?;
    Ok(WasmSolid::from_inner(inner))
}
