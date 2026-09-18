use wasm_bindgen::prelude::*;

use crate::StandardPayload;
use crate::binding_common::explore::SharedModel;
use crate::topology::shape::{EdgeTag, FaceTag, ProfileTag, Shape, SheetTag, SolidTag};

use super::super::topology::{WasmEdge, WasmFace, WasmProfile, WasmSheet, WasmSolid};

pub(super) fn js_err(error: impl ToString) -> JsValue {
    JsValue::from_str(&error.to_string())
}

pub(super) fn wasm_edge(shape: Shape<EdgeTag, StandardPayload>) -> Result<WasmEdge, JsValue> {
    let (map, key) = shape.into_model();
    let map = SharedModel::from_model(map);
    let inner = map
        .edge_by_key(key)
        .ok_or_else(|| js_err(format!("missing edge {key:?}")))?;
    Ok(WasmEdge::from_inner(inner))
}

pub(super) fn wasm_profile(
    shape: Shape<ProfileTag, StandardPayload>,
) -> Result<WasmProfile, JsValue> {
    let (map, key) = shape.into_model();
    let map = SharedModel::from_model(map);
    let inner = map
        .profile_by_key(key)
        .ok_or_else(|| js_err(format!("missing profile {key:?}")))?;
    Ok(WasmProfile::from_inner(inner))
}

pub(super) fn wasm_face(shape: Shape<FaceTag, StandardPayload>) -> Result<WasmFace, JsValue> {
    let (map, key) = shape.into_model();
    let map = SharedModel::from_model(map);
    let inner = map
        .face_by_key(key)
        .ok_or_else(|| js_err(format!("missing face {key:?}")))?;
    Ok(WasmFace::from_inner(inner))
}

pub(super) fn wasm_sheet(shape: Shape<SheetTag, StandardPayload>) -> Result<WasmSheet, JsValue> {
    let (map, key) = shape.into_model();
    let map = SharedModel::from_model(map);
    let inner = map
        .sheet_by_key(key)
        .ok_or_else(|| js_err(format!("missing sheet {key:?}")))?;
    Ok(WasmSheet::from_inner(inner))
}

pub(super) fn wasm_solid(shape: Shape<SolidTag, StandardPayload>) -> Result<WasmSolid, JsValue> {
    let (map, key) = shape.into_model();
    let map = SharedModel::from_model(map);
    let inner = map
        .solid_by_key(key)
        .ok_or_else(|| js_err(format!("missing solid {key:?}")))?;
    Ok(WasmSolid::from_inner(inner))
}
