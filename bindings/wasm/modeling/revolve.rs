use radians::Rad64;
use wasm_bindgen::prelude::*;

use crate::modeling;

use super::super::geometry::WasmAxis3;
use super::super::topology::{WasmEdge, WasmFace, WasmProfile, WasmSheet, WasmSolid};
use super::common::{js_err, wasm_face, wasm_sheet, wasm_solid};

/// Revolves an edge around an axis to make one face.
#[wasm_bindgen(js_name = revolveEdge)]
pub fn revolve_edge(edge: &WasmEdge, axis: &WasmAxis3, angle: f64) -> Result<WasmFace, JsValue> {
    modeling::revolve::revolve_edge(edge.isolated_shape()?, axis.axis, Rad64::new(angle))
        .map_err(js_err)
        .and_then(wasm_face)
}

/// Revolves a profile around an axis to make a sheet.
#[wasm_bindgen(js_name = revolveProfile)]
pub fn revolve_profile(
    profile: &WasmProfile,
    axis: &WasmAxis3,
    angle: f64,
) -> Result<WasmSheet, JsValue> {
    let shape = profile.isolated_shape()?;
    modeling::revolve::revolve_profile(shape.profile(), axis.axis, Rad64::new(angle))
        .map_err(js_err)
        .and_then(wasm_sheet)
}

/// Revolves a face around an axis to make a solid.
#[wasm_bindgen(js_name = revolveFace)]
pub fn revolve_face(face: &WasmFace, axis: &WasmAxis3, angle: f64) -> Result<WasmSolid, JsValue> {
    modeling::revolve::revolve_face(face.isolated_shape()?, axis.axis, Rad64::new(angle))
        .map_err(js_err)
        .and_then(wasm_solid)
}
