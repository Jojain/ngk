use wasm_bindgen::prelude::*;

use crate::geometry::Plane;
use crate::modeling;

use super::super::topology::{WasmEdge, WasmProfile};
use super::common::{js_err, wasm_profile};

/// Builds a rectangular profile on the XY plane.
#[wasm_bindgen(js_name = rectangleProfile)]
pub fn rectangle(x_size: f64, y_size: f64) -> Result<WasmProfile, JsValue> {
    modeling::profiles::rectangle(Plane::xy(), x_size, y_size)
        .map_err(js_err)
        .and_then(wasm_profile)
}

/// Builds a profile from connected edges in any input order.
#[wasm_bindgen(js_name = profileFromEdges)]
pub fn from_edges(edges: Vec<WasmEdge>) -> Result<WasmProfile, JsValue> {
    let shapes = edges
        .iter()
        .map(WasmEdge::isolated_shape)
        .collect::<Result<Vec<_>, _>>()?;
    let references = shapes.iter().collect::<Vec<_>>();
    modeling::profiles::from_edges(&references)
        .map_err(js_err)
        .and_then(wasm_profile)
}
