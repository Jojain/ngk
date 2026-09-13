use wasm_bindgen::prelude::*;

use crate::geometry::Point3;
use crate::modeling;

use super::super::topology::WasmEdge;
use super::common::{js_err, wasm_edge};

/// Builds a straight edge from two three-component point arrays.
#[wasm_bindgen(js_name = line)]
pub fn line(start: &[f64], end: &[f64]) -> Result<WasmEdge, JsValue> {
    if start.len() != 3 || end.len() != 3 {
        return Err(js_err(
            "line endpoints must each contain exactly three coordinates",
        ));
    }
    modeling::edges::line(
        Point3::new(start[0], start[1], start[2]),
        Point3::new(end[0], end[1], end[2]),
    )
    .map_err(js_err)
    .and_then(wasm_edge)
}
