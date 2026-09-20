use radians::Rad64;
use wasm_bindgen::prelude::*;

use crate::geometry::Point3;
use crate::modeling;

use super::super::geometry::WasmAxis3;
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

/// Builds a finite helical edge around an axis.
#[wasm_bindgen(js_name = helix)]
pub fn helix(
    axis: &WasmAxis3,
    radius: f64,
    pitch: f64,
    start_angle: f64,
    end_angle: f64,
) -> Result<WasmEdge, JsValue> {
    modeling::edges::helix(
        axis.axis,
        radius,
        pitch,
        Rad64::new(start_angle),
        Rad64::new(end_angle),
    )
    .map_err(js_err)
    .and_then(wasm_edge)
}
