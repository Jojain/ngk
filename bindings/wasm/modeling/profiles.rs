use wasm_bindgen::prelude::*;

use crate::geometry::{Plane, Point3};
use crate::modeling;
use radians::Rad64;

use super::super::geometry::{WasmPlane, WasmPoint3};
use super::super::topology::{WasmEdge, WasmProfile};
use super::common::{js_err, wasm_profile};

/// Builds a rectangular profile on the XY plane.
#[wasm_bindgen(js_name = rectangleProfile)]
pub fn rectangle(
    x_size: f64,
    y_size: f64,
    plane: Option<WasmPlane>,
) -> Result<WasmProfile, JsValue> {
    modeling::profiles::rectangle(
        plane.map_or_else(Plane::xy, |value| value.inner),
        x_size,
        y_size,
    )
    .map_err(js_err)
    .and_then(wasm_profile)
}

/// Builds a square profile on a plane, defaulting to world XY.
#[wasm_bindgen(js_name = squareProfile)]
pub fn square(size: f64, plane: Option<WasmPlane>) -> Result<WasmProfile, JsValue> {
    modeling::profiles::square(plane.map_or_else(Plane::xy, |value| value.inner), size)
        .map_err(js_err)
        .and_then(wasm_profile)
}

/// Builds an open or closed profile from ordered points.
#[wasm_bindgen(js_name = polylineProfile)]
pub fn polyline(points: Vec<WasmPoint3>) -> Result<WasmProfile, JsValue> {
    let points: Vec<Point3> = points.into_iter().map(|value| value.inner).collect();
    modeling::profiles::polyline(&points)
        .map_err(js_err)
        .and_then(wasm_profile)
}

/// Builds a polygon profile from ordered corners.
#[wasm_bindgen(js_name = polygonProfile)]
pub fn polygon(points: Vec<WasmPoint3>) -> Result<WasmProfile, JsValue> {
    let points: Vec<Point3> = points.into_iter().map(|value| value.inner).collect();
    modeling::profiles::polygon(&points)
        .map_err(js_err)
        .and_then(wasm_profile)
}

/// Builds an arc profile on a plane. Angles are in radians.
#[wasm_bindgen(js_name = arcProfile)]
pub fn arc(
    plane: &WasmPlane,
    radius: f64,
    start_angle: f64,
    end_angle: f64,
) -> Result<WasmProfile, JsValue> {
    modeling::profiles::arc(
        plane.inner.clone(),
        radius,
        Rad64::new(start_angle),
        Rad64::new(end_angle),
    )
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

/// Promotes an owned edge to a one-edge profile.
#[wasm_bindgen(js_name = profileFromEdge)]
pub fn from_edge(edge: &WasmEdge) -> Result<WasmProfile, JsValue> {
    wasm_profile(edge.isolated_shape()?.into_profile())
}

/// Returns a profile with one connected edge appended.
#[wasm_bindgen(js_name = appendedProfile)]
pub fn appended(profile: &WasmProfile, edge: &WasmEdge) -> Result<WasmProfile, JsValue> {
    let profile = profile.isolated_shape()?;
    let edge = edge.isolated_shape()?;
    modeling::profiles::appended(profile, &edge)
        .map_err(js_err)
        .and_then(wasm_profile)
}
