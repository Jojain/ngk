use wasm_bindgen::prelude::*;

use crate::geometry::{Fraction, Plane, Point3};
use crate::modeling;
use crate::topology::shape::Shape;

use super::super::geometry::{WasmPlane, WasmPoint3};
use super::super::topology::{WasmEdge, WasmFace, WasmProfile};
use super::common::{js_err, wasm_face};
use super::edges::WasmEdgeSplitResult;

/// Cuts a boundary edge and updates every incident pcurve.
#[wasm_bindgen(js_name = splitFaceBoundaryEdge)]
pub fn split_boundary_edge(
    face: &WasmFace,
    edge: &WasmEdge,
    fraction: f64,
) -> Result<WasmEdgeSplitResult, JsValue> {
    let source = face.inner.model();
    if !source.ptr_eq(&edge.inner.model()) {
        return Err(js_err("edge and face must belong to the same model"));
    }
    let shape = Shape::new(source.model().clone(), face.inner.key());
    let result =
        modeling::faces::split_boundary_edge(shape, edge.inner.key(), Fraction::new(fraction))
            .map_err(js_err)?;
    Ok(WasmEdgeSplitResult::from_result(result))
}

/// Builds a rectangular face on the XY plane.
#[wasm_bindgen(js_name = rectangleFace)]
pub fn rectangle(x_size: f64, y_size: f64, plane: Option<WasmPlane>) -> Result<WasmFace, JsValue> {
    modeling::faces::rectangle(
        plane.map_or_else(Plane::xy, |value| value.inner),
        x_size,
        y_size,
    )
    .map_err(js_err)
    .and_then(wasm_face)
}

/// Builds a square face on a plane, defaulting to world XY.
#[wasm_bindgen(js_name = squareFace)]
pub fn square(size: f64, plane: Option<WasmPlane>) -> Result<WasmFace, JsValue> {
    modeling::faces::square(plane.map_or_else(Plane::xy, |value| value.inner), size)
        .map_err(js_err)
        .and_then(wasm_face)
}

/// Builds a circular face on a plane, defaulting to world XY.
#[wasm_bindgen(js_name = circleFace)]
pub fn circle(radius: f64, plane: Option<WasmPlane>) -> Result<WasmFace, JsValue> {
    modeling::faces::circle(plane.map_or_else(Plane::xy, |value| value.inner), radius)
        .map_err(js_err)
        .and_then(wasm_face)
}

/// Builds an annular face on a plane, defaulting to world XY.
#[wasm_bindgen(js_name = annulusFace)]
pub fn annulus(
    outer_radius: f64,
    inner_radius: f64,
    plane: Option<WasmPlane>,
) -> Result<WasmFace, JsValue> {
    modeling::faces::annulus(
        plane.map_or_else(Plane::xy, |value| value.inner),
        outer_radius,
        inner_radius,
    )
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

/// Builds a planar face from an outer polygon and zero or more holes.
#[wasm_bindgen(js_name = polygonWithHolesFace)]
pub fn polygon_with_holes(
    #[wasm_bindgen(unchecked_param_type = "number[][]")] outer: JsValue,
    #[wasm_bindgen(unchecked_param_type = "number[][][]")] holes: JsValue,
    plane: Option<WasmPlane>,
) -> Result<WasmFace, JsValue> {
    let outer: Vec<[f64; 3]> = serde_wasm_bindgen::from_value(outer).map_err(js_err)?;
    let holes: Vec<Vec<[f64; 3]>> = serde_wasm_bindgen::from_value(holes).map_err(js_err)?;
    let outer = outer.into_iter().map(Point3::from).collect::<Vec<_>>();
    let holes = holes
        .into_iter()
        .map(|hole| hole.into_iter().map(Point3::from).collect::<Vec<_>>())
        .collect::<Vec<_>>();
    let hole_refs = holes.iter().map(Vec::as_slice).collect::<Vec<_>>();
    modeling::faces::polygon_with_holes(
        plane.map_or_else(Plane::xy, |value| value.inner),
        &outer,
        &hole_refs,
    )
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
