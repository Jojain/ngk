use js_sys::Array;
use wasm_bindgen::prelude::*;

use crate::binding_common::explore::SharedModel;
use crate::binding_common::modeling::extrusion_direction;
use crate::builders::boolean::BooleanOperation;
use crate::modeling;
use crate::topology::shape_keys::FaceKey;

use super::super::geometry::{WasmFrame, WasmVector3};
use super::super::topology::WasmModel;
use super::super::topology::{WasmFace, WasmSolid};
use super::common::{js_err, wasm_solid};

/// Builds an axis-aligned block and returns its read-only solid handle.
#[wasm_bindgen(js_name = block)]
pub fn block(
    x_size: f64,
    y_size: f64,
    z_size: f64,
    frame: Option<WasmFrame>,
) -> Result<WasmSolid, JsValue> {
    match frame {
        Some(frame) => modeling::solids::block_at(frame.inner, x_size, y_size, z_size),
        None => modeling::solids::block(x_size, y_size, z_size),
    }
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
///
/// The optional frame matches the Python binding: omitted means `Frame::xyz()`.
#[wasm_bindgen(js_name = cylinder)]
pub fn cylinder(radius: f64, height: f64, frame: Option<WasmFrame>) -> Result<WasmSolid, JsValue> {
    match frame {
        Some(frame) => modeling::solids::cylinder_at(frame.inner.clone(), radius, height),
        None => modeling::solids::cylinder(radius, height),
    }
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

/// Extrudes an owned face along a direction by the given distance.
#[wasm_bindgen(js_name = extruded)]
pub fn extrude_face(
    face: &WasmFace,
    direction: &WasmVector3,
    distance: f64,
) -> Result<WasmSolid, JsValue> {
    let direction = extrusion_direction(direction.inner).map_err(js_err)?;
    modeling::solids::extruded(face.isolated_shape()?, direction, distance)
        .map_err(js_err)
        .and_then(wasm_solid)
}

/// Builds a sphere and returns its read-only solid handle.
#[wasm_bindgen(js_name = sphere)]
pub fn sphere(radius: f64, frame: Option<WasmFrame>) -> Result<WasmSolid, JsValue> {
    match frame {
        Some(frame) => modeling::solids::sphere_at(frame.inner, radius),
        None => modeling::solids::sphere(radius),
    }
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
pub fn torus(major: f64, minor: f64, frame: Option<WasmFrame>) -> Result<WasmSolid, JsValue> {
    match frame {
        Some(frame) => modeling::solids::torus_at(frame.inner, major, minor),
        None => modeling::solids::torus(major, minor),
    }
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

/// One face Boolean answer with all result faces in a shared model.
#[wasm_bindgen(js_name = FaceBooleanResult)]
pub struct WasmFaceBooleanResult {
    model: SharedModel,
    faces: Vec<FaceKey>,
}

#[wasm_bindgen]
impl WasmFaceBooleanResult {
    #[wasm_bindgen(getter)]
    pub fn model(&self) -> WasmModel {
        WasmModel::from_inner(self.model.clone())
    }

    #[wasm_bindgen(unchecked_return_type = "Face[]")]
    pub fn faces(&self) -> Result<Array, JsValue> {
        let result = Array::new();
        for key in &self.faces {
            let face = self
                .model
                .face_by_key(*key)
                .ok_or_else(|| js_err(format!("missing face {key:?}")))?;
            result.push(&WasmFace::from_inner(face).into());
        }
        Ok(result)
    }
}

#[wasm_bindgen(js_name = fuseFaces)]
pub fn fuse_faces(first: &WasmFace, second: &WasmFace) -> Result<WasmFaceBooleanResult, JsValue> {
    combine_faces(first, second, BooleanOperation::Union)
}

#[wasm_bindgen(js_name = cutFaces)]
pub fn cut_faces(first: &WasmFace, second: &WasmFace) -> Result<WasmFaceBooleanResult, JsValue> {
    combine_faces(first, second, BooleanOperation::Difference)
}

#[wasm_bindgen(js_name = intersectFaces)]
pub fn intersect_faces(
    first: &WasmFace,
    second: &WasmFace,
) -> Result<WasmFaceBooleanResult, JsValue> {
    combine_faces(first, second, BooleanOperation::Intersection)
}

fn combine_faces(
    first: &WasmFace,
    second: &WasmFace,
    operation: BooleanOperation,
) -> Result<WasmFaceBooleanResult, JsValue> {
    let result = modeling::faces::combine_views(
        first.inner.view().map_err(js_err)?,
        second.inner.view().map_err(js_err)?,
        operation,
    )
    .map_err(js_err)?;
    let (model, faces) = result.into_model();
    Ok(WasmFaceBooleanResult {
        model: SharedModel::from_model(model),
        faces,
    })
}
