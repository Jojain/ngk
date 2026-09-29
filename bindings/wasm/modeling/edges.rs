use js_sys::Array;
use radians::Rad64;
use wasm_bindgen::prelude::*;

use crate::binding_common::explore::SharedModel;
use crate::builders::edges::EdgeSplit;
use crate::geometry::{Fraction, Point3};
use crate::modeling;

use super::super::geometry::{WasmAxis3, WasmPlane};
use super::super::topology::{WasmEdge, WasmFace, WasmModel, WasmVertex};
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

/// Builds a circular arc edge on a plane. Angles are in radians.
#[wasm_bindgen(js_name = arc)]
pub fn arc(
    plane: &WasmPlane,
    radius: f64,
    start_angle: f64,
    end_angle: f64,
) -> Result<WasmEdge, JsValue> {
    modeling::edges::arc(
        plane.inner.clone(),
        radius,
        Rad64::new(start_angle),
        Rad64::new(end_angle),
    )
    .map_err(js_err)
    .and_then(wasm_edge)
}

/// Builds a closed circular edge on a plane.
#[wasm_bindgen(js_name = circleEdge)]
pub fn circle(plane: &WasmPlane, radius: f64) -> Result<WasmEdge, JsValue> {
    modeling::edges::circle(plane.inner.clone(), radius)
        .map_err(js_err)
        .and_then(wasm_edge)
}

#[wasm_bindgen(js_name = EdgeSplitResult)]
pub struct WasmEdgeSplitResult {
    model: SharedModel,
    split: EdgeSplit,
    face: Option<crate::topology::shape_keys::FaceKey>,
}

impl WasmEdgeSplitResult {
    pub(crate) fn from_result(result: modeling::edges::EdgeSplitResult) -> Self {
        let (model, split, face) = result.into_model();
        Self {
            model: SharedModel::from_model(model),
            split,
            face,
        }
    }
}

#[wasm_bindgen]
impl WasmEdgeSplitResult {
    #[wasm_bindgen(getter)]
    pub fn model(&self) -> WasmModel {
        WasmModel::from_inner(self.model.clone())
    }

    #[wasm_bindgen(getter)]
    pub fn separated(&self) -> bool {
        matches!(self.split, EdgeSplit::Separated { .. })
    }

    #[wasm_bindgen(unchecked_return_type = "Edge[]")]
    pub fn edges(&self) -> Result<Array, JsValue> {
        let result = Array::new();
        for key in self.split.edges() {
            let edge = self
                .model
                .edge_by_key(key)
                .ok_or_else(|| js_err(format!("missing edge {key:?}")))?;
            result.push(&WasmEdge::from_inner(edge).into());
        }
        Ok(result)
    }

    pub fn vertex(&self) -> Result<WasmVertex, JsValue> {
        let key = self.split.vertex();
        self.model
            .vertex_by_key(key)
            .map(WasmVertex::from_inner)
            .ok_or_else(|| js_err(format!("missing vertex {key:?}")))
    }

    pub fn face(&self) -> Result<Option<WasmFace>, JsValue> {
        self.face
            .map(|key| {
                self.model
                    .face_by_key(key)
                    .map(WasmFace::from_inner)
                    .ok_or_else(|| js_err(format!("missing face {key:?}")))
            })
            .transpose()
    }
}

#[wasm_bindgen(js_name = splitEdge)]
pub fn split(edge: &WasmEdge, fraction: f64) -> Result<WasmEdgeSplitResult, JsValue> {
    let shape = edge.isolated_shape()?;
    let result = modeling::edges::split(shape, Fraction::new(fraction)).map_err(js_err)?;
    Ok(WasmEdgeSplitResult::from_result(result))
}
