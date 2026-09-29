use wasm_bindgen::prelude::*;

use crate::binding_common::modeling::sweep_options;
use crate::builders::sweep::SweepOptions;
use crate::modeling;

use super::super::geometry::{WasmAxis3, WasmVector3};
use super::super::topology::{WasmEdge, WasmFace, WasmProfile, WasmSheet, WasmSolid};
use super::common::{js_err, wasm_sheet, wasm_solid};

/// Frame transport, junction transition, and curved-spine resolution.
#[wasm_bindgen(js_name = SweepOptions)]
#[derive(Clone)]
pub struct WasmSweepOptions {
    inner: SweepOptions,
}

#[wasm_bindgen]
impl WasmSweepOptions {
    /// Creates sweep options, defaulting to parallel/smooth with eight samples.
    #[wasm_bindgen(constructor)]
    pub fn new(
        frame: Option<String>,
        axis: Option<WasmAxis3>,
        transition: Option<String>,
        samples_per_segment: Option<usize>,
    ) -> Result<WasmSweepOptions, JsValue> {
        let inner = sweep_options(
            frame.as_deref().unwrap_or("parallel"),
            axis.map(|value| value.axis),
            transition.as_deref().unwrap_or("smooth"),
            samples_per_segment.unwrap_or(8),
        )
        .map_err(js_err)?;
        Ok(Self { inner })
    }
}

/// Extrudes a profile by a displacement vector to make a sheet.
#[wasm_bindgen(js_name = extrudeProfile)]
pub fn extrude_profile(
    profile: &WasmProfile,
    direction: &WasmVector3,
) -> Result<WasmSheet, JsValue> {
    let shape = profile.isolated_shape()?;
    modeling::sweep::extrude_profile(shape.profile(), direction.inner)
        .map_err(js_err)
        .and_then(wasm_sheet)
}

/// Extrudes a face by a displacement vector to make a solid.
#[wasm_bindgen(js_name = extrudeFace)]
pub fn extrude_face(face: &WasmFace, direction: &WasmVector3) -> Result<WasmSolid, JsValue> {
    modeling::sweep::extrude_face(face.isolated_shape()?, direction.inner)
        .map_err(js_err)
        .and_then(wasm_solid)
}

/// Sweeps a face along one edge.
#[wasm_bindgen(js_name = sweepFaceAlongEdge)]
pub fn face_along_edge(
    face: &WasmFace,
    spine: &WasmEdge,
    options: Option<WasmSweepOptions>,
) -> Result<WasmSolid, JsValue> {
    let face = face.isolated_shape()?;
    let spine = spine.isolated_shape()?;
    modeling::sweep::sweep_face(
        face,
        &spine.edge(),
        options.map_or_else(SweepOptions::default, |value| value.inner),
    )
    .map_err(js_err)
    .and_then(wasm_solid)
}

/// Sweeps a face along an open or closed profile.
#[wasm_bindgen(js_name = sweepFaceAlongProfile)]
pub fn face_along_profile(
    face: &WasmFace,
    spine: &WasmProfile,
    options: Option<WasmSweepOptions>,
) -> Result<WasmSolid, JsValue> {
    let face = face.isolated_shape()?;
    let spine = spine.isolated_shape()?;
    modeling::sweep::sweep_face(
        face,
        &spine.profile(),
        options.map_or_else(SweepOptions::default, |value| value.inner),
    )
    .map_err(js_err)
    .and_then(wasm_solid)
}
