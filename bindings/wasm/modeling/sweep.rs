use wasm_bindgen::prelude::*;

use crate::builders::sweep::{SweepFrame, SweepOptions};
use crate::modeling;

use super::super::geometry::WasmAxis3;
use super::super::topology::{WasmEdge, WasmFace, WasmSolid};
use super::common::{js_err, wasm_solid};

/// Sweeps a face along a helical edge using an axial frame.
#[wasm_bindgen(js_name = sweepFaceAxial)]
pub fn sweep_face_axial(
    face: &WasmFace,
    spine: &WasmEdge,
    axis: &WasmAxis3,
    samples_per_segment: usize,
) -> Result<WasmSolid, JsValue> {
    let face = face.isolated_shape()?;
    let spine = spine.isolated_shape()?;
    let options = SweepOptions {
        frame: SweepFrame::Axial(axis.axis),
        samples_per_segment,
        ..SweepOptions::default()
    };
    modeling::sweep::sweep_face(face, &spine.edge(), options)
        .map_err(js_err)
        .and_then(wasm_solid)
}
