use nalgebra::UnitVector3;
use wasm_bindgen::prelude::*;

use crate::geometry::LINEAR_TOLERANCE;
use crate::modeling;
use crate::topology::shape::Shape;

use super::super::geometry::WasmVector3;
use super::super::topology::{WasmEdge, WasmFace, WasmProfile, WasmSheet, WasmSolid};
use super::common::{js_err, wasm_edge, wasm_face, wasm_profile, wasm_sheet, wasm_solid};

fn unit(direction: &WasmVector3) -> Result<UnitVector3<f64>, JsValue> {
    UnitVector3::try_new(direction.inner, LINEAR_TOLERANCE)
        .ok_or_else(|| js_err("the offset direction must not be zero"))
}

macro_rules! surface_offset {
    ($name:ident, $js_name:ident, $input:ty, $output:ident) => {
        /// Returns a copy moved `distance` along its faces' normals, in a copy
        /// of the input's model that keeps the source beside it.
        #[wasm_bindgen(js_name = $js_name)]
        pub fn $name(shape: &$input, distance: f64) -> Result<$input, JsValue> {
            let source = shape.inner.model();
            let owned = Shape::new(source.model().clone(), shape.inner.key());
            modeling::offset::offset(owned, distance)
                .map_err(js_err)
                .and_then($output)
        }
    };
}

macro_rules! wire_offset {
    ($name:ident, $js_name:ident, $input:ty, $output:ident) => {
        /// Returns a copy moved `distance` along `T × direction`, in a copy of
        /// the input's model that keeps the source beside it.
        #[wasm_bindgen(js_name = $js_name)]
        pub fn $name(
            shape: &$input,
            direction: &WasmVector3,
            distance: f64,
        ) -> Result<$input, JsValue> {
            let direction = unit(direction)?;
            let source = shape.inner.model();
            let owned = Shape::new(source.model().clone(), shape.inner.key());
            modeling::offset::offset_along(owned, direction, distance)
                .map_err(js_err)
                .and_then($output)
        }
    };
}

surface_offset!(offset_face, offsetFace, WasmFace, wasm_face);
surface_offset!(offset_sheet, offsetSheet, WasmSheet, wasm_sheet);
surface_offset!(offset_solid, offsetSolid, WasmSolid, wasm_solid);
wire_offset!(offset_edge, offsetEdge, WasmEdge, wasm_edge);
wire_offset!(offset_profile, offsetProfile, WasmProfile, wasm_profile);

/// Empties `solid` to a wall `thickness` thick, open where `open_faces` were;
/// with no open face, the solid with a cavity. The open faces must come from
/// the solid's own model.
#[wasm_bindgen(js_name = shelledSolid)]
pub fn shelled_solid(
    solid: &WasmSolid,
    open_faces: Vec<WasmFace>,
    thickness: f64,
) -> Result<WasmSolid, JsValue> {
    let source = solid.inner.model();
    let mut keys = Vec::with_capacity(open_faces.len());
    for face in &open_faces {
        if !face.inner.model().ptr_eq(&source) {
            return Err(js_err(
                "an open face must belong to the shelled solid's model",
            ));
        }
        keys.push(face.inner.key());
    }
    let owned = Shape::new(source.model().clone(), solid.inner.key());
    modeling::offset::shelled(owned, &keys, thickness)
        .map_err(js_err)
        .and_then(wasm_solid)
}
