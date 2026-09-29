use wasm_bindgen::prelude::*;

use crate::modeling;

use super::super::geometry::WasmRigid;
use super::super::topology::{WasmEdge, WasmFace, WasmProfile, WasmSheet, WasmSolid};
use super::common::{wasm_edge, wasm_face, wasm_profile, wasm_sheet, wasm_solid};

macro_rules! moved_shape {
    ($name:ident, $js_name:ident, $shape:ty, $convert:ident) => {
        /// Moves an owned shape by a rigid motion without changing the source.
        #[wasm_bindgen(js_name = $js_name)]
        pub fn $name(shape: &$shape, motion: &WasmRigid) -> Result<$shape, JsValue> {
            let owned = shape.isolated_shape()?;
            $convert(modeling::transform::moved(owned, motion.inner))
        }
    };
}

moved_shape!(moved_edge, movedEdge, WasmEdge, wasm_edge);
moved_shape!(moved_profile, movedProfile, WasmProfile, wasm_profile);
moved_shape!(moved_face, movedFace, WasmFace, wasm_face);
moved_shape!(moved_sheet, movedSheet, WasmSheet, wasm_sheet);
moved_shape!(moved_solid, movedSolid, WasmSolid, wasm_solid);
