use wasm_bindgen::prelude::*;

use crate::binding_common::modeling::SharedBlendTarget;
use crate::modeling;
use crate::topology::shape::Shape;

use super::super::topology::{WasmEdge, WasmFace, WasmProfile, WasmSolid, WasmVertex};
use super::common::{js_err, wasm_face, wasm_profile, wasm_solid};

/// A mixed selection of vertices, edges, profiles, and faces from one model.
#[wasm_bindgen(js_name = BlendTarget)]
#[derive(Clone)]
pub struct WasmBlendTarget {
    inner: SharedBlendTarget,
}

impl Default for WasmBlendTarget {
    fn default() -> Self {
        Self::new()
    }
}

#[wasm_bindgen]
impl WasmBlendTarget {
    /// Creates an empty blend target.
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        Self {
            inner: SharedBlendTarget::new(),
        }
    }

    /// Selects a vertex from this target's model.
    #[wasm_bindgen(js_name = addVertex)]
    pub fn add_vertex(&mut self, vertex: &WasmVertex) -> Result<(), JsValue> {
        self.inner
            .add(vertex.inner.model(), vertex.inner.key().into())
            .map_err(js_err)
    }

    /// Selects an edge from this target's model.
    #[wasm_bindgen(js_name = addEdge)]
    pub fn add_edge(&mut self, edge: &WasmEdge) -> Result<(), JsValue> {
        self.inner
            .add(edge.inner.model(), edge.inner.key().into())
            .map_err(js_err)
    }

    /// Selects a profile from this target's model.
    #[wasm_bindgen(js_name = addProfile)]
    pub fn add_profile(&mut self, profile: &WasmProfile) -> Result<(), JsValue> {
        self.inner
            .add(profile.inner.model(), profile.inner.key().into())
            .map_err(js_err)
    }

    /// Selects a face from this target's model.
    #[wasm_bindgen(js_name = addFace)]
    pub fn add_face(&mut self, face: &WasmFace) -> Result<(), JsValue> {
        self.inner
            .add(face.inner.model(), face.inner.key().into())
            .map_err(js_err)
    }
}

macro_rules! blend_operation {
    ($name:ident, $js_name:ident, $input:ty, $output:ident, $operation:path) => {
        /// Blends the selected corners or edges and returns a new owned shape.
        #[wasm_bindgen(js_name = $js_name)]
        pub fn $name(
            shape: &$input,
            target: &WasmBlendTarget,
            amount: f64,
        ) -> Result<$input, JsValue> {
            let selection = target
                .inner
                .for_model(&shape.inner.model())
                .map_err(js_err)?;
            let source = shape.inner.model();
            let owned = Shape::new(source.model().clone(), shape.inner.key());
            $operation(owned, selection, amount)
                .map_err(js_err)
                .and_then($output)
        }
    };
}

blend_operation!(
    filleted_profile,
    filletedProfile,
    WasmProfile,
    wasm_profile,
    modeling::blend::filleted
);
blend_operation!(
    filleted_face,
    filletedFace,
    WasmFace,
    wasm_face,
    modeling::blend::filleted
);
blend_operation!(
    filleted_solid,
    filletedSolid,
    WasmSolid,
    wasm_solid,
    modeling::blend::filleted
);
blend_operation!(
    chamfered_profile,
    chamferedProfile,
    WasmProfile,
    wasm_profile,
    modeling::blend::chamfered
);
blend_operation!(
    chamfered_face,
    chamferedFace,
    WasmFace,
    wasm_face,
    modeling::blend::chamfered
);
blend_operation!(
    chamfered_solid,
    chamferedSolid,
    WasmSolid,
    wasm_solid,
    modeling::blend::chamfered
);
