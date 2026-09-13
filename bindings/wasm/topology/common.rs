use js_sys::Array;
use wasm_bindgen::prelude::*;

use crate::binding_common::explore::SharedModel;
use crate::topology::StandardPayload;

pub(crate) type Map = SharedModel<StandardPayload>;

pub(crate) fn js_err(error: impl ToString) -> JsValue {
    JsValue::from_str(&error.to_string())
}

pub(crate) fn values(items: impl IntoIterator<Item = JsValue>) -> Array {
    let values = Array::new();
    for item in items {
        values.push(&item);
    }
    values
}

macro_rules! entity_common {
    ($type:ident) => {
        #[wasm_bindgen]
        impl $type {
            /// Returns the owning immutable model.
            #[wasm_bindgen(getter)]
            pub fn model(&self) -> WasmModel {
                WasmModel::from_inner(self.inner.model())
            }

            /// Returns the opaque stable key.
            #[wasm_bindgen(getter)]
            pub fn key(&self) -> String {
                format!("{:?}", self.inner.key())
            }

            /// Returns the contextual dart id, or `undefined` for dart-less topology.
            #[wasm_bindgen(getter, js_name = dartId)]
            pub fn dart_id(&self) -> Option<usize> {
                self.inner.dart_id()
            }

            /// Tests topological identity, independent of contextual orientation.
            pub fn equals(&self, other: &$type) -> bool {
                self.inner.same_entity(&other.inner)
            }
        }
    };
}

/// Read-only solid handle.
pub(crate) use entity_common;
