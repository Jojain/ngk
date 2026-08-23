use js_sys::Array;
use wasm_bindgen::prelude::*;

use crate::binding_common::explore::SharedGMap;
use crate::topology::StandardPayload;

pub(crate) type Map = SharedGMap<StandardPayload>;

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
            /// Returns the owning immutable GMap.
            #[wasm_bindgen(getter)]
            pub fn gmap(&self) -> WasmGMap {
                WasmGMap::from_inner(self.inner.gmap())
            }

            /// Returns the opaque stable key.
            #[wasm_bindgen(getter)]
            pub fn key(&self) -> String {
                format!("{:?}", self.inner.key())
            }

            /// Returns the contextual dart id.
            #[wasm_bindgen(getter, js_name = dartId)]
            pub fn dart_id(&self) -> usize {
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
