use radians::Rad64;
use wasm_bindgen::prelude::*;

use crate::geometry::Rigid;

use super::values::{WasmAxis3, WasmFrame, WasmPoint3, WasmVector3, point, vector};

/// A rigid motion that preserves distances and orientation.
#[wasm_bindgen(js_name = Rigid)]
#[derive(Clone)]
pub struct WasmRigid {
    pub(crate) inner: Rigid,
}

#[wasm_bindgen]
impl WasmRigid {
    /// Returns the motion that moves nothing.
    pub fn identity() -> Self {
        Self {
            inner: Rigid::identity(),
        }
    }

    /// Returns a pure translation.
    pub fn translation(offset: &WasmVector3) -> Self {
        Self {
            inner: Rigid::translation(offset.inner),
        }
    }

    /// Returns a rotation around an axis, with angle in radians.
    pub fn rotation(axis: &WasmAxis3, angle: f64) -> Self {
        Self {
            inner: Rigid::rotation(axis.axis, Rad64::new(angle)),
        }
    }

    /// Carries one frame onto another.
    #[wasm_bindgen(js_name = betweenFrames)]
    pub fn between_frames(from: &WasmFrame, to: &WasmFrame) -> Self {
        Self {
            inner: Rigid::between_frames(&from.inner, &to.inner),
        }
    }

    /// Applies this motion first and `other` second.
    pub fn compose(&self, other: &Self) -> Self {
        Self {
            inner: self.inner.compose(other.inner),
        }
    }

    /// Returns the inverse motion.
    pub fn inverse(&self) -> Self {
        Self {
            inner: self.inner.inverse(),
        }
    }

    /// Moves a point.
    pub fn apply(&self, value: &WasmPoint3) -> WasmPoint3 {
        point(self.inner.apply(value.inner))
    }

    /// Rotates a vector without translating it.
    #[wasm_bindgen(js_name = applyVector)]
    pub fn apply_vector(&self, value: &WasmVector3) -> WasmVector3 {
        vector(self.inner.apply_vector(value.inner))
    }
}
