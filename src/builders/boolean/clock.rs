//! Stage timing on native and browser targets.

#[cfg(not(target_arch = "wasm32"))]
pub(super) use std::time::Instant;

#[cfg(target_arch = "wasm32")]
#[derive(Clone, Copy)]
pub(super) struct Instant(f64);

#[cfg(target_arch = "wasm32")]
impl Instant {
    pub(super) fn now() -> Self {
        Self(js_sys::Date::now())
    }

    pub(super) fn elapsed(self) -> std::time::Duration {
        Self::now() - self
    }
}

#[cfg(target_arch = "wasm32")]
impl std::ops::Sub for Instant {
    type Output = std::time::Duration;

    fn sub(self, earlier: Self) -> Self::Output {
        std::time::Duration::from_secs_f64(((self.0 - earlier.0) / 1000.0).max(0.0))
    }
}
