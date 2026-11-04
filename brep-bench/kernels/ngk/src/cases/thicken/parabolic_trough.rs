//! A parabolic trough, a B-spline sheet, thickened on its concave side.
use crate::{Refusal, Solid, refuse};

pub fn build() -> Result<Solid, Refusal> {
    refuse("NGK has no thickening of a sheet into a solid")
}
