//! A hollow square duct bent a quarter turn about the z axis on the arc of radius 30, its
//! half-width running linearly with the angle from 9 to 6.
use crate::{Refusal, Solid, refuse};

pub fn build() -> Result<Solid, Refusal> {
    refuse("NGK sweeps a section rigidly; it has no sweep whose section scales along the path")
}
