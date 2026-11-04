//! A block under a Bézier patch of degree 15 each way.
use crate::{Refusal, Solid, refuse};

pub fn build() -> Result<Solid, Refusal> {
    refuse("NGK makes no face on a given NURBS surface: its NURBS faces come only from lofts, sweeps and revolutions")
}
