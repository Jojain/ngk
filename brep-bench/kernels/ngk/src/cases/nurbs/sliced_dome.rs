//! A block whose top is a given B-spline patch over the square 0 ≤ x, y ≤ 20.
use crate::{Refusal, Solid, refuse};

pub fn build() -> Result<Solid, Refusal> {
    refuse("NGK makes no face on a given NURBS surface: its NURBS faces come only from lofts, sweeps and revolutions")
}
