//! The drum of `geometry/saddle-drum`, bored along its axis by a hole of radius 3, the drill
//! running from z = −1 to 30, past the floor and the saddle.
use super::saddle_drum::under_saddle;
use crate::bench::{cut, cylinder_z};
use crate::{Refusal, Solid};
use ngk::modeling::solids;

pub fn build() -> Result<Solid, Refusal> {
    let drum = under_saddle(solids::cylinder(8.0, 30.0)?)?;
    cut(drum, &cylinder_z(0.0, 0.0, -1.0, 30.0, 3.0)?)
}
