//! A cube of side 10, all twelve edges chamfered 2 on both faces, the three at each corner
//! meeting in a point: NGK chamfers the edges together and mitres them where they meet.
use crate::bench::edges;
use crate::{Refusal, Solid};
use ngk::modeling::{blend, solids};

pub fn build() -> Result<Solid, Refusal> {
    let cube = solids::block(10.0, 10.0, 10.0)?;
    let all = edges(&cube);
    Ok(blend::chamfered(cube, all, 2.0)?)
}
