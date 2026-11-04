//! A cube of side 10 with all twelve edges rounded to radius 1, three rounds at each corner.
use crate::bench::edges;
use crate::{Refusal, Solid};
use ngk::modeling::{blend, solids};

pub fn build() -> Result<Solid, Refusal> {
    let cube = solids::block(10.0, 10.0, 10.0)?;
    let all = edges(&cube);
    Ok(blend::filleted(cube, all, 1.0)?)
}
