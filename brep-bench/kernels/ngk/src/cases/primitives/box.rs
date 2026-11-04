//! A rectangular block 10 long in x, 20 in y and 30 in z, between the origin and (10, 20, 30).
use crate::{Refusal, Solid};
use ngk::modeling::solids;

pub fn build() -> Result<Solid, Refusal> {
    Ok(solids::block(10.0, 20.0, 30.0)?)
}
