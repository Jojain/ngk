//! A cube of side 10 between the origin and (10, 10, 10), the edge where its faces x = 10 and
//! y = 10 meet rounded to radius 2.
use crate::bench::{fillet_through, p};
use crate::{Refusal, Solid};
use ngk::modeling::solids;

pub fn build() -> Result<Solid, Refusal> {
    fillet_through(solids::block(10.0, 10.0, 10.0)?, &[p(10.0, 10.0, 5.0)], 2.0)
}
