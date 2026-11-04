//! The L section (0, 0), (10, 0), (10, 3), (3, 3), (3, 10), (0, 10) in xz extruded along +y by 10,
//! its inside edge x = 3, z = 3 rounded to radius 7, which consumes both faces beside it.
use crate::bench::{fillet_through, p, prism, v};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let l = prism(
        &[p(0.0, 0.0, 0.0), p(10.0, 0.0, 0.0), p(10.0, 0.0, 3.0), p(3.0, 0.0, 3.0), p(3.0, 0.0, 10.0), p(0.0, 0.0, 10.0)],
        v(0.0, 10.0, 0.0),
    )?;
    fillet_through(l, &[p(3.0, 5.0, 3.0)], 7.0)
}
