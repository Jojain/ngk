//! The cone on z with base the disc of radius 5 at z = 0 and apex (0, 0, 10), less everything
//! where x < 0: the cut holds the axis and so the apex.
use crate::bench::{block, cone, cut, p, v};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let c = cone(p(0.0, 0.0, 0.0), v(0.0, 0.0, 1.0), 5.0, 0.0, 10.0)?;
    cut(c, &block(p(-10.0, -10.0, -1.0), p(0.0, 10.0, 11.0))?)
}
