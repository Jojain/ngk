//! A C bracket, the polygon (0, 0), (40, 0), (40, 6), (6, 6), (6, 24), (40, 24), (40, 30), (0, 30)
//! in the plane y = −10 extruded to y = 10, its upper arm drilled by a hole of radius 4 on the
//! vertical through (28, 0) down to the gap. NGK has no hole up to the next face, so the drill is
//! a cylinder from z = 31 down into the gap, z = 20, which the gap leaves empty.
use crate::bench::{cut, cylinder_z, p, prism, v};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let at = |x: f64, z: f64| p(x, -10.0, z);
    let c = prism(
        &[at(0.0, 0.0), at(40.0, 0.0), at(40.0, 6.0), at(6.0, 6.0), at(6.0, 24.0), at(40.0, 24.0), at(40.0, 30.0), at(0.0, 30.0)],
        v(0.0, 20.0, 0.0),
    )?;
    cut(c, &cylinder_z(28.0, 0.0, 20.0, 31.0, 4.0)?)
}
