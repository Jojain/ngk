//! The rectangle x from 6 to 10, z from 0 to 4 in the xz plane, revolved 90° about the z axis
//! from +x towards +y.
use crate::bench::{axis, deg, p, v};
use crate::{Refusal, Solid};
use ngk::modeling::{faces, revolve};

pub fn build() -> Result<Solid, Refusal> {
    let section = faces::polygon(&[p(6.0, 0.0, 4.0), p(10.0, 0.0, 4.0), p(10.0, 0.0, 0.0), p(6.0, 0.0, 0.0)])?;
    Ok(revolve::revolve_face(section, axis(p(0.0, 0.0, 0.0), v(0.0, 0.0, 1.0)), deg(90.0))?)
}
