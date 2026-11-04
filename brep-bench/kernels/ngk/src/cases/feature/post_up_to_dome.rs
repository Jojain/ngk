//! A closed hollow dome: the plate of radius 20 from z = −2 to 0 and on it the hemispherical
//! shell between 18 and 20, around a cavity, the half ball of radius 18; and a post, the disc of
//! radius 3 on the cavity's floor extruded up until it meets the dome's inside, joined to it.
//! NGK has no extrusion up to a face, so the post is the cylinder of radius 3 from z = 0 to 18
//! less what lies outside the ball of radius 18.
use crate::bench::{ball, block, common, cut, cylinder_z, fuse, p};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let upper = |r: f64| -> Result<Solid, Refusal> {
        cut(ball(p(0.0, 0.0, 0.0), r)?, &block(p(-30.0, -30.0, -30.0), p(30.0, 30.0, 0.0))?)
    };
    let dome = fuse(cylinder_z(0.0, 0.0, -2.0, 0.0, 20.0)?, &upper(20.0)?)?;
    let cavity = upper(18.0)?;
    let post = common(cylinder_z(0.0, 0.0, 0.0, 18.0, 3.0)?, &ball(p(0.0, 0.0, 0.0), 18.0)?)?;
    let cavity = cut(cavity, &post)?;
    cut(dome, &cavity)
}
