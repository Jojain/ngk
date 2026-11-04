//! The union of a cylinder of radius 5 along x from −15 to 15 and one of radius 3 parallel to y
//! from −15 to 15 on the line x = 0, z = 2: their tops touch at (0, 0, 5).
use crate::bench::{fuse, cylinder, p, v};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let main = cylinder(p(-15.0, 0.0, 0.0), v(1.0, 0.0, 0.0), 5.0, 30.0)?;
    let branch = cylinder(p(0.0, -15.0, 2.0), v(0.0, 1.0, 0.0), 3.0, 30.0)?;
    fuse(main, &branch)
}
