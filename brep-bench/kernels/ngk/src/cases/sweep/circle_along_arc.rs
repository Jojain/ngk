//! A circle of radius 2 swept along the quarter arc of radius 10 about the z axis in the xy
//! plane, from (10, 0, 0) to (0, 10, 0), square to the path, its ends flat.
use crate::bench::{deg, disc, p, v};
use crate::{Refusal, Solid};
use ngk::geometry::Plane;
use ngk::modeling::{edges, sweep};

pub fn build() -> Result<Solid, Refusal> {
    let path = edges::arc(Plane::xy(), 10.0, deg(0.0), deg(90.0))?;
    let section = disc(p(10.0, 0.0, 0.0), v(0.0, 1.0, 0.0), 2.0)?;
    Ok(sweep::sweep_face(section, &path.edge(), Default::default())?)
}
