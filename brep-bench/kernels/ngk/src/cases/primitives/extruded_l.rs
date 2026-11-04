//! An L-shaped plate: the polygon (0, 0), (20, 0), (20, 5), (5, 5), (5, 15), (0, 15) in the xy
//! plane, extruded 8 along +z.
use crate::bench::p;
use crate::{Refusal, Solid};
use nalgebra::Vector3;
use ngk::modeling::{faces, solids};

pub fn build() -> Result<Solid, Refusal> {
    let outline = faces::polygon(&[
        p(0.0, 0.0, 0.0),
        p(20.0, 0.0, 0.0),
        p(20.0, 5.0, 0.0),
        p(5.0, 5.0, 0.0),
        p(5.0, 15.0, 0.0),
        p(0.0, 15.0, 0.0),
    ])?;
    Ok(solids::extruded(outline, Vector3::z_axis(), 8.0)?)
}
