//! The octahedron |x| + |y| + |z| ≤ 10, all twelve edges chamfered √6 on both faces, the four
//! at each corner meeting in a point: chamfered together, mitred where they meet.
use crate::bench::{edges, octahedron};
use crate::{Refusal, Solid};
use ngk::modeling::blend;

pub fn build() -> Result<Solid, Refusal> {
    let octa = octahedron(10.0)?;
    let all = edges(&octa);
    Ok(blend::chamfered(octa, all, 6f64.sqrt())?)
}
