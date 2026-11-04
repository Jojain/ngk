//! A square bar 20 × 20 swept along the cubic Bézier (0, 0, 0), (16, 12, 0), (18, 26, 0),
//! (12, 34, 0) in the xy plane, square to it, two of its sides horizontal; its ends flat.
use crate::bench::{bezier_edge, p};
use crate::{Refusal, Solid};
use ngk::geometry::Fraction;
use ngk::modeling::{faces, sweep};

pub fn build() -> Result<Solid, Refusal> {
    let path = bezier_edge([p(0.0, 0.0, 0.0), p(16.0, 12.0, 0.0), p(18.0, 26.0, 0.0), p(12.0, 34.0, 0.0)])?;
    let t = path.edge().trimmed_curve().derivative_at(Fraction::START, 1).normalize();
    // The section's horizontal sides run square to the start tangent, (−3, 4, 0)/5.
    let side = nalgebra::Vector3::new(-t.y, t.x, 0.0) * 10.0;
    let at = |s: f64, z: f64| p(side.x * s, side.y * s, z);
    let section = faces::polygon(&[at(-1.0, -10.0), at(1.0, -10.0), at(1.0, 10.0), at(-1.0, 10.0)])?;
    Ok(sweep::sweep_face(section, &path.edge(), Default::default())?)
}
