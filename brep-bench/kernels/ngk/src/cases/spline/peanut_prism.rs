//! A prism from z = 0 to 10 over the peanut inside the closed uniform periodic cubic B-spline on
//! twelve control points. The periodic curve is written as the cubic on its twelve points and the
//! first three again, on uniform knots, over its periodic domain, then clamped there; it is cut
//! in two at its middle, so each half is an edge with two distinct corners.
use crate::bench::{face_from_edges, nurbs_curve_edge, p, v};
use crate::{Refusal, Solid};
use ngk::geometry::{ControlPolygon, Degree, KnotVector, NurbsCurve};
use ngk::modeling::solids;

pub fn build() -> Result<Solid, Refusal> {
    let k = 21.0 / 8.0;
    let mut points = vec![
        p(14.0, 0.0, 0.0), p(12.0, 8.0, 0.0), p(4.0, 6.0, 0.0), p(0.0, -k, 0.0),
        p(-4.0, 6.0, 0.0), p(-12.0, 8.0, 0.0), p(-14.0, 0.0, 0.0), p(-12.0, -8.0, 0.0),
        p(-4.0, -6.0, 0.0), p(0.0, k, 0.0), p(4.0, -6.0, 0.0), p(12.0, -8.0, 0.0),
    ];
    points.extend_from_within(0..3);
    let knots: Vec<f64> = (0..points.len() + 4).map(|i| i as f64).collect();
    let curve = NurbsCurve::new(
        Degree::new(3)?,
        ControlPolygon::from_cartesian(points.clone(), &vec![1.0; points.len()])?,
        KnotVector::new(knots)?,
    )?;
    // The periodic domain is [3, 15]; split it at 9.
    let curve = curve.trimmed(3.0, 15.0)?;
    let (first, second) = curve.split_at(9.0)?;
    let (a, b) = (nurbs_curve_edge(first)?, nurbs_curve_edge(second)?);
    let section = face_from_edges(&[&a, &b])?;
    Ok(solids::extruded(section, nalgebra::Unit::new_normalize(v(0.0, 0.0, 1.0)), 10.0)?)
}
