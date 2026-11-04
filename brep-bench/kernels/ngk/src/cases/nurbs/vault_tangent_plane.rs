//! A block on −12 ≤ x, y ≤ 12 under the barrel vault y² + (z − 1)² = 15², less everything above
//! the plane z = 16, which touches the vault along its crown. NGK makes no face on a given NURBS
//! surface, so the vault is its row along y, the rational quadratic arc (−12, 10), (0, 26),
//! (12, 10) with weights 1, 3/5, 1, closed by the sides and floor in the plane x = −12 and
//! extruded 24 along x: the degree-1 direction of the vault's net.
use crate::bench::{block, conic_edge, cut, extrude, face_from_edges, line, p, v};
use crate::{Refusal, Solid};

pub fn vault() -> Result<Solid, Refusal> {
    let x = -12.0;
    let floor = line(p(x, -12.0, 0.0), p(x, 12.0, 0.0))?;
    let right = line(p(x, 12.0, 0.0), p(x, 12.0, 10.0))?;
    let arch = conic_edge(p(x, 12.0, 10.0), p(x, 0.0, 26.0), 0.6, p(x, -12.0, 10.0))?;
    let left = line(p(x, -12.0, 10.0), p(x, -12.0, 0.0))?;
    extrude(face_from_edges(&[&floor, &right, &arch, &left])?, v(24.0, 0.0, 0.0))
}

pub fn build() -> Result<Solid, Refusal> {
    cut(vault()?, &block(p(-20.0, -20.0, 16.0), p(20.0, 20.0, 30.0))?)
}
