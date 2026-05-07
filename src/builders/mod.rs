//! Higher-level helpers that mutate a [`GMap`]. Kept separate from the gmap
//! itself so the combinatorial core stays small; anything opinionated (how to
//! build a polygon, how to stitch cells, etc.) lives here.

use crate::geometry::{Curve, Line, Point3};
use crate::topology::attributes::{EdgeAttr, VertexAttr};
use crate::topology::gmap::{Dart, Dim, GMap};
use crate::topology::payload::Payload;
use crate::topology::shape_keys::EdgeKey;

pub fn add_edge<P: Payload>(g: &mut GMap<P>, start: Point3, end: Point3, curve: Curve) -> EdgeKey {
    let d1 = g.add_dart();
    let d2 = g.add_dart();
    g.sew_unchecked(Dim::Zero, d1, d2);
    g.add_vertex(VertexAttr::new(d1, start, P::V::default()));
    g.add_vertex(VertexAttr::new(d2, end, P::V::default()));
    let e = g.add_edge(EdgeAttr::new(d1, curve, P::E::default()));
    e
}

/// Adds a single polygon face to `g` with the given corner points (in order).
///
/// Sews α0 and α1 to form a closed `n`-gon, stamps the vertex positions on
/// every dart of each corner's vertex orbit, and attaches a straight
/// [`Curve::Line`] on every 1-cell so downstream consumers (edge tessellation,
/// dart geometry) have a curve to follow. Does not touch α2 — the face is
/// returned with free boundary, ready to be stitched to neighbors.
///
/// Returns a dart on the outer ⟨α₀, α₁⟩ loop (same as the first corner dart).
pub fn add_polygon<P: Payload>(g: &mut GMap<P>, corners: &[Point3]) -> Dart {
    assert!(
        corners.len() >= 3,
        "add_polygon requires at least 3 corners, got {}",
        corners.len()
    );
    let n = corners.len();
    let darts: Vec<Dart> = (0..2 * n).map(|_| g.add_dart()).collect();

    for i in 0..n {
        g.sew(Dim::Zero, darts[2 * i], darts[2 * i + 1])
            .expect("fresh dart pair should be alpha0-sewable");
    }
    for i in 0..n {
        let a = darts[2 * i + 1];
        let b = darts[(2 * i + 2) % (2 * n)];
        g.sew(Dim::One, a, b)
            .expect("fresh dart pair should be alpha1-sewable");
    }

    for i in 0..n {
        let dart = g.cell_representative(darts[2 * i], Dim::Zero);
        g.add_vertex(VertexAttr::new(dart, corners[i], P::V::default()));
    }

    for i in 0..n {
        let edge_dart = g.cell_representative(darts[2 * i], Dim::One);
        let curve = Curve::Line(Line::new(corners[i], corners[(i + 1) % n]));
        g.add_edge(EdgeAttr::new(edge_dart, curve, P::E::default()));
    }
    darts[0]
}
