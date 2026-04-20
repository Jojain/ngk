//! Higher-level helpers that mutate a [`GMap`]. Kept separate from the gmap
//! itself so the combinatorial core stays small; anything opinionated (how to
//! build a polygon, how to stitch cells, etc.) lives here.

use crate::geometry::utils::Point3;
use crate::topology::attributes::VertexAttr;
use crate::topology::gmap::{Cell0, Dart, GMap};
use crate::topology::payload::Payload;

/// Adds a single polygon face to `g` with the given corner points (in order).
///
/// Sews α0 and α1 to form a closed `n`-gon and stamps the vertex positions on
/// every dart of each corner's vertex orbit. Does not touch α2 — the face is
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
        g.sew(0, darts[2 * i], darts[2 * i + 1])
            .expect("fresh dart pair should be alpha0-sewable");
    }
    for i in 0..n {
        let a = darts[2 * i + 1];
        let b = darts[(2 * i + 2) % (2 * n)];
        g.sew(1, a, b)
            .expect("fresh dart pair should be alpha1-sewable");
    }

    for i in 0..n {
        g.attribute_mut_or_insert_with::<Cell0, _>(darts[2 * i], || VertexAttr {
            point: corners[i],
            data: P::V::default(),
        });
    }
    darts[0]
}
