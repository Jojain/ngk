//! Higher-level helpers that mutate a [`GMap`]. Kept separate from the gmap
//! itself so the combinatorial core stays small; anything opinionated (how to
//! build a polygon, how to stitch cells, etc.) lives here.

use crate::StandardPayload;
use crate::topology::gmap::{Dart, GMap};

pub fn make_polygon(sides_count: usize) -> GMap<'static, StandardPayload> {
    let n = sides_count;
    let mut g = GMap::<StandardPayload>::new();
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
    g
}

pub fn make_cube() -> GMap<'static, StandardPayload> {
    let mut cube = GMap::<StandardPayload>::new();
    let f1 = make_polygon(4);
    let f2 = make_polygon(4);
    let f3 = make_polygon(4);
    let f4 = make_polygon(4);
    let f5 = make_polygon(4);
    let f6 = make_polygon(4);
    cube
}
