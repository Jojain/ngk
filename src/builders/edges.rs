use crate::geometry::{Curve, Point3};
use crate::topology::attributes::{EdgeAttr, VertexAttr};
use crate::topology::gmap::{Dart, Dim, GMap};
use crate::topology::payload::Payload;
use crate::topology::shape_keys::EdgeKey;

pub fn add_edge<P: Payload>(
    g: &mut GMap<P>,
    start: Point3,
    end: Point3,
    curve: Curve,
) -> (Dart, EdgeKey) {
    let d1 = g.add_dart();
    let d2 = g.add_dart();
    g.sew_unchecked(Dim::Zero, d1, d2);
    g.add_vertex(VertexAttr::new(d1, start, P::V::default()));
    g.add_vertex(VertexAttr::new(d2, end, P::V::default()));
    let e = g.add_edge(EdgeAttr::new(d1, curve, P::E::default()));
    (d1, e)
}
