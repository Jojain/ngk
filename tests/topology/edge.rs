use ngk::builders::edges::add_circle;
use ngk::geometry::Plane;
use ngk::topology::edge::Edge;
use ngk::topology::gmap::{Dim, GMap};
use ngk::topology::payload::StandardPayload;

#[test]
fn closed_edge_darts_resolve_opposite_orientations() {
    let mut g = GMap::<StandardPayload>::new();
    let edge_key = add_circle(&mut g, Plane::xy(), 1.0).expect("circle edge should build");
    let default_dart = g.edge_attr_unchecked(edge_key).dart;
    let reversed_dart = g.alpha(Dim::Zero, default_dart);

    let default_edge =
        Edge::from_dart(&g, default_dart).expect("default dart should resolve its edge");
    let reversed_edge =
        Edge::from_dart(&g, reversed_dart).expect("reversed dart should resolve its edge");

    assert_eq!(default_edge.key(), edge_key);
    assert_eq!(reversed_edge.key(), edge_key);
    assert_eq!(default_edge.dart(), default_dart);
    assert_eq!(reversed_edge.dart(), reversed_dart);
    assert_eq!(default_edge.start().key(), reversed_edge.end().key());
    assert_eq!(default_edge.end().key(), reversed_edge.start().key());
}
