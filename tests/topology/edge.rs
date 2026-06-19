use ngk::builders::edges::add_circle;
use ngk::geometry::Plane;
use ngk::topology::Orientation;
use ngk::topology::edge::Edge;
use ngk::topology::gmap::{Dim, GMap};
use ngk::topology::payload::StandardPayload;

#[test]
fn closed_edge_darts_resolve_opposite_orientations() {
    let mut g = GMap::<StandardPayload>::new();
    let edge_key = add_circle(&mut g, Plane::xy(), 1.0).expect("circle edge should build");
    let default_dart = g
        .edge_attr(edge_key)
        .expect("circle edge should exist")
        .dart;
    let reversed_dart = g.alpha(Dim::Zero, default_dart);

    let default_edge =
        Edge::from_dart(&g, default_dart).expect("default dart should resolve its edge");
    let reversed_edge =
        Edge::from_dart(&g, reversed_dart).expect("reversed dart should resolve its edge");

    assert_eq!(default_edge.key(), edge_key);
    assert_eq!(default_edge.orientation, Orientation::Same);
    assert_eq!(reversed_edge.key(), edge_key);
    assert_eq!(reversed_edge.orientation, Orientation::Reversed);
}
