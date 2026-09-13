use ngk::builders::edges::add_circle;
use ngk::geometry::Plane;
use ngk::model::Model;
use ngk::topology::edge::Edge;
use ngk::topology::gmap::Dim;
use ngk::topology::payload::StandardPayload;

#[test]
fn closed_edge_darts_resolve_opposite_orientations() {
    let mut g = Model::<StandardPayload>::new();
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
    // Both darts report the edge closed, from either direction. This used to
    // compare `start` against `end` across the two views, which a circle answers
    // vacuously — its ends are one vertex, so either comparison held whatever
    // the darts did. What the two views really disagree about is direction, and
    // that is asserted above; what they agree on is that there is no arc here to
    // name with endpoints.
    assert!(matches!(default_edge, Edge::Closed(_)));
    assert!(matches!(reversed_edge, Edge::Closed(_)));
    assert!(
        default_edge.bounded().is_none(),
        "a closed edge has no endpoints to hand out"
    );
    assert_eq!(
        default_edge.vertices().len(),
        1,
        "the circle's two ends meet at one vertex"
    );
}
