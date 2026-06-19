use ngk::builders::profiles::add_rectangle;
use ngk::geometry::Plane;
use ngk::topology::closed::Closed;
use ngk::topology::gmap::GMap;
use ngk::topology::payload::StandardPayload;

#[test]
fn closed_profile_corners_pair_each_vertex_with_ordered_incident_edges() {
    let mut g = GMap::<StandardPayload>::new();
    let key = add_rectangle(&mut g, Plane::xy(), 2.0, 3.0).expect("rectangle should build");
    let profile = Closed::new(g.profile(key).expect("profile should exist"))
        .expect("rectangle should be closed");
    let edges = profile.edges();
    let corners = profile.corners();

    assert_eq!(corners.len(), edges.len());

    for (index, corner) in corners.iter().enumerate() {
        let previous = (index + edges.len() - 1) % edges.len();

        assert_eq!(corner.incoming().dart(), edges[previous].dart());
        assert_eq!(corner.outgoing().dart(), edges[index].dart());
        assert_eq!(corner.vertex().key(), edges[index].start().key());
        assert_eq!(corner.vertex().key(), edges[previous].end().key());
    }
}
