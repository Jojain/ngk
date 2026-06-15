use nalgebra::Vector3;
use ngk::builders::profiles::add_polyline;
use ngk::builders::sheets::add_extruded_profile;
use ngk::geometry::Point3;
use ngk::topology::gmap::{Dim, GMap};
use ngk::topology::payload::StandardPayload;

#[test]
fn alpha2_detach_duplicates_edge_and_vertex_attributes_for_separated_orbits() {
    let mut g = GMap::<StandardPayload>::new();
    let profile = add_polyline(
        &mut g,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ],
    )
    .expect("profile should build");
    add_extruded_profile(&mut g, profile, Vector3::z()).expect("sheet should build");
    let edge = g
        .iter_edges()
        .find(|(_, edge)| {
            g.orbit(edge.dart, g.orbit_indices(Dim::One))
                .any(|dart| !g.is_free(dart, Dim::Two))
        })
        .map(|(key, _)| key)
        .expect("shared edge should exist");
    let dart = g.edge(edge).expect("edge should exist").dart;
    let vertex_count = g.iter_vertices().count();
    let edge_count = g.iter_edges().count();

    let detached = g.detach(Dim::Two, dart).expect("shared edge should detach");

    assert_eq!(detached.new_edges.len(), 1);
    assert_eq!(detached.new_vertices.len(), 2);
    assert_eq!(g.iter_edges().count(), edge_count + 1);
    assert_eq!(g.iter_vertices().count(), vertex_count + 2);
}
