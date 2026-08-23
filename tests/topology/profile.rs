use ngk::builders::profiles::add_rectangle;
use ngk::geometry::{LINEAR_TOLERANCE, Plane, Point3, PointCoincidence};
use ngk::topology::closed::Closed;
use ngk::topology::gmap::{Dim, GMap};
use ngk::topology::payload::StandardPayload;
use ngk::topology::profile::Profile;

#[test]
fn closed_profile_corners_pair_each_vertex_with_ordered_incident_edges() {
    let mut g = GMap::<StandardPayload>::new();
    let key = add_rectangle(&mut g, Plane::xy(), 2.0, 3.0).expect("rectangle should build");
    let profile = Closed::new(g.profile_unchecked(key)).expect("rectangle should be closed");
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

#[test]
fn rectangle_profile_traverses_corners_and_edges_in_geometric_order() {
    let mut g = GMap::<StandardPayload>::new();
    let key = add_rectangle(&mut g, Plane::xy(), 2.0, 3.0).expect("rectangle should build");
    let profile = g.profile_unchecked(key);
    let expected = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
        Point3::new(2.0, 3.0, 0.0),
        Point3::new(0.0, 3.0, 0.0),
    ];

    assert_profile_points(&profile, &expected);

    for (edge, next) in profile
        .edges()
        .iter()
        .zip(profile.edges().iter().cycle().skip(1))
        .take(expected.len())
    {
        assert_eq!(
            edge.end().key(),
            next.start().key(),
            "consecutive oriented edges should share end and start vertices"
        );
    }
}

#[test]
fn alpha0_of_rectangle_profile_seed_traverses_the_same_corners_in_reverse() {
    let mut g = GMap::<StandardPayload>::new();
    let key = add_rectangle(&mut g, Plane::xy(), 2.0, 3.0).expect("rectangle should build");
    let seed = g.profile_attr_unchecked(key).dart;
    let reversed = Profile::from_dart(&g, g.alpha(Dim::Zero, seed))
        .expect("reversed seed should resolve the same profile");
    let expected = [
        Point3::new(2.0, 0.0, 0.0),
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(0.0, 3.0, 0.0),
        Point3::new(2.0, 3.0, 0.0),
    ];

    assert_profile_points(&reversed, &expected);
}

fn assert_profile_points(profile: &Profile<'_>, expected: &[Point3]) {
    let vertices = profile.vertices();
    assert_eq!(vertices.len(), expected.len());

    for (index, (vertex, expected)) in vertices.iter().zip(expected).enumerate() {
        let actual = vertex
            .point()
            .expect("rectangle profile vertices should have geometry");
        assert!(
            actual.coincides(expected, LINEAR_TOLERANCE),
            "profile vertex {index} should be {expected:?}, got {actual:?}"
        );
    }
}
