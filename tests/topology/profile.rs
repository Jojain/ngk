use ngk::builders::profiles::add_rectangle;
use ngk::geometry::{LINEAR_TOLERANCE, Plane, Point3, PointCoincidence};
use ngk::model::Model;
use ngk::modeling::profiles::polyline;
use ngk::topology::closed::Closed;
use ngk::topology::gmap::Dim;
use ngk::topology::payload::StandardPayload;
use ngk::topology::profile::Profile;

#[test]
fn closed_profile_corners_pair_each_vertex_with_ordered_incident_edges() {
    let mut g = Model::<StandardPayload>::new();
    let key = add_rectangle(&mut g, Plane::xy(), 2.0, 3.0).expect("rectangle should build");
    let profile = Closed::new(g.profile_unchecked(key)).expect("rectangle should be closed");
    let edges = profile.edges();
    let corners = profile.corners();

    assert_eq!(corners.len(), edges.len());

    for (index, corner) in corners.iter().enumerate() {
        let previous = (index + edges.len() - 1) % edges.len();

        assert_eq!(corner.incoming().dart(), edges[previous].dart());
        assert_eq!(corner.outgoing().dart(), edges[index].dart());
        assert_eq!(
            corner.vertex().key(),
            edges[index].bounded_unchecked().start().key()
        );
        assert_eq!(
            corner.vertex().key(),
            edges[previous].bounded_unchecked().end().key()
        );
    }
}

#[test]
fn rectangle_profile_traverses_corners_and_edges_in_geometric_order() {
    let mut g = Model::<StandardPayload>::new();
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
            edge.bounded_unchecked().end().key(),
            next.bounded_unchecked().start().key(),
            "consecutive oriented edges should share end and start vertices"
        );
    }
}

#[test]
fn alpha0_of_rectangle_profile_seed_traverses_the_same_corners_in_reverse() {
    let mut g = Model::<StandardPayload>::new();
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

#[test]
fn an_open_profile_lists_every_vertex_once_in_traversal_order() {
    let path = polyline(&[
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(0.0, 0.0, 4.0),
        Point3::new(4.0, 0.0, 4.0),
    ])
    .expect("polyline should build");
    let profile = path.profile();

    assert_eq!(profile.edges().len(), 2);
    assert_profile_points(
        &profile,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 4.0),
            Point3::new(4.0, 0.0, 4.0),
        ],
    );
    assert!(
        profile
            .end()
            .point()
            .coincides(&Point3::new(4.0, 0.0, 4.0), LINEAR_TOLERANCE)
    );
}

#[test]
fn a_reversed_open_profile_runs_the_whole_chain_back_to_its_start() {
    let path = polyline(&[
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(0.0, 0.0, 4.0),
        Point3::new(4.0, 0.0, 4.0),
    ])
    .expect("polyline should build");
    let profile = path.profile();
    let reversed = profile.reversed();

    assert_eq!(reversed.edges().len(), 2);
    assert_profile_points(
        &reversed,
        &[
            Point3::new(4.0, 0.0, 4.0),
            Point3::new(0.0, 0.0, 4.0),
            Point3::new(0.0, 0.0, 0.0),
        ],
    );
    for edge in reversed.edges() {
        let bounded = edge.bounded_unchecked();
        assert!(
            bounded.start().point().z >= bounded.end().point().z
                && bounded.start().point().x >= bounded.end().point().x,
            "each edge of the reversed view should run back towards the start"
        );
    }
    assert_eq!(reversed.reversed().dart, profile.dart);
}

#[test]
fn a_reversed_closed_profile_keeps_every_edge_and_vertex() {
    let mut g = Model::<StandardPayload>::new();
    let key = add_rectangle(&mut g, Plane::xy(), 2.0, 3.0).expect("rectangle should build");
    let profile = g.profile_unchecked(key);
    let reversed = profile.reversed();

    assert_eq!(profile.vertices().len(), 4);
    assert_eq!(reversed.edges().len(), 4);
    assert_eq!(reversed.vertices().len(), 4);
    assert_eq!(reversed.reversed().dart, profile.dart);
}

fn assert_profile_points(profile: &Profile<'_>, expected: &[Point3]) {
    let vertices = profile.vertices();
    assert_eq!(vertices.len(), expected.len());

    for (index, (vertex, expected)) in vertices.iter().zip(expected).enumerate() {
        let actual = vertex.point();
        assert!(
            actual.coincides(expected, LINEAR_TOLERANCE),
            "profile vertex {index} should be {expected:?}, got {actual:?}"
        );
    }
}
