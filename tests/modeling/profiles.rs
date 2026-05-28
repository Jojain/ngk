use ngk::builders::profiles::PolylineError;
use ngk::geometry::Plane;
use ngk::geometry::Point3;
use ngk::modeling::edges;
use ngk::modeling::profiles;
use ngk::topology::closed::Closeable;

#[test]
fn rectangle_returns_owned_profile_shape() {
    let shape = profiles::rectangle(Plane::xy(), 2.0, 3.0).expect("profile should build");
    let profile = shape.profile();

    assert!(profile.is_closed());
    assert_eq!(profile.edges().len(), 4);
}

#[test]
fn polyline_returns_owned_open_profile_shape() {
    let points = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
    ];

    let shape = profiles::polyline(&points).expect("profile should build");
    let profile = shape.profile();

    assert!(!profile.is_closed());
    assert_eq!(profile.edges().len(), 2);
}

#[test]
fn polygon_returns_owned_closed_profile_shape() {
    let points = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    ];

    let shape = profiles::polygon(&points).expect("profile should build");
    let profile = shape.profile();

    assert!(profile.is_closed());
    assert_eq!(profile.edges().len(), 4);
}

#[test]
fn add_copies_an_edge_shape_into_the_profile() {
    let arc = edges::arc(Plane::xy(), 2.0, 0.0, std::f64::consts::FRAC_PI_2)
        .expect("arc edge should build");
    let start = *arc.edge().start().point().expect("arc start");
    let end = *arc.edge().end().point().expect("arc end");
    let closing_edge = edges::line(end, start).expect("closing edge should build");
    let mut shape = arc.into_profile();

    shape.add(&closing_edge).expect("edge should be added");
    let profile = shape.profile();

    assert!(profile.is_closed());
    assert_eq!(profile.edges().len(), 2);
    assert_eq!(closing_edge.map().iter_edges().count(), 1);
}

#[test]
fn add_rejects_closed_profiles() {
    let mut profile = profiles::polygon(&[
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
    ])
    .expect("closed profile should build");
    let edge = edges::line(Point3::new(1.0, 1.0, 0.0), Point3::new(2.0, 1.0, 0.0))
        .expect("edge should build");
    let edge_count = profile.map().iter_edges().count();

    let err = profile
        .add(&edge)
        .expect_err("closed profile should reject add");

    assert!(matches!(err, PolylineError::ClosedProfile { .. }));
    assert_eq!(profile.map().iter_edges().count(), edge_count);
}
