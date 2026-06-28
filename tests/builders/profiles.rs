use ngk::builders::edges::add_line;
use ngk::builders::profiles::{PolylineError, add_polyline, add_rectangle, append_edge};
use ngk::geometry::{Plane, Point3};
use ngk::topology::closed::Closeable;
use ngk::topology::gmap::{Dim, GMap};
use ngk::topology::payload::StandardPayload;

#[test]
fn add_rectangle_creates_closed_four_edge_profile() {
    let mut g = GMap::<StandardPayload>::new();
    let key = add_rectangle(&mut g, Plane::xy(), 2.0, 3.0).expect("rectangle should build");
    let profile = g.profile_unchecked(key);

    assert!(profile.is_closed());
    assert_eq!(profile.edges().len(), 4);
    assert_eq!(profile.vertices().len(), 4);
    assert_eq!(g.iter_edges().count(), 4);
    assert_eq!(g.iter_vertices().count(), 4);
    assert_eq!(g.cells(Dim::Zero).count(), 4);
}

#[test]
fn add_polyline_creates_valid_profile() {
    let mut g = GMap::<StandardPayload>::new();
    let points = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
    ];

    let key = add_polyline(&mut g, &points).expect("open polyline should build");
    let profile = g.profile_unchecked(key);

    assert!(!profile.is_closed());
    assert_eq!(profile.edges().len(), 2);
    assert_eq!(profile.darts().count(), 4);
    assert_eq!(g.iter_vertices().count(), 3);
    assert_eq!(g.cells(Dim::Zero).count(), 3);
}

#[test]
fn append_edge_appends_contiguous_edge_without_duplicate_vertex() {
    let mut g = GMap::<StandardPayload>::new();
    let profile_key = add_polyline(
        &mut g,
        &[Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)],
    )
    .expect("open profile should build");
    let edge_key = add_line(
        &mut g,
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
    )
    .expect("edge should build");

    append_edge(&mut g, profile_key, edge_key).expect("edge should append");

    let profile = g.profile_unchecked(profile_key);
    assert!(!profile.is_closed());
    assert_eq!(profile.edges().len(), 2);
    assert_eq!(g.iter_edges().count(), 2);
    assert_eq!(g.iter_vertices().count(), 3);
    assert_eq!(g.cells(Dim::Zero).count(), 3);
}

#[test]
fn append_edge_closes_profile_without_duplicate_vertices() {
    let mut g = GMap::<StandardPayload>::new();
    let profile_key = add_polyline(
        &mut g,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ],
    )
    .expect("open profile should build");
    let edge_key = add_line(
        &mut g,
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(0.0, 0.0, 0.0),
    )
    .expect("edge should build");

    append_edge(&mut g, profile_key, edge_key).expect("edge should close profile");

    let profile = g.profile_unchecked(profile_key);
    assert!(profile.is_closed());
    assert_eq!(profile.edges().len(), 3);
    assert_eq!(g.iter_edges().count(), 3);
    assert_eq!(g.iter_vertices().count(), 3);
    assert_eq!(g.cells(Dim::Zero).count(), 3);
}

#[test]
fn append_edge_accepts_reversed_edge_orientation() {
    let mut g = GMap::<StandardPayload>::new();
    let profile_key = add_polyline(
        &mut g,
        &[Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)],
    )
    .expect("open profile should build");
    let edge_key = add_line(
        &mut g,
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
    )
    .expect("edge should build");

    append_edge(&mut g, profile_key, edge_key).expect("reversed edge should append");

    let profile = g.profile_unchecked(profile_key);
    assert!(!profile.is_closed());
    assert_eq!(profile.edges().len(), 2);
    assert_eq!(g.iter_edges().count(), 2);
    assert_eq!(g.iter_vertices().count(), 3);
    assert_eq!(g.cells(Dim::Zero).count(), 3);
}

#[test]
fn add_rectangle_rejects_invalid_sizes() {
    assert_eq!(
        add_rectangle(&mut GMap::<StandardPayload>::new(), Plane::xy(), 0.0, 1.0)
            .expect_err("zero x should fail"),
        PolylineError::InvalidRectangleSize {
            axis: "x",
            value: 0.0,
        }
    );
    assert!(matches!(
        add_rectangle(&mut GMap::<StandardPayload>::new(), Plane::xy(), 1.0, f64::NAN),
        Err(PolylineError::InvalidRectangleSize { axis: "y", value }) if value.is_nan()
    ));
}
