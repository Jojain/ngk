use ngk::builders::boolean::{BooleanError, SplitSide};
use ngk::builders::profiles::PolylineError;
use ngk::geometry::Plane;
use ngk::geometry::Point3;
use ngk::modeling::edges;
use ngk::modeling::profiles;
use ngk::modeling::split;
use ngk::topology::closed::Closeable;
use radians::Rad64;

use crate::quad_sheet::quad_sheet;

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
fn from_edges_orders_connected_edge_shapes() {
    let first = edges::line(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0))
        .expect("first edge should build");
    let second = edges::line(Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0))
        .expect("second edge should build");
    let third = edges::line(Point3::new(1.0, 1.0, 0.0), Point3::new(0.0, 1.0, 0.0))
        .expect("third edge should build");

    let profile = profiles::from_edges(&[&second, &third, &first])
        .expect("connected edge shapes should build a profile");

    assert_eq!(profile.profile().edges().len(), 3);
    assert_eq!(first.model().iter_edges().count(), 1);
}

#[test]
fn appended_returns_a_profile_with_the_new_edge() {
    let first = edges::line(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0))
        .expect("first edge should build");
    let second = edges::line(Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0))
        .expect("second edge should build");
    let profile = first.into_profile();

    let extended = profiles::appended(profile, &second).expect("connected edge should append");
    assert_eq!(extended.profile().edges().len(), 2);
}

#[test]
fn add_copies_an_edge_shape_into_the_profile() {
    let arc = edges::arc(Plane::xy(), 2.0, Rad64::ZERO, Rad64::QUARTER_TURN)
        .expect("arc edge should build");
    let start = *arc.edge().bounded_unchecked().start().point();
    let end = *arc.edge().bounded_unchecked().end().point();
    let closing_edge = edges::line(end, start).expect("closing edge should build");
    let mut shape = arc.into_profile();

    shape.add(&closing_edge).expect("edge should be added");
    let profile = shape.profile();

    assert!(profile.is_closed());
    assert_eq!(profile.edges().len(), 2);
    assert_eq!(closing_edge.model().iter_edges().count(), 1);
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
    let edge_count = profile.model().iter_edges().count();

    let err = profile
        .add(&edge)
        .expect_err("closed profile should reject add");

    assert!(matches!(err, PolylineError::ClosedProfile { .. }));
    assert_eq!(profile.model().iter_edges().count(), edge_count);
}

/// The plane x = `x`, facing `+x`.
fn wall(x: f64) -> ngk::topology::shape::Shape<ngk::topology::shape::SheetTag> {
    quad_sheet(&[[
        (x, -1.0, -1.0),
        (x, 3.0, -1.0),
        (x, 3.0, 1.0),
        (x, -1.0, 1.0),
    ]])
}

#[test]
fn split_cuts_a_polyline_where_it_crosses_the_sheet() {
    let target = profiles::polyline(&[
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
        Point3::new(2.0, 2.0, 0.0),
    ])
    .expect("polyline should build");

    let parts = split(&target, &wall(1.0)).expect("the plane should split the polyline");

    assert!((parts.front.profile().length() - 3.0).abs() <= 1e-9);
    assert!((parts.back.profile().length() - 1.0).abs() <= 1e-9);
    assert_eq!(parts.front.profile().edges().len(), 2);
    assert!((target.profile().length() - 4.0).abs() <= 1e-9);
}

#[test]
fn split_opens_a_closed_profile_into_two() {
    let target = profiles::square(Plane::xy(), 2.0).expect("square should build");

    let parts = split(&target, &wall(0.5)).expect("the plane should split the square");

    assert!((parts.front.profile().length() - 5.0).abs() <= 1e-9);
    assert!((parts.back.profile().length() - 3.0).abs() <= 1e-9);
    assert!(!parts.front.profile().is_closed());
}

#[test]
fn split_refuses_a_profile_side_that_falls_in_two() {
    let target = profiles::polyline(&[Point3::new(0.0, 0.5, 0.0), Point3::new(3.0, 0.5, 0.0)])
        .expect("polyline should build");
    // A channel over 1 <= x <= 2, its faces turned in.
    let channel = quad_sheet(&[
        [
            (1.0, -1.0, -1.0),
            (1.0, 2.0, -1.0),
            (1.0, 2.0, 1.0),
            (1.0, -1.0, 1.0),
        ],
        [
            (1.0, -1.0, 1.0),
            (1.0, 2.0, 1.0),
            (2.0, 2.0, 1.0),
            (2.0, -1.0, 1.0),
        ],
        [
            (2.0, -1.0, -1.0),
            (2.0, -1.0, 1.0),
            (2.0, 2.0, 1.0),
            (2.0, 2.0, -1.0),
        ],
    ]);

    assert!(matches!(
        split(&target, &channel),
        Err(BooleanError::SplitSideDisconnected {
            side: SplitSide::Back,
            components: 2
        })
    ));
}
