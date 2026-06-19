use ngk::builders::profiles::{PolylineError, add_rectangle};
use ngk::geometry::Plane;
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
