use ngk::geometry::Plane;
use ngk::modeling::profiles;
use ngk::topology::closed::Closeable;

#[test]
fn rectangle_returns_owned_profile_shape() {
    let shape = profiles::rectangle(Plane::xy(), 2.0, 3.0).expect("profile should build");
    let profile = shape.profile();

    assert!(profile.is_closed());
    assert_eq!(profile.edges().len(), 4);
}
