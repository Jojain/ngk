use ngk::geometry::{Axis3, Point3};
use ngk::modeling::{edges, revolve};
use radians::Rad64;

#[test]
fn revolving_an_owned_edge_returns_one_face() {
    let edge = edges::line(Point3::new(2.0, 0.0, 0.0), Point3::new(2.0, 0.0, 1.0))
        .expect("line should build");
    let face =
        revolve::revolve_edge(edge, Axis3::z(), Rad64::new(0.5)).expect("edge should revolve");
    assert_eq!(face.model().iter_faces().count(), 1);
}
