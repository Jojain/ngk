use nalgebra::Vector3;
use ngk::geometry::axis::Axis3;
use ngk::geometry::{Circle, Point3};

fn assert_point_near(actual: Point3, expected: Point3) {
    let err = (actual - expected).norm();
    assert!(
        err < 1e-10,
        "expected {expected:?}, got {actual:?}, err={err}"
    );
}

#[test]
fn circle_from_axis_handles_z_axis() {
    let circle = Circle::from_axis(Axis3::new(Point3::new(1.0, 2.0, 3.0), Vector3::z()), 2.0);

    assert_point_near(circle.point_at(0.0), Point3::new(-1.0, 2.0, 3.0));
}
