use nalgebra::Vector3;
use ngk::geometry::{Frame, LINEAR_TOLERANCE, Point3, PointCoincidence};

fn assert_point_near(actual: Point3, expected: Point3) {
    assert!(
        actual.coincides(expected, LINEAR_TOLERANCE),
        "expected {expected:?}, got {actual:?}"
    );
}

#[test]
fn frame_coordinates_of_project_point_onto_frame_axes() {
    let frame = Frame::from_xy(
        Point3::new(10.0, 20.0, 30.0),
        Vector3::new(1.0, 1.0, 0.0),
        Vector3::new(-1.0, 1.0, 0.0),
    );
    let local = Vector3::new(2.0, 3.0, 4.0);
    let point = frame.point_at(local);

    assert!((frame.coordinates_of(point) - local).norm() <= LINEAR_TOLERANCE);
}

#[test]
fn frame_point_at_reconstructs_world_point() {
    let frame = Frame::from_xy(Point3::new(1.0, 2.0, 3.0), Vector3::x(), Vector3::z());

    assert_point_near(
        frame.point_at(Vector3::new(2.0, 3.0, 4.0)),
        Point3::new(3.0, -2.0, 6.0),
    );
}
