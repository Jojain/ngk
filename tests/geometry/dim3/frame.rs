use nalgebra::Vector3;
use ngk::geometry::{Frame, FrameError, LINEAR_TOLERANCE, Point3, PointCoincidence};

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

#[test]
fn try_from_xz_refuses_parallel_directions() {
    let frame = Frame::try_from_xz(Point3::origin(), Vector3::x(), Vector3::new(2.0, 0.0, 0.0));

    assert_eq!(frame, Err(FrameError::ParallelDirections));
}

#[test]
fn try_from_xz_refuses_opposite_directions() {
    let frame = Frame::try_from_xz(Point3::origin(), Vector3::z(), -Vector3::z());

    assert_eq!(frame, Err(FrameError::ParallelDirections));
}

#[test]
fn try_from_xz_refuses_a_zero_direction() {
    let zero_x = Frame::try_from_xz(Point3::origin(), Vector3::zeros(), Vector3::z());
    let zero_z = Frame::try_from_xz(Point3::origin(), Vector3::x(), Vector3::zeros());

    assert_eq!(zero_x, Err(FrameError::ZeroDirection));
    assert_eq!(zero_z, Err(FrameError::ZeroDirection));
}

#[test]
fn try_from_xz_builds_the_frame_its_directions_describe() {
    let frame = Frame::try_from_xz(
        Point3::new(1.0, 2.0, 3.0),
        Vector3::new(2.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, 3.0),
    )
    .unwrap();

    assert_eq!(frame.origin, Point3::new(1.0, 2.0, 3.0));
    assert_eq!(*frame.x_dir, Vector3::new(1.0, 0.0, 0.0));
    assert_eq!(*frame.y_dir, Vector3::new(0.0, 1.0, 0.0));
    assert_eq!(*frame.z_dir, Vector3::new(0.0, 0.0, 1.0));
}

#[test]
fn try_from_xy_refuses_directions_that_span_no_plane() {
    let origin = Point3::origin();

    assert_eq!(
        Frame::try_from_xy(origin, Vector3::y(), Vector3::new(0.0, 5.0, 0.0)),
        Err(FrameError::ParallelDirections)
    );
    assert_eq!(
        Frame::try_from_xy(origin, Vector3::y(), -Vector3::y()),
        Err(FrameError::ParallelDirections)
    );
    assert_eq!(
        Frame::try_from_xy(origin, Vector3::zeros(), Vector3::y()),
        Err(FrameError::ZeroDirection)
    );
    assert_eq!(
        Frame::try_from_xy(origin, Vector3::x(), Vector3::zeros()),
        Err(FrameError::ZeroDirection)
    );
}

#[test]
fn try_from_xy_builds_the_frame_its_directions_describe() {
    let frame = Frame::try_from_xy(
        Point3::origin(),
        Vector3::new(0.0, 4.0, 0.0),
        Vector3::new(0.0, 0.0, 0.5),
    )
    .unwrap();

    assert_eq!(*frame.x_dir, Vector3::new(0.0, 1.0, 0.0));
    assert_eq!(*frame.y_dir, Vector3::new(0.0, 0.0, 1.0));
    assert_eq!(*frame.z_dir, Vector3::new(1.0, 0.0, 0.0));
}

#[test]
#[should_panic(expected = "frame directions are parallel")]
fn from_xz_panics_on_parallel_directions() {
    Frame::from_xz(Point3::origin(), Vector3::x(), Vector3::x());
}

#[test]
#[should_panic(expected = "frame direction is zero")]
fn from_xy_panics_on_a_zero_direction() {
    Frame::from_xy(Point3::origin(), Vector3::zeros(), Vector3::y());
}

#[test]
fn try_from_yz_refuses_directions_that_span_no_plane() {
    let origin = Point3::origin();

    assert_eq!(
        Frame::try_from_yz(origin, Vector3::y(), Vector3::new(0.0, 5.0, 0.0)),
        Err(FrameError::ParallelDirections)
    );
    assert_eq!(
        Frame::try_from_yz(origin, Vector3::zeros(), Vector3::z()),
        Err(FrameError::ZeroDirection)
    );
    assert_eq!(
        Frame::try_from_yz(origin, Vector3::y(), Vector3::zeros()),
        Err(FrameError::ZeroDirection)
    );
}

#[test]
fn try_from_yz_builds_the_frame_its_directions_describe() {
    let frame = Frame::try_from_yz(
        Point3::new(1.0, 2.0, 3.0),
        Vector3::new(0.0, 2.0, 0.0),
        Vector3::new(0.0, 0.0, 3.0),
    )
    .unwrap();

    assert_eq!(frame.origin, Point3::new(1.0, 2.0, 3.0));
    assert_eq!(*frame.x_dir, Vector3::new(1.0, 0.0, 0.0));
    assert_eq!(*frame.y_dir, Vector3::new(0.0, 1.0, 0.0));
    assert_eq!(*frame.z_dir, Vector3::new(0.0, 0.0, 1.0));
}

#[test]
fn try_from_yz_keeps_z_and_projects_y_onto_the_plane_perpendicular_to_it() {
    let frame = Frame::try_from_yz(
        Point3::origin(),
        Vector3::new(0.0, 1.0, 1.0),
        Vector3::new(0.0, 0.0, 1.0),
    )
    .unwrap();

    assert!((*frame.z_dir - Vector3::z()).norm() <= LINEAR_TOLERANCE);
    assert!((*frame.y_dir - Vector3::y()).norm() <= LINEAR_TOLERANCE);
    assert!((*frame.x_dir - Vector3::x()).norm() <= LINEAR_TOLERANCE);
}

#[test]
#[should_panic(expected = "frame directions are parallel")]
fn from_yz_panics_on_parallel_directions() {
    Frame::from_yz(Point3::origin(), Vector3::y(), Vector3::y());
}
