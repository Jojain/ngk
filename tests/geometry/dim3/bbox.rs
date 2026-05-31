use nalgebra::Vector3;
use ngk::geometry::{BBox, LINEAR_TOLERANCE, Point3, PointCoincidence};

fn assert_point_near(actual: Point3, expected: Point3) {
    assert!(
        actual.coincides(expected, LINEAR_TOLERANCE),
        "expected {expected:?}, got {actual:?}"
    );
}

fn assert_scalar_near(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() <= LINEAR_TOLERANCE,
        "expected {expected}, got {actual}"
    );
}

#[test]
fn bbox_from_points_returns_empty_for_empty_point_cloud() {
    assert!(BBox::from_points([]).is_empty());
}

#[test]
fn bbox_from_points_accepts_single_point_as_zero_size_box() {
    let bbox = BBox::from_points([Point3::new(1.0, 2.0, 3.0)]);

    assert_point_near(bbox.frame().unwrap().origin, Point3::new(1.0, 2.0, 3.0));
    assert_point_near(bbox.min().unwrap(), Point3::new(1.0, 2.0, 3.0));
    assert_point_near(bbox.max().unwrap(), Point3::new(1.0, 2.0, 3.0));
    assert_eq!(bbox.size(), Vector3::zeros());
}

#[test]
fn bbox_empty_can_be_extended_into_a_non_empty_box() {
    let mut bbox = BBox::empty();

    bbox.extend(Point3::new(1.0, 2.0, 3.0));

    assert!(!bbox.is_empty());
    assert_point_near(bbox.frame().unwrap().origin, Point3::new(1.0, 2.0, 3.0));
    assert_eq!(bbox.size(), Vector3::zeros());
}

#[test]
fn bbox_corners_are_absent_for_empty_and_fixed_size_for_non_empty_boxes() {
    assert!(BBox::empty().corners().is_none());

    let point = Point3::new(1.0, 2.0, 3.0);
    let corners = BBox::from_points([point]).corners().unwrap();

    assert_eq!(corners.len(), 8);
    for corner in corners {
        assert_point_near(corner, point);
    }
}

#[test]
fn bbox_from_points_orients_frame_from_principal_axes() {
    let center = Point3::new(10.0, 20.0, 30.0);
    let x_axis = Vector3::new(1.0, 1.0, 0.0).normalize();
    let y_axis = Vector3::new(-1.0, 1.0, 0.0).normalize();
    let z_axis = Vector3::z();
    let half_size = Vector3::new(3.0, 1.0, 0.5);
    let mut points = Vec::new();

    for x_sign in [-1.0, 1.0] {
        for y_sign in [-1.0, 1.0] {
            for z_sign in [-1.0, 1.0] {
                points.push(
                    center
                        + x_axis * (x_sign * half_size.x)
                        + y_axis * (y_sign * half_size.y)
                        + z_axis * (z_sign * half_size.z),
                );
            }
        }
    }

    let bbox = BBox::from_points(points);

    assert_point_near(bbox.frame().unwrap().origin, center);
    assert_scalar_near(bbox.x_size(), half_size.x * 2.0);
    assert_scalar_near(bbox.y_size(), half_size.y * 2.0);
    assert_scalar_near(bbox.z_size(), half_size.z * 2.0);
    assert!(bbox.frame().unwrap().x_dir.dot(&x_axis).abs() > 1.0 - LINEAR_TOLERANCE);
    assert!(bbox.frame().unwrap().y_dir.dot(&y_axis).abs() > 1.0 - LINEAR_TOLERANCE);
    assert!(bbox.frame().unwrap().z_dir.dot(&z_axis).abs() > 1.0 - LINEAR_TOLERANCE);
}

#[test]
fn bbox_intersects_detects_separated_boxes() {
    let a = BBox::from_points(axis_aligned_box_points(
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 1.0),
    ));
    let b = BBox::from_points(axis_aligned_box_points(
        Point3::new(3.0, 0.0, 0.0),
        Point3::new(4.0, 1.0, 1.0),
    ));

    assert!(!a.intersects(&b, LINEAR_TOLERANCE));
}

#[test]
fn bbox_intersects_uses_tolerance_for_near_touching_boxes() {
    let a = BBox::from_points(axis_aligned_box_points(
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 1.0),
    ));
    let b = BBox::from_points(axis_aligned_box_points(
        Point3::new(1.05, 0.0, 0.0),
        Point3::new(2.05, 1.0, 1.0),
    ));

    assert!(!a.intersects(&b, 0.01));
    assert!(a.intersects(&b, 0.05));
}

#[test]
fn bbox_expanded_preserves_center_and_grows_size() {
    let bbox = BBox::from_points(axis_aligned_box_points(
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 2.0, 3.0),
    ));
    let center = bbox.center().unwrap();

    let expanded = bbox.expanded(0.25);

    assert_point_near(expanded.center().unwrap(), center);
    assert_vector_near(expanded.size(), bbox.size() + Vector3::repeat(0.5));
}

#[test]
fn bbox_contains_point_checks_oriented_local_coordinates() {
    let center = Point3::new(2.0, -1.0, 0.5);
    let x_axis = Vector3::new(1.0, 1.0, 0.0).normalize();
    let y_axis = Vector3::new(-1.0, 1.0, 0.0).normalize();
    let z_axis = Vector3::z();
    let half_size = Vector3::new(2.0, 1.0, 0.25);
    let mut points = Vec::new();

    for x_sign in [-1.0, 1.0] {
        for y_sign in [-1.0, 1.0] {
            for z_sign in [-1.0, 1.0] {
                points.push(
                    center
                        + x_axis * (x_sign * half_size.x)
                        + y_axis * (y_sign * half_size.y)
                        + z_axis * (z_sign * half_size.z),
                );
            }
        }
    }

    let bbox = BBox::from_points(points);

    assert!(bbox.contains_point(center + x_axis * 0.5 + y_axis * 0.5, LINEAR_TOLERANCE));
    assert!(!bbox.contains_point(center + x_axis * 2.2, LINEAR_TOLERANCE));
}

#[test]
fn bbox_diagonal_length_is_size_norm() {
    let bbox = BBox::from_points(axis_aligned_box_points(
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 2.0, 2.0),
    ));

    assert_scalar_near(bbox.diagonal_length(), bbox.size().norm());
}

fn assert_vector_near(actual: Vector3<f64>, expected: Vector3<f64>) {
    assert!(
        (actual - expected).norm() <= LINEAR_TOLERANCE,
        "expected {expected:?}, got {actual:?}"
    );
}

fn axis_aligned_box_points(min: Point3, max: Point3) -> Vec<Point3> {
    let mut points = Vec::with_capacity(8);
    for x in [min.x, max.x] {
        for y in [min.y, max.y] {
            for z in [min.z, max.z] {
                points.push(Point3::new(x, y, z));
            }
        }
    }
    points
}
