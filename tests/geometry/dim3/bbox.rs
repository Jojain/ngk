use nalgebra::Vector3;
use ngk::geometry::{BBox, Frame, LINEAR_TOLERANCE, Point3, PointCoincidence};

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
fn bbox_of_no_points_is_none() {
    assert!(BBox::from_points([]).is_none());
    assert!(BBox::from_points_in_frame(Frame::xyz(), []).is_none());
}

#[test]
fn bbox_from_points_accepts_single_point_as_zero_size_box() {
    let point = Point3::new(1.0, 2.0, 3.0);
    let bbox = BBox::from_points([point]).expect("one point has a box");

    assert_eq!(bbox.frame().origin, point);
    assert_eq!(bbox.min(), point);
    assert_eq!(bbox.max(), point);
    assert_eq!(bbox.size(), Vector3::zeros());
    for corner in bbox.corners() {
        assert_eq!(corner, point);
    }
}

#[test]
fn bbox_in_a_frame_keeps_that_frame_origin_included() {
    let frame = Frame::from_xy(Point3::new(100.0, 0.0, 0.0), Vector3::x(), Vector3::y());
    let bbox = BBox::from_points_in_frame(
        frame.clone(),
        [Point3::new(1.0, 2.0, 3.0), Point3::new(4.0, 6.0, 8.0)],
    )
    .expect("two points have a box");

    assert_eq!(bbox.frame(), &frame);
    assert_point_near(bbox.min(), Point3::new(1.0, 2.0, 3.0));
    assert_point_near(bbox.max(), Point3::new(4.0, 6.0, 8.0));
    assert_point_near(bbox.center(), Point3::new(2.5, 4.0, 5.5));
}

/// In the world frame the corners are the inputs' component-wise extremes,
/// bit for bit: no centre and half-size round trip stands in between.
#[test]
fn bbox_in_the_world_frame_reads_back_exact_extremes() {
    let low = Point3::new(0.1, -0.7, 1.0e-3);
    let high = Point3::new(0.3, 0.2, 7.9);
    let bbox = BBox::from_points_in_frame(
        Frame::xyz(),
        [
            Point3::new(low.x, high.y, low.z),
            Point3::new(high.x, low.y, high.z),
        ],
    )
    .expect("two points have a box");

    assert_eq!(bbox.min(), low);
    assert_eq!(bbox.max(), high);
}

#[test]
fn bbox_extend_grows_the_ranges_and_keeps_the_frame() {
    let mut bbox =
        BBox::from_points_in_frame(Frame::xyz(), [Point3::new(1.0, 2.0, 3.0)]).expect("a box");

    bbox.extend(Point3::new(-1.0, 5.0, 3.0));

    assert_eq!(bbox.frame(), &Frame::xyz());
    assert_eq!(bbox.min(), Point3::new(-1.0, 2.0, 3.0));
    assert_eq!(bbox.max(), Point3::new(1.0, 5.0, 3.0));
}

#[test]
fn bbox_from_points_orients_frame_from_principal_axes() {
    let center = Point3::new(10.0, 20.0, 30.0);
    let x_axis = Vector3::new(1.0, 1.0, 0.0).normalize();
    let y_axis = Vector3::new(-1.0, 1.0, 0.0).normalize();
    let z_axis = Vector3::z();
    let half_size = Vector3::new(3.0, 1.0, 0.5);

    let bbox = BBox::from_points(oriented_box_points(
        center,
        [x_axis, y_axis, z_axis],
        half_size,
    ))
    .expect("eight points have a box");

    assert_point_near(bbox.frame().origin, center);
    assert_point_near(bbox.center(), center);
    assert_scalar_near(bbox.x_size(), half_size.x * 2.0);
    assert_scalar_near(bbox.y_size(), half_size.y * 2.0);
    assert_scalar_near(bbox.z_size(), half_size.z * 2.0);
    assert!(bbox.frame().x_dir.dot(&x_axis).abs() > 1.0 - LINEAR_TOLERANCE);
    assert!(bbox.frame().y_dir.dot(&y_axis).abs() > 1.0 - LINEAR_TOLERANCE);
    assert!(bbox.frame().z_dir.dot(&z_axis).abs() > 1.0 - LINEAR_TOLERANCE);
}

#[test]
fn bbox_new_spans_the_given_ranges_in_the_given_frame() {
    let frame = Frame::from_xy(Point3::new(1.0, 0.0, 0.0), Vector3::y(), Vector3::z());
    let bbox = BBox::new(
        frame.clone(),
        Vector3::new(0.0, -1.0, 2.0),
        Vector3::new(3.0, 1.0, 2.0),
    );

    assert_eq!(bbox.frame(), &frame);
    assert_eq!(bbox.size(), Vector3::new(3.0, 2.0, 0.0));
    assert_point_near(bbox.min(), Point3::new(3.0, 0.0, -1.0));
    assert_point_near(bbox.max(), Point3::new(3.0, 3.0, 1.0));
}

#[test]
#[should_panic(expected = "ordered")]
fn bbox_new_refuses_a_reversed_range() {
    BBox::new(
        Frame::xyz(),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 1.0),
    );
}

#[test]
fn bbox_corner_picks_one_end_of_each_range() {
    use ngk::geometry::Bound::{Max, Min};
    let bbox = aabb(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 2.0, 3.0));

    assert_eq!(bbox.corner(Min, Min, Min), bbox.min());
    assert_eq!(bbox.corner(Max, Max, Max), bbox.max());
    assert_eq!(bbox.corner(Max, Min, Max), Point3::new(1.0, 0.0, 3.0));
    assert_eq!(bbox.corners()[5], bbox.corner(Max, Min, Max));
}

#[test]
fn bbox_merge_in_one_frame_is_exact() {
    let mut merged = aabb(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
    let other = aabb(Point3::new(0.5, -2.0, 0.25), Point3::new(3.0, 0.5, 0.75));

    merged.merge(&other);

    assert_eq!(merged.frame(), &Frame::xyz());
    assert_eq!(merged.min(), Point3::new(0.0, -2.0, 0.0));
    assert_eq!(merged.max(), Point3::new(3.0, 1.0, 1.0));
}

#[test]
fn bbox_merge_across_frames_keeps_the_first_frame_and_contains_the_second_box() {
    let mut merged = aabb(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
    let tilted = Frame::from_xy(
        Point3::new(3.0, 0.0, 0.0),
        Vector3::new(1.0, 1.0, 0.0),
        Vector3::new(-1.0, 1.0, 0.0),
    );
    let other = BBox::new(tilted, Vector3::zeros(), Vector3::new(1.0, 1.0, 1.0));

    merged.merge(&other);

    assert_eq!(merged.frame(), &Frame::xyz());
    for corner in other.corners() {
        assert!(merged.contains_point(corner, LINEAR_TOLERANCE));
    }
    assert_point_near(merged.min(), Point3::new(0.0, 0.0, 0.0));
    assert_point_near(
        merged.max(),
        Point3::new(3.0 + 1.0 / 2.0_f64.sqrt(), 2.0_f64.sqrt(), 1.0),
    );
}

#[test]
fn bbox_intersects_detects_separated_boxes() {
    let a = aabb(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
    let b = aabb(Point3::new(3.0, 0.0, 0.0), Point3::new(4.0, 1.0, 1.0));

    assert!(!a.intersects(&b, LINEAR_TOLERANCE));
}

#[test]
fn bbox_intersects_uses_tolerance_for_near_touching_boxes() {
    let a = aabb(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
    let b = aabb(Point3::new(1.05, 0.0, 0.0), Point3::new(2.05, 1.0, 1.0));

    assert!(!a.intersects(&b, 0.01));
    assert!(a.intersects(&b, 0.05));
}

#[test]
fn bbox_expanded_grows_both_ends_of_every_range() {
    let bbox = aabb(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 2.0, 3.0));

    let expanded = bbox.expanded(0.25);

    assert_eq!(expanded.frame(), bbox.frame());
    assert_eq!(expanded.min(), Point3::new(-0.25, -0.25, -0.25));
    assert_eq!(expanded.max(), Point3::new(1.25, 2.25, 3.25));
}

#[test]
fn bbox_contains_point_checks_oriented_local_coordinates() {
    let center = Point3::new(2.0, -1.0, 0.5);
    let x_axis = Vector3::new(1.0, 1.0, 0.0).normalize();
    let y_axis = Vector3::new(-1.0, 1.0, 0.0).normalize();
    let z_axis = Vector3::z();
    let half_size = Vector3::new(2.0, 1.0, 0.25);

    let bbox = BBox::from_points(oriented_box_points(
        center,
        [x_axis, y_axis, z_axis],
        half_size,
    ))
    .expect("eight points have a box");

    assert!(bbox.contains_point(center + x_axis * 0.5 + y_axis * 0.5, LINEAR_TOLERANCE));
    assert!(!bbox.contains_point(center + x_axis * 2.2, LINEAR_TOLERANCE));
}

#[test]
fn bbox_diagonal_length_is_size_norm() {
    let bbox = aabb(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 2.0, 2.0));

    assert_scalar_near(bbox.diagonal_length(), 3.0);
}

fn aabb(min: Point3, max: Point3) -> BBox {
    BBox::from_points_in_frame(Frame::xyz(), [min, max]).expect("two points have a box")
}

fn oriented_box_points(
    center: Point3,
    axes: [Vector3<f64>; 3],
    half_size: Vector3<f64>,
) -> Vec<Point3> {
    let mut points = Vec::with_capacity(8);
    for x_sign in [-1.0, 1.0] {
        for y_sign in [-1.0, 1.0] {
            for z_sign in [-1.0, 1.0] {
                points.push(
                    center
                        + axes[0] * (x_sign * half_size.x)
                        + axes[1] * (y_sign * half_size.y)
                        + axes[2] * (z_sign * half_size.z),
                );
            }
        }
    }
    points
}
