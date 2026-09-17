use nalgebra::Vector3;
use ngk::geometry::axis::Axis3;
use ngk::geometry::{Frame, LINEAR_TOLERANCE, Point3, PointCoincidence, Rigid};
use ngk::model::Model;
use ngk::modeling::solids::block;
use ngk::modeling::transform::{moved, placed, rotated, translated};
use ngk::topology::payload::StandardPayload;
use radians::Rad64;

/// The block's eight corners, sorted so two models compare corner for corner.
fn corners(model: &Model<StandardPayload>) -> Vec<Point3> {
    let mut points = model
        .iter_vertices()
        .map(|(_, attr)| attr.point)
        .collect::<Vec<_>>();
    points.sort_by(|a, b| {
        (a.x, a.y, a.z)
            .partial_cmp(&(b.x, b.y, b.z))
            .expect("block corner coordinates are finite")
    });
    points
}

fn assert_same_points(actual: &Model<StandardPayload>, expected: &Model<StandardPayload>) {
    let (actual, expected) = (corners(actual), corners(expected));
    assert_eq!(actual.len(), expected.len());
    for (actual, expected) in actual.into_iter().zip(expected) {
        assert!(
            actual.coincides(expected, LINEAR_TOLERANCE),
            "expected {expected:?}, got {actual:?}"
        );
    }
}

#[test]
fn translated_is_moved_by_a_translation() {
    let offset = Vector3::new(1.0, -2.0, 3.0);
    let by_verb = translated(block(1.0, 2.0, 3.0).expect("block should build"), offset);
    let by_motion = moved(
        block(1.0, 2.0, 3.0).expect("block should build"),
        Rigid::translation(offset),
    );

    assert_same_points(by_verb.model(), by_motion.model());
}

#[test]
fn rotated_is_moved_by_a_rotation() {
    let axis = Axis3::new(Point3::new(1.0, 0.0, 0.0), Vector3::z());
    let angle = Rad64::new(0.63);
    let by_verb = rotated(
        block(1.0, 2.0, 3.0).expect("block should build"),
        axis,
        angle,
    );
    let by_motion = moved(
        block(1.0, 2.0, 3.0).expect("block should build"),
        Rigid::rotation(axis, angle),
    );

    assert_same_points(by_verb.model(), by_motion.model());
}

#[test]
fn placed_carries_a_shape_from_one_frame_onto_another() {
    let to = Frame::from_xy(
        Point3::new(5.0, -1.0, 2.0),
        Vector3::new(1.0, 1.0, 0.0),
        Vector3::new(-1.0, 1.0, 1.0),
    );
    let shape = placed(
        block(1.0, 2.0, 3.0).expect("block should build"),
        &Frame::xyz(),
        &to,
    );

    // The block sits on the world origin, so the frame origin is one corner and
    // has to land on the target frame's origin.
    assert!(
        corners(shape.model())
            .iter()
            .any(|corner| corner.coincides(to.origin, LINEAR_TOLERANCE)),
        "the corner at the source frame's origin should land on the target's"
    );
}

/// The point of the inherent verbs: no `?` anywhere in the chain.
#[test]
fn the_shape_verbs_chain_without_unwrapping() {
    let bracket = block(40.0, 20.0, 6.0)
        .expect("block should build")
        .rotated(Axis3::z(), Rad64::QUARTER_TURN)
        .translated(Vector3::new(0.0, 0.0, 12.0));

    assert!(
        bracket
            .model()
            .iter_vertices()
            .all(|(_, attr)| attr.point.z >= 12.0 - LINEAR_TOLERANCE),
        "every corner should have been lifted to z = 12 or above"
    );
}

#[test]
fn a_moved_shape_keeps_its_primary_handle() {
    let shape = block(1.0, 2.0, 3.0).expect("block should build");
    let key = shape.key();

    let shape = shape.translated(Vector3::new(1.0, 1.0, 1.0));

    assert_eq!(shape.key(), key);
}
