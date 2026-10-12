//! Booleans of one target with several tools at once.
//!
//! The answer is a set formula over every operand -- `T − ∪tᵢ`, `T ∪ ∪tᵢ`,
//! `T ∩ ∩tᵢ` -- so it must not depend on the order the tools come in, and only
//! the final solid has to be one connected piece. Volumes are the main
//! assertion: they say which pieces survived without depending on which keys
//! the splitter handed out.

use nalgebra::Vector3;
use ngk::builders::boolean::{
    BooleanError, BooleanInput, BooleanOperation, BooleanOptions, boolean, face_boolean,
};
use ngk::geometry::{Frame, Plane, Point3, Surface};
use ngk::model::{Cell2, Model};
use ngk::modeling::{faces, solids};
use ngk::topology::ModelEditError;
use ngk::topology::shape::{FaceTag, Shape, SolidTag};
use ngk::topology::shape_keys::{FaceKey, SolidKey};

fn block(x: f64, y: f64, z: f64, size: [f64; 3]) -> Shape<SolidTag> {
    solids::block_at(Frame::at(Point3::new(x, y, z)), size[0], size[1], size[2]).expect("block")
}

fn volume(shape: &Shape<SolidTag>) -> f64 {
    shape.solid().volume().expect("volume")
}

fn counts(shape: &Shape<SolidTag>) -> [usize; 3] {
    let solid = shape.solid();
    [
        solid.faces().len(),
        solid.edges().len(),
        solid.vertices().len(),
    ]
}

fn assert_close(actual: f64, expected: f64, what: &str) {
    assert!(
        (actual - expected).abs() <= 1e-6 * expected.abs().max(1.0),
        "{what}: {actual}, expected {expected}"
    );
}

/// Two blocks apart and a bar overlapping both: the fuse of all three is one
/// solid, although the fuse of the first two alone is not.
fn apart_and_bridged() -> [Shape<SolidTag>; 3] {
    [
        block(0.0, 0.0, 0.0, [10.0, 10.0, 10.0]),
        block(30.0, 0.0, 0.0, [10.0, 10.0, 10.0]),
        block(5.0, 3.0, 3.0, [30.0, 4.0, 4.0]),
    ]
}

#[test]
fn tools_bridged_only_by_a_later_tool_fuse_the_same_in_every_order() {
    let orders = [
        [0, 1, 2],
        [0, 2, 1],
        [1, 0, 2],
        [1, 2, 0],
        [2, 0, 1],
        [2, 1, 0],
    ];
    let mut topology = None;
    for order in orders {
        let mut operands = apart_and_bridged().map(Some);
        let [target, first, second] = order.map(|index| operands[index].take().unwrap());
        let fused = solids::fuse(target, [&first, &second])
            .unwrap_or_else(|error| panic!("{order:?} should fuse: {error}"));
        assert_close(volume(&fused), 2320.0, &format!("{order:?} volume"));
        let counts = counts(&fused);
        assert_eq!(
            *topology.get_or_insert(counts),
            counts,
            "{order:?} faces, edges and vertices"
        );
    }
}

#[test]
fn a_disconnected_final_result_is_still_refused() {
    let [a, b, _] = apart_and_bridged();
    let error = solids::fuse(a, [&b]).err().expect("two blocks apart");
    assert!(
        matches!(error, BooleanError::DisconnectedResult { components: 2 }),
        "{error:?}"
    );
}

/// Two tool bars through the target along x and along y, crossing each other
/// inside it, so the tools overlap and each one's walls are cut by the other.
fn crossing_bars() -> [Shape<SolidTag>; 3] {
    [
        block(0.0, 0.0, 0.0, [10.0, 10.0, 10.0]),
        block(-1.0, 2.0, 2.0, [12.0, 4.0, 4.0]),
        block(4.0, -1.0, 3.0, [4.0, 12.0, 4.0]),
    ]
}

#[test]
fn overlapping_tools_cut_their_union_once() {
    let [target, first, second] = crossing_bars();
    let cut = solids::cut(target, [&first, &second]).expect("cut");
    // Each bar removes 160 from the target; they share x in [4, 8], y in
    // [2, 6], z in [3, 6].
    assert_close(volume(&cut), 1000.0 - (160.0 + 160.0 - 48.0), "volume");
}

#[test]
fn overlapping_tools_fuse_and_intersect_through_each_other() {
    let [target, first, second] = crossing_bars();
    let fused = solids::fuse(target, [&first, &second]).expect("fuse");
    // Each bar sticks out by 1 at both ends.
    assert_close(volume(&fused), 1000.0 + 2.0 * 16.0 * 2.0, "fused volume");

    let [target, first, second] = crossing_bars();
    let common = solids::intersect(target, [&first, &second]).expect("intersect");
    assert_close(volume(&common), 48.0, "common volume");
}

/// The target and two tools whose tops all lie on z = 10, overlapping each
/// other pairwise and all three over x in [9, 10], y in [3, 5].
fn flush_tops() -> [Shape<SolidTag>; 3] {
    [
        block(0.0, 0.0, 0.0, [10.0, 10.0, 10.0]),
        block(8.0, 2.0, 4.0, [6.0, 4.0, 6.0]),
        block(9.0, 3.0, 2.0, [7.0, 2.0, 8.0]),
    ]
}

#[test]
fn a_face_coplanar_across_the_target_and_two_tools_is_kept_once() {
    let [target, first, second] = flush_tops();
    let fused = solids::fuse(target, [&first, &second]).expect("fuse");
    // 1000 + 144 + 112, less the pairwise overlaps 48, 16 and 60, plus the
    // triple overlap 12 they removed once too often.
    assert_close(volume(&fused), 1144.0, "fused volume");
    let top = fused
        .solid()
        .faces()
        .into_iter()
        .filter(|face| {
            face.vertices()
                .iter()
                .all(|vertex| (vertex.point().z - 10.0).abs() < 1e-9)
        })
        .count();
    assert_eq!(top, 1, "the shared top is healed into one face");

    let [target, first, second] = flush_tops();
    let cut = solids::cut(target, [&first, &second]).expect("cut");
    assert_close(volume(&cut), 1000.0 - (48.0 + 16.0 - 12.0), "cut volume");

    let [target, first, second] = flush_tops();
    let common = solids::intersect(target, [&first, &second]).expect("intersect");
    assert_close(volume(&common), 12.0, "common volume");
}

#[test]
fn no_tools_return_the_target_unchanged() {
    for operation in [
        BooleanOperation::Union,
        BooleanOperation::Difference,
        BooleanOperation::Intersection,
    ] {
        let target = block(0.0, 0.0, 0.0, [1.0, 2.0, 3.0]);
        let key = target.key();
        let faces = target
            .solid()
            .faces()
            .iter()
            .map(|face| face.key())
            .collect::<Vec<_>>();
        let (mut map, _) = target.into_model();
        let revision = map.revision();
        let result =
            boolean(&mut map, key, &[], operation, BooleanOptions::default()).expect("no tools");
        assert_eq!(result.solid, key, "{operation:?}");
        assert!(result.lineage.tools.is_empty());
        assert_eq!(map.revision(), revision, "{operation:?} touched the model");
        let after = map
            .solid_unchecked(key)
            .faces()
            .iter()
            .map(|face| face.key())
            .collect::<Vec<_>>();
        assert_eq!(after, faces, "{operation:?}");
    }
}

#[test]
fn no_tools_return_the_target_unchanged_through_the_modeling_api() {
    let target = block(0.0, 0.0, 0.0, [1.0, 2.0, 3.0]);
    let key = target.key();
    let cut = solids::cut(target, []).expect("no tools");
    assert_eq!(cut.key(), key);
    assert_close(volume(&cut), 6.0, "volume");
}

/// Puts every shape's solid into one model, returning the keys in order.
fn one_model(shapes: Vec<Shape<SolidTag>>) -> (Model, Vec<SolidKey>) {
    let mut shapes = shapes.into_iter();
    let (mut map, first) = shapes.next().expect("one shape").into_model();
    let mut keys = vec![first];
    for shape in shapes {
        let (other, key) = shape.into_model();
        keys.push(
            map.transaction(|edit| {
                let dart = edit.merge(other.solid_unchecked(key));
                Ok::<_, ModelEditError>(edit.solid_key(dart).unwrap())
            })
            .expect("merge"),
        );
    }
    (map, keys)
}

#[test]
fn a_tool_passed_twice_is_one_operand() {
    let (mut map, keys) = one_model(crossing_bars().into());
    let result = boolean(
        &mut map,
        keys[0],
        &[keys[1], keys[1]],
        BooleanOperation::Difference,
        BooleanOptions::default(),
    )
    .expect("cut");
    assert_close(
        map.solid_unchecked(result.solid).volume().unwrap(),
        1000.0 - 160.0,
        "volume",
    );
    assert_eq!(result.lineage.tools.len(), 2, "one lineage per input");
}

#[test]
fn a_shape_passed_twice_as_a_tool_is_copied_in_once() {
    let [target, tool, _] = crossing_bars();
    let cut = solids::cut(target, [&tool, &tool]).expect("cut");
    assert_close(volume(&cut), 1000.0 - 160.0, "volume");
    assert_eq!(cut.model().iter_solids().count(), 1, "one copy, consumed");
}

#[test]
fn a_tool_equal_to_the_target_collapses_by_the_set_rules() {
    let (mut map, keys) = one_model(vec![block(0.0, 0.0, 0.0, [1.0, 1.0, 1.0])]);
    for operation in [BooleanOperation::Union, BooleanOperation::Intersection] {
        let result = boolean(
            &mut map,
            keys[0],
            &[keys[0]],
            operation,
            BooleanOptions::default(),
        )
        .expect("T with itself is T");
        assert_eq!(result.solid, keys[0]);
    }
    let error = boolean(
        &mut map,
        keys[0],
        &[keys[0]],
        BooleanOperation::Difference,
        BooleanOptions::default(),
    )
    .expect_err("T minus itself is empty");
    assert!(matches!(error, BooleanError::EmptyResult), "{error:?}");
}

#[test]
fn errors_name_the_tool_they_are_about() {
    let (mut map, keys) = one_model(crossing_bars().into());
    let error = boolean(
        &mut map,
        keys[0],
        &[keys[1], SolidKey::default()],
        BooleanOperation::Difference,
        BooleanOptions::default(),
    )
    .expect_err("an unregistered tool");
    assert!(
        matches!(
            error,
            BooleanError::MissingInput {
                input: BooleanInput::Tool(1),
                ..
            }
        ),
        "{error:?}"
    );
    assert!(error.to_string().contains("tools[1]"), "{error}");
}

#[test]
fn cells_the_boolean_does_not_touch_keep_their_keys() {
    // A second solid shares the target's model and is far from the tool.
    let (map, keys) = one_model(vec![
        block(0.0, 0.0, 0.0, [10.0, 10.0, 10.0]),
        block(50.0, 0.0, 0.0, [1.0, 1.0, 1.0]),
    ]);
    let bystander = keys[1];
    let bystander_faces = map
        .solid_unchecked(bystander)
        .faces()
        .iter()
        .map(|face| face.key())
        .collect::<Vec<_>>();
    let target = Shape::<SolidTag>::new(map, keys[0]);
    let tool = block(2.0, 2.0, -1.0, [2.0, 2.0, 12.0]);
    let cut = solids::cut(target, [&tool]).expect("cut");
    let after = cut
        .model()
        .solid(bystander)
        .expect("the bystander is still in the model")
        .faces()
        .iter()
        .map(|face| face.key())
        .collect::<Vec<_>>();
    assert_eq!(after, bystander_faces);
}

#[test]
fn a_target_face_an_earlier_tool_left_keeps_its_key_through_a_later_cut() {
    let plate = block(0.0, 0.0, 0.0, [40.0, 20.0, 10.0]);
    let hole = |x: f64| {
        solids::cylinder_at(
            Frame::from_xz(Point3::new(x, 10.0, -1.0), Vector3::x(), Vector3::z()),
            3.0,
            12.0,
        )
        .expect("cylinder")
    };
    // Told apart by the axis, not by corners: a wall bounded by two whole
    // circles has none, and "every corner is left of x = 20" holds of it
    // vacuously.
    let wall = |shape: &Shape<SolidTag>| {
        shape
            .solid()
            .faces()
            .into_iter()
            .find(|face| match face.surface() {
                Surface::Cylinder(cylinder) => cylinder.origin().x < 20.0,
                _ => false,
            })
            .map(|face| face.key())
            .expect("the first hole's wall")
    };
    let one_hole = solids::cut(plate, [&hole(10.0)]).expect("first hole");
    let first_wall = wall(&one_hole);
    let two_holes = solids::cut(one_hole, [&hole(30.0)]).expect("second hole");
    assert_eq!(wall(&two_holes), first_wall);
}

/// A square of `size` with its lower corner at `(x, y)` on the xy plane.
fn square(x: f64, y: f64, size: [f64; 2]) -> Shape<FaceTag> {
    faces::rectangle(
        Plane::new(Point3::new(x, y, 0.0), Vector3::x(), Vector3::z()),
        size[0],
        size[1],
    )
    .expect("rectangle")
}

fn area(result: &faces::FaceBooleanResult) -> f64 {
    result
        .faces()
        .iter()
        .map(|face| face.area().expect("area"))
        .sum()
}

#[test]
fn faces_combine_with_several_overlapping_tools_at_once() {
    // A = [0,4]², B = [3,6]x[1,3] overlapping A and C, C = [5,8]x[0,2] apart
    // from A.
    let shapes = || {
        (
            square(0.0, 0.0, [4.0, 4.0]),
            square(3.0, 1.0, [3.0, 2.0]),
            square(5.0, 0.0, [3.0, 2.0]),
        )
    };
    let (a, b, c) = shapes();
    let union = faces::fuse(a, [&b, &c]).expect("fuse");
    assert_close(area(&union), 16.0 + 6.0 + 6.0 - 2.0 - 1.0, "union area");
    assert_eq!(union.face_keys().len(), 1, "the union is one face");

    let (a, b, c) = shapes();
    let cut = faces::cut(a, [&b, &c]).expect("cut");
    assert_close(area(&cut), 16.0 - 2.0, "cut area");

    let (a, b, _) = shapes();
    let d = square(2.0, 0.0, [3.0, 2.0]);
    let common = faces::intersect(a, [&b, &d]).expect("intersect");
    assert_close(area(&common), 1.0, "common area");
}

#[test]
fn a_face_boolean_keeps_the_other_faces_of_the_target_model() {
    // The fuse of two disjoint faces leaves both in one model.
    let square_face = faces::rectangle(Plane::xy(), 40.0, 30.0).expect("square");
    let triangle = faces::polygon(&[
        Point3::new(-30.0, -20.0, 0.0),
        Point3::new(-20.0, -20.0, 0.0),
        Point3::new(-25.0, -12.0, 0.0),
    ])
    .expect("triangle");
    let both = faces::fuse(square_face, [&triangle]).expect("disjoint fuse");
    assert_eq!(both.face_keys().len(), 2);
    let (model, keys) = both.into_model();
    let square_key = keys
        .iter()
        .copied()
        .find(|&key| model.face_unchecked(key).vertices().len() == 4)
        .expect("the square");
    let triangle_key: FaceKey = keys.iter().copied().find(|&key| key != square_key).unwrap();

    let circle = faces::circle(
        Plane::new(Point3::new(40.0, 15.0, 0.0), Vector3::x(), Vector3::z()),
        5.0,
    )
    .expect("circle");
    let grown = faces::fuse(Shape::new(model, square_key), [&circle]).expect("fuse");
    assert_eq!(grown.model().iter_faces().count(), 2);
    assert!(
        grown.model().face(triangle_key).is_some(),
        "the triangle keeps its key"
    );
}

#[test]
fn a_face_tool_off_the_target_plane_is_named() {
    let (mut map, target) = square(0.0, 0.0, [1.0, 1.0]).into_model();
    let lifted = faces::rectangle(
        Plane::new(Point3::new(0.0, 0.0, 1.0), Vector3::x(), Vector3::z()),
        1.0,
        1.0,
    )
    .expect("lifted");
    let flat = square(0.5, 0.5, [1.0, 1.0]);
    let tools = map
        .transaction(|edit| {
            let keys = [flat.face(), lifted.face()].map(|face| {
                let dart = edit.merge(face);
                edit.cell_key_unchecked::<Cell2>(dart)
            });
            Ok::<_, ModelEditError>(keys)
        })
        .expect("merge tools");
    let error = face_boolean(
        &mut map,
        target,
        &tools,
        BooleanOperation::Union,
        BooleanOptions::default(),
    )
    .expect_err("a tool off the plane");
    assert!(
        matches!(
            error,
            BooleanError::OperandSupportsDiffer {
                input: BooleanInput::Tool(1),
                ..
            }
        ),
        "{error:?}"
    );
}
