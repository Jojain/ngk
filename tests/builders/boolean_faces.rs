//! Regularized Booleans on two coplanar faces.
//!
//! The fixture throughout is the pair A = [0,2]x[0,2] and B = [1,3]x[1,3],
//! overlapping in [1,2]x[1,2]. Areas are the clearest assertion available: the
//! answer's area says which faces survived without depending on which keys the
//! splitter happened to hand out.

use nalgebra::Vector3;
use ngk::builders::boolean::{BooleanError, BooleanOperation, BooleanOptions, face_boolean};
use ngk::geometry::{Plane, Point3};
use ngk::model::{Cell2, Model};
use ngk::modeling::faces;
use ngk::topology::ModelEditError;
use ngk::topology::shape_keys::FaceKey;

/// Builds the overlapping pair in one model and returns both face keys.
fn overlapping_squares() -> (Model, FaceKey, FaceKey) {
    let (mut map, first) = faces::square(Plane::xy(), 2.0).expect("first").into_model();
    let (second_map, second) = faces::square(
        Plane::new(
            Point3::new(1.0, 1.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 1.0),
        ),
        2.0,
    )
    .expect("second")
    .into_model();
    let second = map
        .transaction(|edit| {
            let dart = edit.merge(second_map.face_unchecked(second));
            Ok::<_, ModelEditError>(edit.cell_key_unchecked::<Cell2>(dart))
        })
        .expect("merge second operand");
    (map, first, second)
}

/// Total area of the faces an operation reported.
fn reported_area(map: &Model, faces: &[FaceKey]) -> f64 {
    faces
        .iter()
        .map(|&face| map.face_unchecked(face).area().expect("face area"))
        .sum()
}

#[test]
fn intersection_keeps_only_the_shared_square() {
    let (mut map, first, second) = overlapping_squares();
    let result = face_boolean(
        &mut map,
        first,
        second,
        BooleanOperation::Intersection,
        BooleanOptions::default(),
    )
    .expect("intersection");
    assert_eq!(result.faces.len(), 1, "the overlap is one square");
    assert!(
        (reported_area(&map, &result.faces) - 1.0).abs() < 1e-9,
        "expected the unit overlap, got {}",
        reported_area(&map, &result.faces)
    );
}

#[test]
fn difference_keeps_only_the_first_operands_l_shape() {
    let (mut map, first, second) = overlapping_squares();
    let result = face_boolean(
        &mut map,
        first,
        second,
        BooleanOperation::Difference,
        BooleanOptions::default(),
    )
    .expect("difference");
    assert!(
        (reported_area(&map, &result.faces) - 3.0).abs() < 1e-9,
        "a 2x2 square less a unit overlap is 3, got {}",
        reported_area(&map, &result.faces)
    );
}

/// The second operand is consumed, and nothing of it is left behind.
///
/// Both surviving rules drop every face descended from the second operand, so
/// after either operation the model holds only the answer.
#[test]
fn the_second_operand_is_removed_entirely() {
    for operation in [BooleanOperation::Intersection, BooleanOperation::Difference] {
        let (mut map, first, second) = overlapping_squares();
        let result = face_boolean(
            &mut map,
            first,
            second,
            operation,
            BooleanOptions::default(),
        )
        .expect("operation");
        assert!(
            map.face(second).is_none(),
            "{operation:?} left the second operand registered"
        );
        assert_eq!(
            map.iter_faces().count(),
            result.faces.len(),
            "{operation:?} left faces behind that it did not report"
        );
    }
}

/// Every reported face descends from the first operand, and says so.
#[test]
fn survivors_are_reported_against_their_source_face() {
    let (mut map, first, second) = overlapping_squares();
    let result = face_boolean(
        &mut map,
        first,
        second,
        BooleanOperation::Difference,
        BooleanOptions::default(),
    )
    .expect("difference");
    let sources = result.lineage.faces.keys().copied().collect::<Vec<_>>();
    assert_eq!(sources, vec![first], "only the first operand is a source");
    let reported = result.lineage.faces[&first].clone();
    assert_eq!(reported, result.faces, "lineage and answer must agree");
}

/// The first operand keeps its identity: trimming it does not rename it.
///
/// `split_face_by_imprints` retains the source face's key on one of the two
/// pieces it makes, so an operation that only trims the first operand can hand
/// back the same `FaceKey` the caller passed in. Whichever piece inherits that
/// key must be one the operation kept.
#[test]
fn the_first_operand_keeps_its_key() {
    for operation in [BooleanOperation::Intersection, BooleanOperation::Difference] {
        let (mut map, first, second) = overlapping_squares();
        let result = face_boolean(
            &mut map,
            first,
            second,
            operation,
            BooleanOptions::default(),
        )
        .expect("operation");
        assert!(
            result.faces.contains(&first),
            "{operation:?} dropped the first operand's own key; kept {:?}",
            result.faces
        );
    }
}

/// Two faces on different planes have no comparable parameter space.
#[test]
fn faces_on_different_planes_are_refused() {
    let (mut map, first) = faces::square(Plane::xy(), 2.0).expect("first").into_model();
    let (second_map, second) = faces::square(Plane::xz(), 2.0)
        .expect("second")
        .into_model();
    let second = map
        .transaction(|edit| {
            let dart = edit.merge(second_map.face_unchecked(second));
            Ok::<_, ModelEditError>(edit.cell_key_unchecked::<Cell2>(dart))
        })
        .expect("merge second operand");
    let error = face_boolean(
        &mut map,
        first,
        second,
        BooleanOperation::Intersection,
        BooleanOptions::default(),
    )
    .expect_err("non-coplanar operands have no answer");
    assert!(
        matches!(error, BooleanError::OperandSupportsDiffer { .. }),
        "expected a named refusal, got {error:?}"
    );
}

/// Union covers both operands, as one face.
///
/// Two 2x2 squares overlapping in a unit square cover 4 + 4 - 1 = 7. The answer
/// is one face, not three: welding joins the operands across the section, and
/// healing then removes the shape-free edges between coplanar neighbours.
#[test]
fn union_covers_both_operands_as_one_face() {
    let (mut map, first, second) = overlapping_squares();
    let result = face_boolean(
        &mut map,
        first,
        second,
        BooleanOperation::Union,
        BooleanOptions::default(),
    )
    .expect("union");
    assert!(
        (reported_area(&map, &result.faces) - 7.0).abs() < 1e-9,
        "expected 7, got {}",
        reported_area(&map, &result.faces)
    );
    assert_eq!(
        result.faces.len(),
        1,
        "the union of two overlapping squares is one face"
    );
}

/// A union's answer descends from both operands, and the lineage names both.
#[test]
fn union_reports_both_operands_as_sources() {
    let (mut map, first, second) = overlapping_squares();
    let result = face_boolean(
        &mut map,
        first,
        second,
        BooleanOperation::Union,
        BooleanOptions::default(),
    )
    .expect("union");
    let mut sources = result.lineage.faces.keys().copied().collect::<Vec<_>>();
    sources.sort();
    let mut expected = vec![first, second];
    expected.sort();
    assert_eq!(sources, expected, "a union comes from both operands");
}

/// Builds a square of `size` whose lower corner sits at `(x, y)` on the xy plane.
fn square_at(x: f64, y: f64, size: f64) -> (Model, FaceKey) {
    faces::square(
        Plane::new(
            Point3::new(x, y, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 1.0),
        ),
        size,
    )
    .expect("square")
    .into_model()
}

/// Moves `face` from its own model into `map` and returns its key there.
fn merge_face(map: &mut Model, source: &Model, face: FaceKey) -> FaceKey {
    map.transaction(|edit| {
        let dart = edit.merge(source.face_unchecked(face));
        Ok::<_, ModelEditError>(edit.cell_key_unchecked::<Cell2>(dart))
    })
    .expect("merge face")
}

/// A face wholly inside another, sharing no boundary.
fn nested_squares() -> (Model, FaceKey, FaceKey) {
    let (mut map, outer) = square_at(0.0, 0.0, 4.0);
    let (inner_map, inner) = square_at(1.0, 1.0, 1.0);
    let inner = merge_face(&mut map, &inner_map, inner);
    (map, outer, inner)
}

/// Subtracting a contained face leaves a hole rather than two faces.
///
/// The inner boundary is a closed imprint, so the split gives the outer face a
/// loop it does not bound from outside. Area is what shows the loop is a hole
/// and not a second piece: 16 less 1 is 15, and it arrives as one face.
#[test]
fn difference_with_a_contained_face_leaves_a_hole() {
    let (mut map, outer, inner) = nested_squares();
    let result = face_boolean(
        &mut map,
        outer,
        inner,
        BooleanOperation::Difference,
        BooleanOptions::default(),
    )
    .expect("difference");
    assert_eq!(result.faces.len(), 1, "a square with a hole is one face");
    assert!(
        (reported_area(&map, &result.faces) - 15.0).abs() < 1e-9,
        "expected 4x4 less 1x1, got {}",
        reported_area(&map, &result.faces)
    );
}

/// Intersecting with a contained face returns that face's area.
#[test]
fn intersection_with_a_contained_face_returns_the_inner_square() {
    let (mut map, outer, inner) = nested_squares();
    let result = face_boolean(
        &mut map,
        outer,
        inner,
        BooleanOperation::Intersection,
        BooleanOptions::default(),
    )
    .expect("intersection");
    assert!(
        (reported_area(&map, &result.faces) - 1.0).abs() < 1e-9,
        "expected the contained unit square, got {}",
        reported_area(&map, &result.faces)
    );
}

/// Uniting with a contained face changes nothing about the outer one.
///
/// Every face of the second operand lies inside the first, so the union rule
/// keeps none of them, and healing fuses the two pieces the imprint made back
/// into the face they came from.
#[test]
fn union_with_a_contained_face_returns_the_outer_square() {
    let (mut map, outer, inner) = nested_squares();
    let result = face_boolean(
        &mut map,
        outer,
        inner,
        BooleanOperation::Union,
        BooleanOptions::default(),
    )
    .expect("union");
    assert_eq!(result.faces.len(), 1, "the outer square is one face");
    assert!(
        (reported_area(&map, &result.faces) - 16.0).abs() < 1e-9,
        "expected the outer square unchanged, got {}",
        reported_area(&map, &result.faces)
    );
}

/// Two squares meeting along one edge and overlapping nowhere.
fn edge_to_edge_squares() -> (Model, FaceKey, FaceKey) {
    let (mut map, first) = square_at(0.0, 0.0, 2.0);
    let (second_map, second) = square_at(2.0, 0.0, 2.0);
    let second = merge_face(&mut map, &second_map, second);
    (map, first, second)
}

/// Faces that only touch have no area in common, and an intersection says so.
///
/// What the two share is a segment, which is not a face: the answer would be a
/// cell of lower dimension than the operands, and a regularized Boolean has no
/// such answer. Every face of the first operand lies outside the second, so the
/// table keeps nothing and the operation reports an empty result rather than
/// returning the seam.
#[test]
fn intersecting_faces_that_only_touch_is_empty() {
    let (mut map, first, second) = edge_to_edge_squares();
    let error = face_boolean(
        &mut map,
        first,
        second,
        BooleanOperation::Intersection,
        BooleanOptions::default(),
    )
    .expect_err("a shared edge is not an area");
    assert!(
        matches!(error, BooleanError::EmptyResult),
        "expected an empty result, got {error:?}"
    );
}

/// A failed operation leaves the model exactly as it was.
///
/// Preparation splits both operands before the table can find the answer empty,
/// so the rollback is what keeps an empty intersection from consuming them.
#[test]
fn an_empty_intersection_rolls_back_both_operands() {
    let (mut map, first, second) = edge_to_edge_squares();
    let before = map.iter_faces().count();
    let _ = face_boolean(
        &mut map,
        first,
        second,
        BooleanOperation::Intersection,
        BooleanOptions::default(),
    );
    assert_eq!(
        map.iter_faces().count(),
        before,
        "rollback left faces behind"
    );
    assert!(map.face(first).is_some(), "the first operand was consumed");
    assert!(
        map.face(second).is_some(),
        "the second operand was consumed"
    );
}

/// Subtracting a face that only touches changes nothing.
#[test]
fn difference_with_a_touching_face_is_unchanged() {
    let (mut map, first, second) = edge_to_edge_squares();
    let result = face_boolean(
        &mut map,
        first,
        second,
        BooleanOperation::Difference,
        BooleanOptions::default(),
    )
    .expect("difference");
    assert_eq!(result.faces, vec![first], "the first operand is untouched");
    assert!(
        (reported_area(&map, &result.faces) - 4.0).abs() < 1e-9,
        "expected the first operand's own area, got {}",
        reported_area(&map, &result.faces)
    );
}

/// Uniting two faces that meet along an edge gives one face of both areas.
#[test]
fn union_of_touching_faces_covers_both() {
    let (mut map, first, second) = edge_to_edge_squares();
    let result = face_boolean(
        &mut map,
        first,
        second,
        BooleanOperation::Union,
        BooleanOptions::default(),
    )
    .expect("union");
    assert!(
        (reported_area(&map, &result.faces) - 8.0).abs() < 1e-9,
        "expected both areas, got {}",
        reported_area(&map, &result.faces)
    );
    assert_eq!(result.faces.len(), 1, "the two squares fuse into one face");
}
