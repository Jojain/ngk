use ngk::builders::boolean::{BooleanError, BooleanOptions, solid_contains_point};
use ngk::geometry::{Frame, Point3};
use ngk::modeling::solids;

#[test]
fn ray_parity_classifies_interior_and_exterior_points_of_a_block() {
    let (map, block) = solids::block_at(Frame::xyz(), 2.0, 2.0, 2.0)
        .expect("block")
        .into_map();
    for (point, expected) in [
        // The centre sends rays straight at the box corners and edges; those
        // rays must be rejected and retried, not counted.
        (Point3::new(1.0, 1.0, 1.0), true),
        (Point3::new(0.25, 0.25, 0.25), true),
        (Point3::new(1.9, 1.9, 0.1), true),
        (Point3::new(3.0, 1.0, 1.0), false),
        (Point3::new(1.0, 1.0, -0.5), false),
        (Point3::new(-1.0, -1.0, -1.0), false),
    ] {
        let inside = solid_contains_point(&map, block, point, BooleanOptions::default()).unwrap();
        assert_eq!(inside, expected, "membership at {point:?}");
    }
}

#[test]
fn a_point_on_the_boundary_is_reported_as_ambiguous_rather_than_guessed() {
    let (map, block) = solids::block_at(Frame::xyz(), 2.0, 2.0, 2.0)
        .expect("block")
        .into_map();
    let error = solid_contains_point(
        &map,
        block,
        Point3::new(1.0, 1.0, 0.0),
        BooleanOptions::default(),
    )
    .expect_err("a point on a face has no certified parity");
    assert!(
        matches!(error, BooleanError::AmbiguousClassification { .. }),
        "expected ambiguity, got {error:?}"
    );
}
