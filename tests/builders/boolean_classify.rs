use nalgebra::Vector3;
use ngk::builders::boolean::{BooleanError, BooleanOptions, solid_contains_point};
use ngk::builders::solids::add_extruded_face;
use ngk::geometry::{Plane, Point3};
use ngk::modeling::faces;
use ngk::topology::gmap::GMap;
use ngk::topology::shape_keys::SolidKey;

fn block_at(origin: Point3, size: f64) -> (GMap<ngk::StandardPayload>, SolidKey) {
    let plane = Plane::from_xy(origin, Vector3::x(), Vector3::y());
    let base = faces::rectangle(plane, size, size).expect("block base");
    let (mut map, face) = base.into_map();
    let solid =
        add_extruded_face(&mut map, face, Vector3::new(0.0, 0.0, size)).expect("block extrusion");
    (map, solid)
}

#[test]
fn ray_parity_classifies_interior_and_exterior_points_of_a_block() {
    let (map, block) = block_at(Point3::origin(), 2.0);
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
    let (map, block) = block_at(Point3::origin(), 2.0);
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
