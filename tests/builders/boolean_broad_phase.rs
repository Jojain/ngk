use nalgebra::Vector3;
use ngk::builders::boolean::{BooleanOperand, BooleanOptions, compute_boolean_intersections};
use ngk::geometry::{Frame, Point3};
use ngk::modeling::solids;
use ngk::topology::TopologyEditError;
use ngk::topology::gmap::GMap;
use ngk::topology::shape_keys::{FaceKey, SolidKey};

fn two_blocks(
    first_origin: Point3,
    first_size: f64,
    second_origin: Point3,
    second_size: f64,
) -> (GMap<ngk::StandardPayload>, SolidKey, SolidKey) {
    let (mut map, first) = solids::block_at(
        Frame::from_xy(first_origin, Vector3::x(), Vector3::y()),
        first_size,
        first_size,
        first_size,
    )
    .expect("first block")
    .into_map();
    let (tool, second) = solids::block_at(
        Frame::from_xy(second_origin, Vector3::x(), Vector3::y()),
        second_size,
        second_size,
        second_size,
    )
    .expect("second block")
    .into_map();
    let second = map
        .transaction(|edit| {
            let dart = edit.merge(tool.solid_unchecked(second));
            Ok::<_, TopologyEditError>(edit.solid_key(dart).unwrap())
        })
        .unwrap();
    (map, first, second)
}

/// Axis-aligned extent of a face, from its own vertices.
fn face_bounds(map: &GMap<ngk::StandardPayload>, face: FaceKey) -> (Point3, Point3) {
    let mut min = Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut max = Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for vertex in map.face_unchecked(face).vertices() {
        let point = *vertex.point().expect("face geometry");
        min = Point3::new(min.x.min(point.x), min.y.min(point.y), min.z.min(point.z));
        max = Point3::new(max.x.max(point.x), max.y.max(point.y), max.z.max(point.z));
    }
    (min, max)
}

/// Brute-force count of face pairs whose extents overlap, which no broad phase
/// may prune.
fn overlapping_pair_count(
    map: &GMap<ngk::StandardPayload>,
    first: SolidKey,
    second: SolidKey,
) -> usize {
    let mut count = 0;
    for left in map.solid_unchecked(first).faces() {
        let (left_min, left_max) = face_bounds(map, left.key());
        for right in map.solid_unchecked(second).faces() {
            let (right_min, right_max) = face_bounds(map, right.key());
            let overlaps = (0..3)
                .all(|axis| left_min[axis] <= right_max[axis] && right_min[axis] <= left_max[axis]);
            count += usize::from(overlaps);
        }
    }
    count
}

#[test]
fn the_broad_phase_keeps_every_pair_whose_bounds_overlap() {
    for offset in [0.0, 0.5, 1.0, 1.5, 2.0, 3.0] {
        let (map, first, second) = two_blocks(
            Point3::origin(),
            2.0,
            Point3::new(offset, offset * 0.5, 0.25),
            2.0,
        );
        let plan = compute_boolean_intersections(
            &map,
            BooleanOperand::Solid(first),
            BooleanOperand::Solid(second),
            BooleanOptions::default(),
        )
        .expect("blocks must produce a plan");
        let expected = overlapping_pair_count(&map, first, second);
        assert!(
            plan.diagnostics.candidate_pairs_tested >= expected,
            "offset {offset}: pruned {} of {expected} overlapping pairs",
            expected.saturating_sub(plan.diagnostics.candidate_pairs_tested)
        );
        assert_eq!(
            plan.diagnostics.candidate_pairs_tested + plan.diagnostics.candidate_pairs_pruned,
            36,
            "every face pair of two blocks is either tested or pruned"
        );
    }
}

#[test]
fn candidate_enumeration_is_deterministic() {
    let (map, first, second) = two_blocks(Point3::origin(), 2.0, Point3::new(1.0, 1.0, 1.0), 2.0);
    let plans = (0..2)
        .map(|_| {
            compute_boolean_intersections(
                &map,
                BooleanOperand::Solid(first),
                BooleanOperand::Solid(second),
                BooleanOptions::default(),
            )
            .expect("blocks must produce a plan")
        })
        .collect::<Vec<_>>();

    assert_eq!(
        plans[0].diagnostics.candidate_pairs_tested,
        plans[1].diagnostics.candidate_pairs_tested
    );
    assert_eq!(
        plans[0].network.spans().len(),
        plans[1].network.spans().len()
    );
    for (left, right) in plans[0]
        .network
        .events()
        .iter()
        .zip(plans[1].network.events())
    {
        assert_eq!(
            left.point, right.point,
            "two runs on one map must canonicalize events in the same order"
        );
    }
    for (left, right) in plans[0]
        .network
        .spans()
        .iter()
        .zip(plans[1].network.spans())
    {
        assert_eq!(left.start, right.start);
        assert_eq!(left.end, right.end);
    }
}
