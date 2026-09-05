use nalgebra::Vector3;
use ngk::builders::boolean::{
    BooleanError, BooleanOperand, BooleanOptions, IntersectionEventId, IntersectionNetwork,
    IntersectionOrientation, IntersectionSpanId, IntersectionSpanUse,
    compute_boolean_intersections, prepare_boolean_with_external_tool, validate_solid_network,
};
use ngk::geometry::{Frame, Plane, Point3, PointCoincidence};
use ngk::modeling::{faces, solids};
use ngk::topology::TopologyEditError;
use ngk::topology::gmap::GMap;
use ngk::topology::shape_keys::SolidKey;

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

fn oriented_endpoints(
    network: &IntersectionNetwork,
    entry: (IntersectionSpanId, IntersectionOrientation),
) -> (IntersectionEventId, IntersectionEventId) {
    let span = network.span(entry.0).expect("boundary span exists");
    match entry.1 {
        IntersectionOrientation::Forward => (span.start, span.end),
        IntersectionOrientation::Reversed => (span.end, span.start),
    }
}

#[test]
fn canonical_spans_carry_no_event_in_their_interior() {
    let (map, first, second) = two_blocks(Point3::origin(), 2.0, Point3::new(1.0, 1.0, 1.0), 2.0);
    let plan = compute_boolean_intersections(
        &map,
        BooleanOperand::Solid(first),
        BooleanOperand::Solid(second),
        BooleanOptions::default(),
    )
    .expect("overlapping blocks must produce a network");
    let tolerances = plan.diagnostics.tolerances;

    for (index, span) in plan.network.spans().iter().enumerate() {
        for event in plan.network.events() {
            let t = span.curve.param_at(event.point);
            if !span
                .curve
                .point_at(t)
                .coincides(event.point, tolerances.linear)
            {
                continue;
            }
            assert!(
                t <= tolerances.parameter || t >= 1.0 - tolerances.parameter,
                "event at {:?} splits span {index} at {t}",
                event.point
            );
        }
    }
}

#[test]
fn coplanar_overlap_regions_are_bounded_by_a_closed_counterclockwise_cycle() {
    let (map, first, second) = two_blocks(Point3::origin(), 2.0, Point3::new(1.0, 1.0, 2.0), 2.0);
    let plan = compute_boolean_intersections(
        &map,
        BooleanOperand::Solid(first),
        BooleanOperand::Solid(second),
        BooleanOptions::default(),
    )
    .expect("coplanar blocks must produce a network");

    let regions = plan.network.regions();
    assert_eq!(regions.len(), 1, "one coplanar overlap is expected");
    for region in regions {
        assert!(
            region.boundary.len() >= 3,
            "an overlap region needs a closed boundary"
        );
        assert!(
            !region.normals_agree,
            "the touching faces of two stacked blocks oppose each other"
        );
        let mut area = 0.0;
        let surface = map.face_unchecked(region.first_face).surface().clone();
        for window in 0..region.boundary.len() {
            let current = oriented_endpoints(&plan.network, region.boundary[window]);
            let next = oriented_endpoints(
                &plan.network,
                region.boundary[(window + 1) % region.boundary.len()],
            );
            assert_eq!(
                current.1, next.0,
                "region boundary must chain end to start in order"
            );
            let start = plan.network.event(current.0).expect("event").point;
            let end = plan.network.event(current.1).expect("event").point;
            let start_uv = surface.closest_parameter(start).expect("planar parameter");
            let end_uv = surface.closest_parameter(end).expect("planar parameter");
            area += start_uv.x * end_uv.y - end_uv.x * start_uv.y;
        }
        assert!(
            area > 0.0,
            "region boundary must run counterclockwise in the first face domain, got {area}"
        );
    }
}

#[test]
fn a_solid_network_is_two_sided_and_closed() {
    let (map, first, second) = two_blocks(Point3::origin(), 2.0, Point3::new(1.0, 1.0, 1.0), 2.0);
    let plan = compute_boolean_intersections(
        &map,
        BooleanOperand::Solid(first),
        BooleanOperand::Solid(second),
        BooleanOptions::default(),
    )
    .expect("overlapping blocks must produce a network");

    validate_solid_network(&map, &plan.network, plan.diagnostics.tolerances)
        .expect("a transverse box pair yields a closed two-sided network");
    for span in plan.network.spans() {
        assert_eq!(
            span.uses
                .iter()
                .filter(|span_use| matches!(span_use, IntersectionSpanUse::Face { .. }))
                .count(),
            2,
            "every solid contact section is imprinted on both operands"
        );
    }
}

#[test]
fn an_open_intersection_loop_is_rejected_for_solid_evaluation() {
    let target = faces::rectangle(Plane::xy(), 1.0, 1.0).expect("target face");
    let tool_plane = Plane::from_xy(Point3::new(0.0, 0.5, -0.5), Vector3::x(), Vector3::z());
    let tool = faces::rectangle(tool_plane, 1.0, 1.0).expect("tool face");
    let (mut target_map, target_face) = target.into_map();
    let (tool_map, tool_face) = tool.into_map();

    let prepared = prepare_boolean_with_external_tool(
        &mut target_map,
        BooleanOperand::Face(target_face),
        &tool_map,
        BooleanOperand::Face(tool_face),
        BooleanOptions::default(),
    )
    .expect("perpendicular faces prepare");

    let error = validate_solid_network(
        &target_map,
        &prepared.network,
        prepared.diagnostics.tolerances,
    )
    .expect_err("a section ending on a free boundary cannot bound a solid");
    assert!(
        matches!(error, BooleanError::OpenIntersectionLoop { .. }),
        "expected an open loop, got {error:?}"
    );
}
