use nalgebra::Vector3;
use ngk::builders::boolean::{
    BooleanCell, BooleanError, BooleanOperand, BooleanOptions, IntersectionEventId,
    IntersectionEventLocation, IntersectionNetwork, IntersectionOrientation, IntersectionSpanId,
    IntersectionSpanUse, compute_boolean_intersections, prepare_boolean_with_external_tool,
    validate_solid_network,
};
use ngk::geometry::{Frame, Plane, Point3, PointCoincidence};
use ngk::model::Model;
use ngk::modeling::{faces, solids};
use ngk::topology::ModelEditError;
use ngk::topology::shape_keys::SolidKey;

fn two_blocks(
    first_origin: Point3,
    first_size: f64,
    second_origin: Point3,
    second_size: f64,
) -> (Model<ngk::StandardPayload>, SolidKey, SolidKey) {
    let (mut map, first) = solids::block_at(
        Frame::from_xy(first_origin, Vector3::x(), Vector3::y()),
        first_size,
        first_size,
        first_size,
    )
    .expect("first block")
    .into_model();
    let (tool, second) = solids::block_at(
        Frame::from_xy(second_origin, Vector3::x(), Vector3::y()),
        second_size,
        second_size,
        second_size,
    )
    .expect("second block")
    .into_model();
    let second = map
        .transaction(|edit| {
            let handle = edit.merge(tool.solid_unchecked(second));
            Ok::<_, ModelEditError>(edit.solid_key(handle).unwrap())
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
            let t = span.parameter_at(event.point);
            if !span.point_at(t).coincides(event.point, tolerances.linear) {
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
            let start_uv = surface.param_at(start).expect("planar parameter");
            let end_uv = surface.param_at(end).expect("planar parameter");
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
    let (mut target_map, target_face) = target.into_model();
    let (tool_map, tool_face) = tool.into_model();

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

#[test]
fn every_event_on_an_edge_lies_between_that_edge_s_own_vertices() {
    // The block's bottom edges are tangent to the cylinder's bottom circle at
    // (2,0,0) and (0,2,0). Their supports meet that circle twice more, at
    // (-2,0,0) and (0,-2,0) — points on the infinite lines, far outside the
    // edges resting on them. A curve is a support, never trimmed to its edge,
    // so nothing but the edge's own parameter span rules those out.
    let size = 2.0;
    let (mut map, block) = solids::block_at(Frame::xyz(), size, size, size)
        .expect("block")
        .into_model();
    let (tool, tool_cylinder) = solids::cylinder_at(Frame::xyz(), size, 2.0 * size)
        .expect("cylinder")
        .into_model();
    let cylinder = map
        .transaction(|edit| {
            let handle = edit.merge(tool.solid_unchecked(tool_cylinder));
            Ok::<_, ModelEditError>(edit.solid_key(handle).unwrap())
        })
        .expect("import cylinder");

    let plan = compute_boolean_intersections(
        &map,
        BooleanOperand::Solid(block),
        BooleanOperand::Solid(cylinder),
        BooleanOptions::default(),
    )
    .expect("the tangent pair intersects");

    let tolerance = plan.diagnostics.tolerances.linear;
    for (index, event) in plan.network.events().iter().enumerate() {
        for event_use in &event.uses {
            let (BooleanCell::Edge(key), IntersectionEventLocation::Edge { parameter }) =
                (event_use.cell, event_use.location)
            else {
                continue;
            };
            let edge = map.edge(key).expect("an event names a live edge");
            let interval = edge
                .parameter_interval()
                .expect("an attributed edge has a parameter interval");
            assert!(
                interval.contains(parameter, 1e-6),
                "event {index} at {:?} sits at {parameter} on edge {key:?}, \
                 whose own span is {interval:?}",
                event.point
            );
            assert!(
                edge.curve()
                    .expect("registered edge geometry")
                    .point_at(parameter)
                    .coincides(event.point, tolerance),
                "event {index}'s edge parameter must locate the event's own point"
            );
        }
    }
}
