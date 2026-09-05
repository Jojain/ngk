use nalgebra::Vector3;
use ngk::builders;
use ngk::builders::boolean::{
    BooleanCell, BooleanOperand, BooleanOperation, BooleanOptions, BooleanSide,
    IntersectionSpanUse, boolean, compute_boolean_intersections, prepare_boolean,
    prepare_boolean_with_external_tool,
};

use ngk::builders::edges::add_line;
use ngk::builders::faces::{FaceImprint, add_rectangle, split_face_by_imprints};
use ngk::geometry::{Curve, Curve2, Line2, Point2, Surface};
use ngk::geometry::{Frame, LINEAR_TOLERANCE, Plane, Point3, PointCoincidence};
use ngk::modeling::{edges, faces, solids};
use ngk::topology::TopologyEditError;
use ngk::topology::attributes::VertexAttr;
use ngk::topology::gmap::GMap;
use ngk::topology::shape_keys::VertexKey;
use ngk::topology::validation::{validate_gmap, validate_solid_manifold};
use ngk::viz::debug_viewer::show;

fn isolated_vertex(point: Point3) -> (GMap<ngk::StandardPayload>, VertexKey) {
    let mut map = GMap::new();
    let key = map
        .transaction(|edit| {
            let dart = edit.add_dart();
            Ok::<_, TopologyEditError>(edit.add_vertex(VertexAttr::new(dart, point, ())))
        })
        .expect("isolated vertex");
    (map, key)
}

#[test]
fn external_crossing_edges_are_copied_and_split_on_both_sides() {
    let target =
        edges::line(Point3::new(-1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)).expect("target edge");
    let tool =
        edges::line(Point3::new(0.0, -1.0, 0.0), Point3::new(0.0, 1.0, 0.0)).expect("tool edge");
    let (mut target_map, target_edge) = target.into_map();
    let (tool_map, tool_edge) = tool.into_map();
    let tool_edge_count = tool_map.iter_edges().count();
    let tool_vertex_count = tool_map.iter_vertices().count();

    let prepared = prepare_boolean_with_external_tool(
        &mut target_map,
        BooleanOperand::Edge(target_edge),
        &tool_map,
        BooleanOperand::Edge(tool_edge),
        Default::default(),
    )
    .expect("crossing edges should prepare");

    assert_eq!(tool_map.iter_edges().count(), tool_edge_count);
    assert_eq!(tool_map.iter_vertices().count(), tool_vertex_count);
    assert_eq!(target_map.iter_edges().count(), 4);
    assert_eq!(
        prepared
            .edge_fragments(BooleanSide::First, target_edge)
            .len(),
        2
    );
    let BooleanOperand::Edge(imported_tool) = prepared.second else {
        panic!("expected imported edge operand")
    };
    assert_eq!(prepared.imported_tool, Some(prepared.second));
    assert_eq!(
        prepared
            .edge_fragments(BooleanSide::Second, imported_tool)
            .len(),
        2
    );
    assert_eq!(prepared.network.events().len(), 1);
    assert!(prepared.network.spans().is_empty());
    assert_eq!(prepared.network.events()[0].uses.len(), 2);
    prepared
        .network
        .validate(LINEAR_TOLERANCE)
        .expect("intersection network must be structurally valid");
    assert!(
        prepared.network.events()[0]
            .point
            .coincides(Point3::origin(), LINEAR_TOLERANCE)
    );
    validate_gmap(&target_map).expect("prepared edge map must remain valid");
}

#[test]
fn external_vertex_on_edge_splits_the_working_edge() {
    let target =
        edges::line(Point3::new(-1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)).expect("target edge");
    let (mut target_map, target_edge) = target.into_map();
    let (tool_map, tool_vertex) = isolated_vertex(Point3::origin());

    let prepared = prepare_boolean_with_external_tool(
        &mut target_map,
        BooleanOperand::Edge(target_edge),
        &tool_map,
        BooleanOperand::Vertex(tool_vertex),
        Default::default(),
    )
    .expect("vertex-on-edge contact should prepare");

    assert_eq!(
        prepared
            .edge_fragments(BooleanSide::First, target_edge)
            .len(),
        2
    );
    assert_eq!(target_map.iter_edges().count(), 2);
    assert_eq!(prepared.network.events().len(), 1);
    assert!(prepared.network.spans().is_empty());
    assert_eq!(tool_map.iter_vertices().count(), 1);
    validate_gmap(&target_map).expect("prepared vertex/edge map must remain valid");
}

#[test]
fn external_vertex_on_face_is_recorded_without_subdividing_the_face() {
    let target = faces::rectangle(Plane::xy(), 1.0, 1.0).expect("target face");
    let (mut target_map, target_face) = target.into_map();
    let (tool_map, tool_vertex) = isolated_vertex(Point3::new(0.5, 0.5, 0.0));

    let prepared = prepare_boolean_with_external_tool(
        &mut target_map,
        BooleanOperand::Face(target_face),
        &tool_map,
        BooleanOperand::Vertex(tool_vertex),
        Default::default(),
    )
    .expect("vertex-on-face contact should prepare");

    assert_eq!(
        prepared
            .face_fragments(BooleanSide::First, target_face)
            .len(),
        1
    );
    assert_eq!(prepared.network.events().len(), 1);
    assert!(prepared.network.spans().is_empty());
    validate_gmap(&target_map).expect("prepared vertex/face map must remain valid");
}

#[test]
fn perpendicular_faces_are_split_on_both_sides() {
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
        Default::default(),
    )
    .expect("perpendicular faces should prepare");

    assert_eq!(
        prepared
            .face_fragments(BooleanSide::First, target_face)
            .len(),
        2
    );
    let BooleanOperand::Face(imported_tool) = prepared.second else {
        panic!("expected imported face operand")
    };
    assert_eq!(
        prepared
            .face_fragments(BooleanSide::Second, imported_tool)
            .len(),
        2
    );
    assert_eq!(target_map.iter_faces().count(), 4);
    assert_eq!(prepared.network.spans().len(), 1);
    assert_eq!(prepared.network.events().len(), 2);
    assert_eq!(
        prepared.network.spans()[0]
            .uses
            .iter()
            .filter(|span_use| matches!(span_use, IntersectionSpanUse::Face { .. }))
            .count(),
        2
    );
    let event_face_use_counts = prepared
        .network
        .events()
        .iter()
        .map(|event| {
            event
                .uses
                .iter()
                .filter(|event_use| matches!(event_use.cell, BooleanCell::Face(_)))
                .count()
        })
        .collect::<Vec<_>>();
    assert!(
        event_face_use_counts.iter().all(|count| *count == 2),
        "expected two face incidences per endpoint, got {event_face_use_counts:?}"
    );
    assert!(prepared.network.spans().iter().all(|span| {
        prepared.network.event(span.start).is_some() && prepared.network.event(span.end).is_some()
    }));
    validate_gmap(&target_map).expect("prepared face map must remain valid");
}

#[test]
fn same_map_edges_are_split_without_duplication() {
    let mut map = ngk::topology::gmap::GMap::<ngk::StandardPayload>::new();
    let first = add_line(
        &mut map,
        Point3::new(-1.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
    )
    .expect("first edge");
    let second = add_line(
        &mut map,
        Point3::new(0.0, -1.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    )
    .expect("second edge");

    let prepared = prepare_boolean(
        &mut map,
        BooleanOperand::Edge(first),
        BooleanOperand::Edge(second),
        Default::default(),
    )
    .expect("same-map edges should prepare");

    assert!(!prepared.imported_second);
    assert_eq!(map.iter_edges().count(), 4);
    assert_eq!(prepared.edge_fragments(BooleanSide::First, first).len(), 2);
    assert_eq!(
        prepared.edge_fragments(BooleanSide::Second, second).len(),
        2
    );
}

#[test]
fn repeated_same_map_preparation_does_not_split_again() {
    let mut map = ngk::topology::gmap::GMap::<ngk::StandardPayload>::new();
    let first = add_line(
        &mut map,
        Point3::new(-1.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
    )
    .expect("first edge");
    let second = add_line(
        &mut map,
        Point3::new(0.0, -1.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    )
    .expect("second edge");

    prepare_boolean(
        &mut map,
        BooleanOperand::Edge(first),
        BooleanOperand::Edge(second),
        Default::default(),
    )
    .expect("first preparation");
    let counts = (
        map.dart_count(),
        map.iter_edges().count(),
        map.iter_vertices().count(),
    );
    prepare_boolean(
        &mut map,
        BooleanOperand::Edge(first),
        BooleanOperand::Edge(second),
        Default::default(),
    )
    .expect("repeated preparation");

    assert_eq!(
        (
            map.dart_count(),
            map.iter_edges().count(),
            map.iter_vertices().count()
        ),
        counts
    );
}

#[test]
fn failed_external_preparation_rolls_back_the_imported_copy() {
    let target = edges::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0)).expect("target");
    let tool = edges::line(Point3::origin(), Point3::new(0.0, 1.0, 0.0)).expect("tool");
    let (mut target_map, target_edge) = target.into_map();
    let (tool_map, tool_edge) = tool.into_map();
    let counts = (
        target_map.dart_count(),
        target_map.iter_edges().count(),
        target_map.iter_vertices().count(),
    );
    let mut options = BooleanOptions::default();
    options.intersections.linear_tolerance = -1.0;

    let result = prepare_boolean_with_external_tool(
        &mut target_map,
        BooleanOperand::Edge(target_edge),
        &tool_map,
        BooleanOperand::Edge(tool_edge),
        options,
    );

    assert!(result.is_err());
    assert_eq!(
        (
            target_map.dart_count(),
            target_map.iter_edges().count(),
            target_map.iter_vertices().count(),
        ),
        counts
    );
    validate_gmap(&target_map).expect("rollback must restore the target map");
}

#[test]
fn edge_face_contact_splits_the_edge_and_records_the_face_contact() {
    let target =
        edges::line(Point3::new(0.5, 0.5, -1.0), Point3::new(0.5, 0.5, 1.0)).expect("target edge");
    let tool = faces::rectangle(Plane::xy(), 1.0, 1.0).expect("tool face");
    let (mut target_map, target_edge) = target.into_map();
    let (tool_map, tool_face) = tool.into_map();

    let prepared = prepare_boolean_with_external_tool(
        &mut target_map,
        BooleanOperand::Edge(target_edge),
        &tool_map,
        BooleanOperand::Face(tool_face),
        Default::default(),
    )
    .expect("edge-face contact should prepare");

    assert_eq!(
        prepared
            .edge_fragments(BooleanSide::First, target_edge)
            .len(),
        2
    );
    let BooleanOperand::Face(imported_face) = prepared.second else {
        panic!("expected imported face")
    };
    assert_eq!(
        prepared
            .face_fragments(BooleanSide::Second, imported_face)
            .len(),
        1
    );
    assert!(prepared.network.events().iter().any(|event| {
        event
            .point
            .coincides(Point3::new(0.5, 0.5, 0.0), LINEAR_TOLERANCE)
    }));
    validate_gmap(&target_map).expect("edge-face preparation must remain valid");
}

#[test]
fn overlapping_external_solids_are_imported_and_split_on_both_sides() {
    let (mut target_map, target_solid) =
        solids::block_at(Frame::xyz(), 1.0, 1.0, 1.0)
            .expect("target block")
            .into_map();
    let (tool_map, tool_solid) = solids::block_at(
        Frame::from_xy(Point3::new(0.5, 0.5, 0.5), Vector3::x(), Vector3::y()),
        1.0,
        1.0,
        1.0,
    )
    .expect("tool block")
    .into_map();

    let prepared = prepare_boolean_with_external_tool(
        &mut target_map,
        BooleanOperand::Solid(target_solid),
        &tool_map,
        BooleanOperand::Solid(tool_solid),
        Default::default(),
    )
    .expect("overlapping solids should prepare");

    let BooleanOperand::Solid(imported_tool) = prepared.second else {
        panic!("expected imported solid")
    };
    assert!(!prepared.network.spans().is_empty());
    assert!(prepared.network.spans().iter().all(|span| {
        prepared.network.event(span.start).is_some() && prepared.network.event(span.end).is_some()
    }));
    let first_fragment_count = prepared
        .first_lineage
        .faces
        .values()
        .map(Vec::len)
        .sum::<usize>();
    let second_fragment_count = prepared
        .second_lineage
        .faces
        .values()
        .map(Vec::len)
        .sum::<usize>();
    assert!(
        first_fragment_count > 6,
        "expected target face fragments, got {first_fragment_count}"
    );
    assert!(
        second_fragment_count > 6,
        "expected tool face fragments, got {second_fragment_count}"
    );
    validate_solid_manifold(&target_map, target_solid).expect("target remains manifold");
    validate_solid_manifold(&target_map, imported_tool).expect("tool copy remains manifold");
}

#[test]
fn coplanar_partial_overlap_imprints_the_region_boundary_on_both_faces() {
    let target = faces::rectangle(Plane::xy(), 2.0, 2.0).expect("target face");
    let tool_plane = Plane::from_xy(Point3::new(1.0, 1.0, 0.0), Vector3::x(), Vector3::y());
    let tool = faces::rectangle(tool_plane, 2.0, 2.0).expect("tool face");
    let (mut target_map, target_face) = target.into_map();
    let (tool_map, tool_face) = tool.into_map();

    let prepared = prepare_boolean_with_external_tool(
        &mut target_map,
        BooleanOperand::Face(target_face),
        &tool_map,
        BooleanOperand::Face(tool_face),
        Default::default(),
    )
    .expect("coplanar overlap should prepare");

    let BooleanOperand::Face(imported_tool) = prepared.second else {
        panic!("expected imported face")
    };
    assert_eq!(prepared.network.regions().len(), 1);
    assert_eq!(
        prepared
            .face_fragments(BooleanSide::First, target_face)
            .len(),
        2
    );
    assert_eq!(
        prepared
            .face_fragments(BooleanSide::Second, imported_tool)
            .len(),
        2
    );
    validate_gmap(&target_map).expect("coplanar preparation must remain valid");
}

fn block_at(
    origin: Point3,
    size: f64,
) -> (ngk::topology::gmap::GMap<ngk::StandardPayload>, SolidKey) {
    let plane = Plane::from_xy(origin, Vector3::x(), Vector3::y());
    let base = faces::rectangle(plane, size, size).expect("block base");
    let (mut map, face) = base.into_map();
    let solid =
        add_extruded_face(&mut map, face, Vector3::new(0.0, 0.0, size)).expect("block extrusion");
    (map, solid)
}

#[test]
fn nurbs_face_intersection_does_not_bridge_an_inner_loop() {
    let mut map = GMap::<ngk::StandardPayload>::new();
    let first = add_rectangle(&mut map, Plane::xy(), 1.0, 1.0).unwrap();
    let points = [
        Point2::new(0.4, 0.4),
        Point2::new(0.6, 0.4),
        Point2::new(0.6, 0.6),
        Point2::new(0.4, 0.6),
        Point2::new(0.4, 0.4),
    ];
    let imprints = points
        .windows(2)
        .map(|pair| {
            FaceImprint::new(
                Curve::line(
                    Point3::new(pair[0].x, pair[0].y, 0.0),
                    Point3::new(pair[1].x, pair[1].y, 0.0),
                ),
                Curve2::Line(Line2::new(pair[0], pair[1])),
            )
        })
        .collect::<Vec<_>>();
    split_face_by_imprints(&mut map, first, &imprints).unwrap();
    let plane = Plane::from_xy(Point3::new(0.0, 0.5, -0.5), Vector3::x(), Vector3::z());
    let second = add_rectangle(&mut map, plane, 1.0, 1.0).unwrap();
    map.transaction(|edit| {
        for face in [first, second] {
            let attr = edit.face_attr_mut(face).unwrap();
            attr.surface = Surface::Nurbs(attr.surface.to_nurbs().unwrap());
        }
        Ok::<_, TopologyEditError>(())
    })
    .unwrap();
    let plan = compute_boolean_intersections(
        &map,
        BooleanOperand::Face(first),
        BooleanOperand::Face(second),
        BooleanOptions::default(),
    )
    .unwrap();
    let spans = plan.network.spans();
    assert!(!spans.is_empty());
    for span in spans {
        for i in 1..100 {
            let point = span.curve.point_at(i as f64 / 100.0);
            assert!(
                point.x <= 0.4 + LINEAR_TOLERANCE || point.x >= 0.6 - LINEAR_TOLERANCE,
                "intersection bridges the hole at {point:?}"
            );
        }
    }
}

#[test]
fn separated_faces_are_pruned_before_surface_intersection() {
    let mut map = GMap::<ngk::StandardPayload>::new();
    let first = add_rectangle(&mut map, Plane::xy(), 1.0, 1.0).unwrap();
    let second = add_rectangle(
        &mut map,
        Plane::from_xy(Point3::new(10.0, 0.0, 0.0), Vector3::x(), Vector3::y()),
        1.0,
        1.0,
    )
    .unwrap();
    let plan = compute_boolean_intersections(
        &map,
        BooleanOperand::Face(first),
        BooleanOperand::Face(second),
        BooleanOptions::default(),
    )
    .unwrap();
    assert_eq!(plan.diagnostics.candidate_pairs_tested, 0);
    assert_eq!(plan.diagnostics.candidate_pairs_pruned, 1);
}

#[test]
fn intersecting_faces_retain_both_sides_of_span_lineage() {
    let mut map = GMap::<ngk::StandardPayload>::new();
    let first = add_rectangle(&mut map, Plane::xy(), 1.0, 1.0).unwrap();
    let second = add_rectangle(
        &mut map,
        Plane::from_xy(Point3::new(0.0, 0.5, -0.5), Vector3::x(), Vector3::z()),
        1.0,
        1.0,
    )
    .unwrap();
    let result = prepare_boolean(
        &mut map,
        BooleanOperand::Face(first),
        BooleanOperand::Face(second),
        BooleanOptions::default(),
    )
    .unwrap();
    assert_eq!(result.span_edges.len(), 1);
    let sides = result.span_edges.values().next().unwrap();
    assert_eq!(sides[0].len(), 1);
    assert_eq!(sides[1].len(), 1);
    for side in sides {
        assert!(map.edge(side[0]).is_some());
    }
}

#[test]
fn planar_face_bounds_keep_every_overlap_on_a_translation_grid() {
    for x in [-2.0, -0.5, 0.0, 0.5, 2.0] {
        let mut map = GMap::<ngk::StandardPayload>::new();
        let first = add_rectangle(&mut map, Plane::xy(), 1.0, 1.0).unwrap();
        let second = add_rectangle(
            &mut map,
            Plane::from_xy(Point3::new(x, 0.5, -0.5), Vector3::x(), Vector3::z()),
            1.0,
            1.0,
        )
        .unwrap();
        let plan = compute_boolean_intersections(
            &map,
            BooleanOperand::Face(first),
            BooleanOperand::Face(second),
            BooleanOptions::default(),
        )
        .unwrap();
        assert_eq!(
            plan.diagnostics.candidate_pairs_tested,
            usize::from(x.abs() < 1.0)
        );
        assert_eq!(
            plan.diagnostics.candidate_pairs_tested + plan.diagnostics.candidate_pairs_pruned,
            1
        );
        if x.abs() < 1.0 {
            assert!(!plan.network.spans().is_empty());
        }
    }
}

#[test]
fn boolean_tolerances_scale_world_distances_but_not_parameters() {
    use ngk::builders::boolean::{BooleanTolerancePolicy, BooleanTolerances};
    let policy = BooleanTolerancePolicy::ModelScaled {
        base_linear: 1.0e-8,
    };
    let small = BooleanTolerances::resolve(policy, 0.01).unwrap();
    let large = BooleanTolerances::resolve(policy, 10.0).unwrap();
    assert!((large.linear / small.linear - 1000.0).abs() < 1.0e-10);
    assert_eq!(large.parameter, small.parameter);
    assert_eq!(large.angular, small.angular);
    assert_eq!(large.probe_margin, 100.0 * large.linear);
    assert!(BooleanTolerances::resolve(policy, f64::NAN).is_err());
}

#[test]
fn fixed_boolean_tolerances_survive_preparation_unchanged() {
    use ngk::builders::boolean::{BooleanTolerancePolicy, BooleanTolerances};
    let tolerances = BooleanTolerances::resolve(
        BooleanTolerancePolicy::ModelScaled {
            base_linear: 1.0e-7,
        },
        2.0,
    )
    .unwrap();
    let mut options = BooleanOptions::default();
    options.tolerances = BooleanTolerancePolicy::Fixed(tolerances);
    let mut map = GMap::<ngk::StandardPayload>::new();
    let first = add_line(
        &mut map,
        Point3::new(-1.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let second = add_line(
        &mut map,
        Point3::new(0.0, -1.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    )
    .unwrap();
    let result = prepare_boolean(
        &mut map,
        BooleanOperand::Edge(first),
        BooleanOperand::Edge(second),
        options,
    )
    .unwrap();
    assert_eq!(result.diagnostics.tolerances, tolerances);
    assert_eq!(result.first_lineage.edges[&first].len(), 2);
}

fn two_blocks(
    first_origin: Point3,
    first_size: f64,
    second_origin: Point3,
    second_size: f64,
) -> (GMap<ngk::StandardPayload>, SolidKey, SolidKey) {
    let (mut map, first) = block_at(first_origin, first_size);
    let (tool, second) = block_at(second_origin, second_size);
    let second = map
        .transaction(|edit| {
            let dart = edit.merge(tool.solid_unchecked(second));
            Ok::<_, TopologyEditError>(edit.solid_key(dart).unwrap())
        })
        .unwrap();
    (map, first, second)
}

#[test]
fn boolean_disjoint_results_follow_the_single_solid_contract() {
    use ngk::builders::boolean::{BooleanError, BooleanOperation, boolean};
    for operation in [
        BooleanOperation::Union,
        BooleanOperation::Intersection,
        BooleanOperation::Difference,
    ] {
        let (mut map, first, second) =
            two_blocks(Point3::origin(), 1.0, Point3::new(3.0, 0.0, 0.0), 1.0);
        let before = serde_json::to_value(&map).unwrap();
        let result = boolean(
            &mut map,
            first,
            second,
            operation,
            BooleanOptions::default(),
        );
        match operation {
            BooleanOperation::Union => {
                assert!(matches!(
                    result,
                    Err(BooleanError::DisconnectedResult { .. })
                ));
                assert_eq!(serde_json::to_value(&map).unwrap(), before);
            }
            BooleanOperation::Intersection => {
                assert!(matches!(result, Err(BooleanError::EmptyResult)));
                assert_eq!(serde_json::to_value(&map).unwrap(), before);
            }
            BooleanOperation::Difference => {
                let result = result.unwrap();
                validate_solid_manifold(&map, result.solid).unwrap();
                assert_eq!(map.solid_unchecked(result.solid).faces().len(), 6);
            }
        }
    }
}

#[test]
fn boolean_nested_difference_registers_a_closed_cavity() {
    use ngk::builders::boolean::{BooleanOperation, boolean};
    let (mut map, first, second) =
        two_blocks(Point3::origin(), 3.0, Point3::new(1.0, 1.0, 1.0), 1.0);
    let result = boolean(
        &mut map,
        first,
        second,
        BooleanOperation::Difference,
        BooleanOptions::default(),
    )
    .unwrap();
    validate_solid_manifold(&map, result.solid).unwrap();
    ngk::topology::validation::validate_solid_orientation(&map, result.solid).unwrap();
    assert_eq!(map.solid_unchecked(result.solid).shells().len(), 2);
    assert_eq!(map.solid_unchecked(result.solid).faces().len(), 12);
}

#[test]
fn boolean_overlapping_boxes_union_produces_one_closed_solid() {
    use ngk::builders::boolean::{BooleanOperation, boolean};
    let (mut map, first, second) =
        two_blocks(Point3::origin(), 2.0, Point3::new(1.0, 1.0, 1.0), 2.0);
    let result = boolean(
        &mut map,
        first,
        second,
        BooleanOperation::Union,
        BooleanOptions::default(),
    )
    .unwrap();

    validate_solid_manifold(&map, result.solid).unwrap();
    ngk::topology::validation::validate_solid_orientation(&map, result.solid).unwrap();
    assert_eq!(map.solid_unchecked(result.solid).shells().len(), 1);
    assert_eq!(map.solid_unchecked(result.solid).faces().len(), 12);
}

#[test]
fn boolean_overlapping_boxes_intersection_and_difference_are_closed() {
    use ngk::builders::boolean::{BooleanOperation, boolean};
    for (operation, faces) in [
        (BooleanOperation::Intersection, 6),
        (BooleanOperation::Difference, 9),
    ] {
        let (mut map, first, second) =
            two_blocks(Point3::origin(), 2.0, Point3::new(1.0, 1.0, 1.0), 2.0);
        let result = boolean(
            &mut map,
            first,
            second,
            operation,
            BooleanOptions::default(),
        )
        .unwrap();

        validate_solid_manifold(&map, result.solid).unwrap();
        ngk::topology::validation::validate_solid_orientation(&map, result.solid).unwrap();
        assert_eq!(map.solid_unchecked(result.solid).faces().len(), faces);
        for face in map.solid_unchecked(result.solid).faces() {
            assert!(
                result
                    .lineage
                    .first
                    .faces
                    .values()
                    .chain(result.lineage.second.faces.values())
                    .any(|descendants| descendants.contains(&face.key()))
            );
        }
    }
}

#[test]
fn boolean_union_preserves_topology_under_scale_and_operand_swap() {
    use ngk::builders::boolean::{BooleanOperation, boolean};
    for scale in [1.0e-3, 1.0e3] {
        let (mut map, first, second) = two_blocks(
            Point3::origin(),
            2.0 * scale,
            Point3::new(scale, scale, scale),
            2.0 * scale,
        );
        let result = boolean(
            &mut map,
            second,
            first,
            BooleanOperation::Union,
            BooleanOptions::default(),
        )
        .unwrap();
        let solid = map.solid_unchecked(result.solid);
        assert_eq!(solid.faces().len(), 12);
        assert_eq!(
            solid.vertices().len() as isize - solid.edges().len() as isize
                + solid.faces().len() as isize,
            2
        );
    }
}

#[test]
fn boolean_result_can_be_consumed_by_a_second_operation() {
    use ngk::builders::boolean::{BooleanOperation, boolean};
    let (mut map, first, second) =
        two_blocks(Point3::origin(), 2.0, Point3::new(1.0, 1.0, 1.0), 2.0);
    let union = boolean(
        &mut map,
        first,
        second,
        BooleanOperation::Union,
        BooleanOptions::default(),
    )
    .unwrap();
    let (tool, cavity) = block_at(Point3::new(0.25, 0.25, 0.25), 0.5);
    let cavity = map
        .transaction(|edit| {
            let dart = edit.merge(tool.solid_unchecked(cavity));
            Ok::<_, TopologyEditError>(edit.solid_key(dart).unwrap())
        })
        .unwrap();
    let result = boolean(
        &mut map,
        union.solid,
        cavity,
        BooleanOperation::Difference,
        BooleanOptions::default(),
    )
    .unwrap();
    validate_solid_manifold(&map, result.solid).unwrap();

    ngk::topology::validation::validate_solid_orientation(&map, result.solid).unwrap();
    assert_eq!(map.solid_unchecked(result.solid).shells().len(), 2);
    assert_eq!(map.solid_unchecked(result.solid).faces().len(), 18);
}

#[test]
fn boolean_intersection_is_closed_for_faces_larger_than_the_unit_patch() {
    use ngk::builders::boolean::{BooleanOperation, boolean};
    // Faces of these blocks are four units across, so their contacts sit at
    // surface parameters well past the unit patch an unbounded plane converts
    // to by default. The narrow phase has to use each face's own trim domain
    // to see them at all.
    let (mut map, first, second) =
        two_blocks(Point3::origin(), 4.0, Point3::new(2.0, 2.0, 2.0), 4.0);

    let result = boolean(
        &mut map,
        first,
        second,
        BooleanOperation::Intersection,
        BooleanOptions::default(),
    )
    .unwrap();

    let solid = map.solid_unchecked(result.solid);
    assert_eq!(solid.faces().len(), 6);
    assert_eq!(
        solid.vertices().len() as isize - solid.edges().len() as isize
            + solid.faces().len() as isize,
        2
    );
}

#[test]
fn edge_face_broad_phase_prunes_pairs_that_cannot_touch() {
    use ngk::builders::boolean::{BooleanOperand, compute_boolean_intersections};
    let (map, first, second) = two_blocks(Point3::origin(), 2.0, Point3::new(1.0, 1.0, 1.0), 2.0);

    let plan = compute_boolean_intersections(
        &map,
        BooleanOperand::Solid(first),
        BooleanOperand::Solid(second),
        BooleanOptions::default(),
    )
    .unwrap();

    let tested = plan.diagnostics.edge_face_pairs_tested;
    let pruned = plan.diagnostics.edge_face_pairs_pruned;
    // Two blocks give 12 edges against 6 faces in each direction. Only the pairs
    // near the shared corner can touch, so the narrow phase must see far fewer.
    assert_eq!(tested + pruned, 144, "{tested} tested, {pruned} pruned");
    assert!(
        tested < 40,
        "expected most edge/face pairs pruned, {tested} still tested"
    );
}

#[test]
fn boolean_union_of_boxes_sharing_a_full_face_closes_across_the_contact() {
    use ngk::builders::boolean::{BooleanOperation, boolean};
    let (mut map, first, second) =
        two_blocks(Point3::origin(), 1.0, Point3::new(1.0, 0.0, 0.0), 1.0);
    let result = boolean(
        &mut map,
        first,
        second,
        BooleanOperation::Union,
        BooleanOptions::default(),
    )
    .unwrap();

    validate_solid_manifold(&map, result.solid).unwrap();
    ngk::topology::validation::validate_solid_orientation(&map, result.solid).unwrap();
    let solid = map.solid_unchecked(result.solid);
    assert_eq!(solid.shells().len(), 1);
    assert_eq!(solid.faces().len(), 10);
    assert_eq!(
        solid.vertices().len() as isize - solid.edges().len() as isize
            + solid.faces().len() as isize,
        2
    );
}

#[test]
fn boolean_difference_of_boxes_sharing_a_full_face_keeps_the_first_operand() {
    use ngk::builders::boolean::{BooleanOperation, boolean};
    let (mut map, first, second) =
        two_blocks(Point3::origin(), 1.0, Point3::new(1.0, 0.0, 0.0), 1.0);
    let result = boolean(
        &mut map,
        first,
        second,
        BooleanOperation::Difference,
        BooleanOptions::default(),
    )
    .unwrap();

    validate_solid_manifold(&map, result.solid).unwrap();
    ngk::topology::validation::validate_solid_orientation(&map, result.solid).unwrap();
    assert_eq!(map.solid_unchecked(result.solid).faces().len(), 6);
}

#[test]
fn boolean_union_of_boxes_meeting_on_a_coplanar_partial_face_is_closed() {
    use ngk::builders::boolean::{BooleanOperation, boolean};
    let (mut map, first, second) =
        two_blocks(Point3::origin(), 2.0, Point3::new(1.0, 1.0, 2.0), 2.0);
    let result = boolean(
        &mut map,
        first,
        second,
        BooleanOperation::Union,
        BooleanOptions::default(),
    )
    .unwrap();

    validate_solid_manifold(&map, result.solid).unwrap();
    ngk::topology::validation::validate_solid_orientation(&map, result.solid).unwrap();
    let solid = map.solid_unchecked(result.solid);
    assert_eq!(solid.shells().len(), 1);
    assert_eq!(solid.faces().len(), 12);
}

#[test]
fn boolean_rejects_operands_meeting_only_on_an_edge_or_a_vertex() {
    use ngk::builders::boolean::{BooleanOperation, boolean};
    for tool_origin in [Point3::new(1.0, 1.0, 0.0), Point3::new(1.0, 1.0, 1.0)] {
        for operation in [
            BooleanOperation::Union,
            BooleanOperation::Intersection,
            BooleanOperation::Difference,
        ] {
            let (mut map, first, second) = two_blocks(Point3::origin(), 1.0, tool_origin, 1.0);
            let before = serde_json::to_value(&map).unwrap();
            let result = boolean(
                &mut map,
                first,
                second,
                operation,
                BooleanOptions::default(),
            );
            if operation == BooleanOperation::Difference {
                let result = result.unwrap();
                validate_solid_manifold(&map, result.solid).unwrap();
                assert_eq!(map.solid_unchecked(result.solid).faces().len(), 6);
            } else {
                assert!(
                    result.is_err(),
                    "{tool_origin:?} {operation:?} must not build a non-manifold solid"
                );
                assert_eq!(serde_json::to_value(&map).unwrap(), before);
            }
        }
    }
}

#[test]
fn boolean_coplanar_partial_contact_keeps_each_operand_whole_under_difference() {
    use ngk::builders::boolean::{BooleanError, BooleanOperation, boolean};
    // The operands meet on a square of the plane z = 2 only, so the regularized
    // difference is the untouched minuend and the intersection is empty.
    for (minuend_first, faces) in [(true, 7), (false, 7)] {
        let (mut map, first, second) =
            two_blocks(Point3::origin(), 2.0, Point3::new(1.0, 1.0, 2.0), 2.0);
        let (minuend, subtrahend) = if minuend_first {
            (first, second)
        } else {
            (second, first)
        };
        let result = boolean(
            &mut map,
            minuend,
            subtrahend,
            BooleanOperation::Difference,
            BooleanOptions::default(),
        )
        .unwrap();
        validate_solid_manifold(&map, result.solid).unwrap();
        ngk::topology::validation::validate_solid_orientation(&map, result.solid).unwrap();
        let solid = map.solid_unchecked(result.solid);
        assert_eq!(solid.shells().len(), 1);
        assert_eq!(solid.faces().len(), faces);
    }

    let (mut map, first, second) =
        two_blocks(Point3::origin(), 2.0, Point3::new(1.0, 1.0, 2.0), 2.0);
    let before = serde_json::to_value(&map).unwrap();
    assert!(matches!(
        boolean(
            &mut map,
            first,
            second,
            BooleanOperation::Intersection,
            BooleanOptions::default(),
        ),
        Err(BooleanError::EmptyResult)
    ));
    assert_eq!(serde_json::to_value(&map).unwrap(), before);
}

#[test]
fn boolean_of_nested_boxes_selects_the_enclosing_or_enclosed_boundary() {
    use ngk::builders::boolean::{BooleanError, BooleanOperation, boolean};
    for (operation, expected) in [
        (BooleanOperation::Union, Some(6)),
        (BooleanOperation::Intersection, Some(6)),
    ] {
        let (mut map, first, second) =
            two_blocks(Point3::origin(), 3.0, Point3::new(1.0, 1.0, 1.0), 1.0);
        let result = boolean(
            &mut map,
            first,
            second,
            operation,
            BooleanOptions::default(),
        )
        .unwrap();
        validate_solid_manifold(&map, result.solid).unwrap();
        ngk::topology::validation::validate_solid_orientation(&map, result.solid).unwrap();
        let solid = map.solid_unchecked(result.solid);
        assert_eq!(solid.shells().len(), 1);
        assert_eq!(solid.faces().len(), expected.unwrap());
    }

    // Subtracting the container from the contained solid leaves nothing.
    let (mut map, first, second) =
        two_blocks(Point3::origin(), 3.0, Point3::new(1.0, 1.0, 1.0), 1.0);
    let before = serde_json::to_value(&map).unwrap();
    assert!(matches!(
        boolean(
            &mut map,
            second,
            first,
            BooleanOperation::Difference,
            BooleanOptions::default(),
        ),
        Err(BooleanError::EmptyResult)
    ));
    assert_eq!(serde_json::to_value(&map).unwrap(), before);
}

#[test]
fn boolean_difference_is_computed_in_both_operand_orders() {
    use ngk::builders::boolean::{BooleanOperation, boolean};
    for swapped in [false, true] {
        let (mut map, first, second) =
            two_blocks(Point3::origin(), 2.0, Point3::new(1.0, 1.0, 1.0), 2.0);
        let (minuend, subtrahend) = if swapped {
            (second, first)
        } else {
            (first, second)
        };
        let result = boolean(
            &mut map,
            minuend,
            subtrahend,
            BooleanOperation::Difference,
            BooleanOptions::default(),
        )
        .unwrap();
        validate_solid_manifold(&map, result.solid).unwrap();
        ngk::topology::validation::validate_solid_orientation(&map, result.solid).unwrap();
        let solid = map.solid_unchecked(result.solid);
        assert_eq!(solid.shells().len(), 1);
        assert_eq!(solid.faces().len(), 9);
        assert_eq!(
            solid.vertices().len() as isize - solid.edges().len() as isize
                + solid.faces().len() as isize,
            2
        );
    }
}

/// Builds a size-2 block at the origin together with a coaxial cylinder.
///
/// The cylinder is centred on the block's axis at `(1, 1)`, so `base` and
/// `height` alone decide whether it passes through, stops inside, or protrudes.
fn block_with_cylinder(
    radius: f64,
    base: f64,
    height: f64,
) -> (GMap<ngk::StandardPayload>, SolidKey, SolidKey) {
    let disc = faces::circle(
        Plane::from_xy(Point3::new(1.0, 1.0, base), Vector3::x(), Vector3::y()),
        radius,
    )
    .expect("cylinder base");
    let (tool, tool_cylinder) = {
        let (mut map, face) = disc.into_map();
        let solid =
            add_extruded_face(&mut map, face, Vector3::new(0.0, 0.0, height)).expect("cylinder");
        (map, solid)
    };
    let (mut map, block) = block_at(Point3::origin(), 2.0);
    let cylinder = map
        .transaction(|edit| {
            let dart = edit.merge(tool.solid_unchecked(tool_cylinder));
            Ok::<_, TopologyEditError>(edit.solid_key(dart).unwrap())
        })
        .unwrap();
    (map, block, cylinder)
}

#[test]
fn boolean_difference_supports_a_cylindrical_through_hole() {
    use ngk::builders::boolean::{BooleanOperation, boolean};
    let (mut map, block, cylinder) = block_with_cylinder(0.5, -1.0, 4.0);

    let result = boolean(
        &mut map,
        block,
        cylinder,
        BooleanOperation::Difference,
        BooleanOptions::default(),
    )
    .unwrap();

    validate_solid_manifold(&map, result.solid).unwrap();
    ngk::topology::validation::validate_solid_orientation(&map, result.solid).unwrap();
    let solid = map.solid_unchecked(result.solid);

    assert_eq!(solid.shells().len(), 1);
    assert_eq!(solid.faces().len(), 7);
    let rims: Vec<_> = solid
        .faces()
        .into_iter()
        .filter(|face| !face.inner_loops().is_empty())
        .collect();
    assert_eq!(rims.len(), 2);

    // Both rims are built the same way: one closed loop of arcs on the bore
    // radius, every one of them shared with the single bore wall face.
    let mut walls = Vec::new();
    for rim in &rims {
        let inner = rim.inner_loops();
        assert_eq!(inner.len(), 1);
        let edges = inner[0].edges();
        assert!(edges.len() >= 2, "a rim needs at least two arcs");
        for edge in &edges {
            let neighbours: Vec<_> = edge
                .faces()
                .iter()
                .map(|face| face.key())
                .filter(|key| *key != rim.key())
                .collect();
            assert_eq!(
                neighbours.len(),
                1,
                "a rim arc joins its cap to exactly one other face"
            );
            walls.push(neighbours[0]);

            let curve = edge.curve().expect("a rim arc carries geometry");
            let mid = curve.point_at(0.5);
            let radius = ((mid.x - 1.0).powi(2) + (mid.y - 1.0).powi(2)).sqrt();
            assert!(
                (radius - 0.5).abs() <= LINEAR_TOLERANCE,
                "rim arc midpoint {mid:?} is off the bore radius ({radius})"
            );
        }
    }
    assert!(
        walls.windows(2).all(|pair| pair[0] == pair[1]),
        "both rims must border the same bore wall, got {walls:?}"
    );
}

#[test]
fn boolean_intersection_of_a_block_and_a_through_cylinder_is_the_common_core() {
    use ngk::builders::boolean::{BooleanOperation, boolean, solid_contains_point};
    let (mut map, block, cylinder) = block_with_cylinder(0.5, -1.0, 4.0);

    let result = boolean(
        &mut map,
        block,
        cylinder,
        BooleanOperation::Intersection,
        BooleanOptions::default(),
    )
    .unwrap();

    validate_solid_manifold(&map, result.solid).unwrap();
    ngk::topology::validation::validate_solid_orientation(&map, result.solid).unwrap();
    let solid = map.solid_unchecked(result.solid);
    assert_eq!(solid.shells().len(), 1);
    // The block's caps clip the cylinder to the block height, leaving the two
    // trimmed discs and the cylindrical wall between them.
    assert_eq!(solid.faces().len(), 3);
    assert!(
        solid
            .faces()
            .iter()
            .all(|face| face.inner_loops().is_empty())
    );
    for (point, expected) in [
        (Point3::new(1.0, 1.0, 1.0), true),
        (Point3::new(1.6, 1.0, 1.0), false),
        (Point3::new(1.0, 1.0, 2.5), false),
    ] {
        assert_eq!(
            solid_contains_point(&map, result.solid, point, BooleanOptions::default()).unwrap(),
            expected,
            "{point:?}"
        );
    }
}

#[test]
fn boolean_union_of_a_block_and_a_protruding_cylinder_opens_one_inner_loop() {
    use ngk::builders::boolean::{BooleanOperation, boolean, solid_contains_point};
    let (mut map, block, cylinder) = block_with_cylinder(0.5, 1.0, 2.0);

    let result = boolean(
        &mut map,
        block,
        cylinder,
        BooleanOperation::Union,
        BooleanOptions::default(),
    )
    .unwrap();

    validate_solid_manifold(&map, result.solid).unwrap();
    ngk::topology::validation::validate_solid_orientation(&map, result.solid).unwrap();
    let solid = map.solid_unchecked(result.solid);
    assert_eq!(solid.shells().len(), 1);
    // Six block faces, the protruding wall, and the cylinder's own top cap; the
    // stub below the block's top face is interior and is discarded.
    assert_eq!(solid.faces().len(), 8);
    assert_eq!(
        solid
            .faces()
            .iter()
            .filter(|face| !face.inner_loops().is_empty())
            .count(),
        1
    );
    for (point, expected) in [
        (Point3::new(1.0, 1.0, 1.0), true),
        (Point3::new(1.0, 1.0, 2.5), true),
        (Point3::new(1.6, 1.0, 2.5), false),
    ] {
        assert_eq!(
            solid_contains_point(&map, result.solid, point, BooleanOptions::default()).unwrap(),
            expected,
            "{point:?}"
        );
    }
}

#[test]
#[ignore = "curved faces are not yet split across their parameter seam"]
fn boolean_difference_crosses_two_cylindrical_holes() {
    // Both remaining gaps are the same one, in the trimming and face-splitting
    // layers rather than in the solver. The bore wall's own seam sits at `u = 0`
    // and one of the two wall/wall loops straddles it, so trim classification
    // drops that loop; the wall/plane loops do reach the network, but realizing
    // one on the crossing cylinder needs a chord that runs from the seam back to
    // the seam, which `FaceImprintCut` cannot express yet.
    use ngk::builders::boolean::{BooleanOperation, boolean, solid_contains_point};
    let (mut map, block, upright) = block_with_cylinder(0.5, -1.0, 4.0);
    let drilled = boolean(
        &mut map,
        block,
        upright,
        BooleanOperation::Difference,
        BooleanOptions::default(),
    )
    .unwrap()
    .solid;

    // The second bore is narrower than the first and crosses it, so its wall
    // meets the first bore's wall: the first operand pair with no planar face.
    let disc = faces::circle(
        Plane::from_xy(Point3::new(-1.0, 1.0, 1.0), Vector3::y(), Vector3::z()),
        0.3,
    )
    .expect("second bore base");
    let (tool, tool_cylinder) = {
        let (mut tool_map, face) = disc.into_map();
        let solid = add_extruded_face(&mut tool_map, face, Vector3::new(4.0, 0.0, 0.0))
            .expect("second bore");
        (tool_map, solid)
    };
    let lying = map
        .transaction(|edit| {
            let dart = edit.merge(tool.solid_unchecked(tool_cylinder));
            Ok::<_, TopologyEditError>(edit.solid_key(dart).unwrap())
        })
        .unwrap();

    let result = boolean(
        &mut map,
        drilled,
        lying,
        BooleanOperation::Difference,
        BooleanOptions::default(),
    )
    .unwrap();

    validate_gmap(&map).unwrap();
    validate_solid_manifold(&map, result.solid).unwrap();
    ngk::topology::validation::validate_solid_orientation(&map, result.solid).unwrap();
    assert_eq!(map.solid_unchecked(result.solid).shells().len(), 1);
    for (point, expected) in [
        // Solid material away from both bores.
        (Point3::new(0.2, 0.2, 1.0), true),
        // Inside the upright bore.
        (Point3::new(1.0, 1.0, 0.2), false),
        // Inside the crossing bore, outside the upright one.
        (Point3::new(0.3, 1.0, 1.0), false),
        // Between the two bore radii, so material again.
        (Point3::new(0.3, 1.0, 1.45), true),
    ] {
        assert_eq!(
            solid_contains_point(&map, result.solid, point, BooleanOptions::default()).unwrap(),
            expected,
            "{point:?}"
        );
    }
}

/// Bounding box of a face in the parameter domain of `surface`.
fn face_uv_extent(
    face: &ngk::topology::face::Face<'_, ngk::StandardPayload>,
    surface: &Surface,
) -> (Point2, Point2) {
    let mut min = Point2::new(f64::INFINITY, f64::INFINITY);
    let mut max = Point2::new(f64::NEG_INFINITY, f64::NEG_INFINITY);
    for vertex in face.vertices() {
        let point = *vertex.point().expect("face geometry");
        let uv = surface.closest_parameter(point).expect("planar parameter");
        min = Point2::new(min.x.min(uv.x), min.y.min(uv.y));
        max = Point2::new(max.x.max(uv.x), max.y.max(uv.y));
    }
    (min, max)
}

/// Whether two planar faces rest on the same plane, in either orientation.
fn share_a_plane(first: &Surface, second: &Surface) -> bool {
    let (Surface::Plane(first), Surface::Plane(second)) = (first, second) else {
        return false;
    };
    let parallel = first.normal().dot(&second.normal()).abs() > 1.0 - LINEAR_TOLERANCE;
    let offset = (second.origin() - first.origin())
        .dot(&first.normal())
        .abs();
    parallel && offset <= LINEAR_TOLERANCE
}

#[test]
fn boolean_union_shells_are_euler_two_and_resolve_to_operand_faces() {
    use ngk::builders::boolean::{BooleanOperation, boolean};
    let (mut map, first, second) =
        two_blocks(Point3::origin(), 2.0, Point3::new(1.0, 1.0, 1.0), 2.0);
    let result = boolean(
        &mut map,
        first,
        second,
        BooleanOperation::Union,
        BooleanOptions::default(),
    )
    .unwrap();

    let solid = map.solid_unchecked(result.solid);
    for shell in solid.shells() {
        let euler = shell.vertices().len() as isize - shell.edges().len() as isize
            + shell.faces().len() as isize;
        assert_eq!(euler, 2, "every result shell must be a topological sphere");
    }

    let sources = result
        .lineage
        .first
        .faces
        .values()
        .chain(result.lineage.second.faces.values())
        .flatten()
        .copied()
        .collect::<Vec<_>>();
    for face in solid.faces() {
        assert!(
            sources.contains(&face.key()),
            "result face {:?} has no operand source",
            face.key()
        );
    }

    let faces = solid.faces();
    for (index, face) in faces.iter().enumerate() {
        for other in &faces[index + 1..] {
            if !share_a_plane(face.surface(), other.surface()) {
                continue;
            }
            let (min, max) = face_uv_extent(face, face.surface());
            let (other_min, other_max) = face_uv_extent(other, face.surface());
            let overlaps = min.x < other_max.x - LINEAR_TOLERANCE
                && other_min.x < max.x - LINEAR_TOLERANCE
                && min.y < other_max.y - LINEAR_TOLERANCE
                && other_min.y < max.y - LINEAR_TOLERANCE;
            assert!(
                !overlaps,
                "coplanar result faces {:?} and {:?} overlap",
                face.key(),
                other.key()
            );
        }
    }
}

#[test]
fn boolean_results_classify_sample_points_like_the_mathematical_set() {
    use ngk::builders::boolean::{BooleanOperation, boolean, solid_contains_point};
    // A = [0,2]^3, B = [1,3]^3.
    let inside_a_only = Point3::new(0.5, 0.5, 0.5);
    let inside_both = Point3::new(1.5, 1.5, 1.5);
    let inside_b_only = Point3::new(2.5, 2.5, 2.5);
    let outside_both = Point3::new(2.5, 0.5, 0.5);
    for (operation, expectations) in [
        (
            BooleanOperation::Union,
            [
                (inside_a_only, true),
                (inside_both, true),
                (inside_b_only, true),
                (outside_both, false),
            ],
        ),
        (
            BooleanOperation::Intersection,
            [
                (inside_a_only, false),
                (inside_both, true),
                (inside_b_only, false),
                (outside_both, false),
            ],
        ),
        (
            BooleanOperation::Difference,
            [
                (inside_a_only, true),
                (inside_both, false),
                (inside_b_only, false),
                (outside_both, false),
            ],
        ),
    ] {
        let (mut map, first, second) =
            two_blocks(Point3::origin(), 2.0, Point3::new(1.0, 1.0, 1.0), 2.0);
        let result = boolean(
            &mut map,
            first,
            second,
            operation,
            BooleanOptions::default(),
        )
        .unwrap();
        for (point, expected) in expectations {
            let inside =
                solid_contains_point(&map, result.solid, point, BooleanOptions::default()).unwrap();
            assert_eq!(
                inside, expected,
                "{operation:?} membership at {point:?} must be {expected}"
            );
        }
    }
}

#[test]
fn boolean_union_topology_is_stable_under_face_reparameterization() {
    use ngk::builders::boolean::{BooleanOperation, boolean};
    // The same box, built on a plane frame rotated a quarter turn in its own
    // domain: identical geometry, different pcurve parameterization.
    let reparameterized = {
        let plane = Plane::from_xy(Point3::new(3.0, 1.0, 1.0), Vector3::y(), -Vector3::x());
        let base = faces::rectangle(plane, 2.0, 2.0).expect("rotated base");
        let (mut map, face) = base.into_map();
        let solid =
            add_extruded_face(&mut map, face, Vector3::new(0.0, 0.0, 2.0)).expect("rotated block");
        (map, solid)
    };

    let mut counts = Vec::new();
    for rotated in [false, true] {
        let (mut map, first, second) = if rotated {
            let (mut map, first) = block_at(Point3::origin(), 2.0);
            let (tool, block) = &reparameterized;
            let second = map
                .transaction(|edit| {
                    let dart = edit.merge(tool.solid_unchecked(*block));
                    Ok::<_, TopologyEditError>(edit.solid_key(dart).unwrap())
                })
                .unwrap();
            (map, first, second)
        } else {
            two_blocks(Point3::origin(), 2.0, Point3::new(1.0, 1.0, 1.0), 2.0)
        };
        let result = boolean(
            &mut map,
            first,
            second,
            BooleanOperation::Union,
            BooleanOptions::default(),
        )
        .unwrap();
        validate_solid_manifold(&map, result.solid).unwrap();
        let solid = map.solid_unchecked(result.solid);
        counts.push((
            solid.vertices().len(),
            solid.edges().len(),
            solid.faces().len(),
        ));
    }
    assert_eq!(
        counts[0], counts[1],
        "reparameterizing a face must not change the result topology"
    );
}

/// An axis-aligned box spanning `min` to `max`.
fn box_between(min: Point3, max: Point3) -> (GMap<ngk::StandardPayload>, SolidKey) {
    let plane = Plane::from_xy(min, Vector3::x(), Vector3::y());
    let base = faces::rectangle(plane, max.x - min.x, max.y - min.y).expect("box base");
    let (mut map, face) = base.into_map();
    let solid = add_extruded_face(&mut map, face, Vector3::new(0.0, 0.0, max.z - min.z))
        .expect("box extrusion");
    (map, solid)
}

#[test]
fn boolean_difference_of_a_through_slot_opens_an_inner_loop_on_both_caps() {
    use ngk::builders::boolean::solid_contains_point;
    // The planar counterpart of "box minus a through cylinder": the tool leaves
    // both caps with a hole and the result is a genus-one shell.
    let (mut map, block) = box_between(Point3::origin(), Point3::new(3.0, 3.0, 3.0));
    let (tool, slot) = box_between(Point3::new(1.0, 1.0, -1.0), Point3::new(2.0, 2.0, 4.0));
    let slot = map
        .transaction(|edit| {
            let dart = edit.merge(tool.solid_unchecked(slot));
            Ok::<_, TopologyEditError>(edit.solid_key(dart).unwrap())
        })
        .unwrap();

    let result = boolean(
        &mut map,
        block,
        slot,
        BooleanOperation::Difference,
        BooleanOptions::default(),
    )
    .unwrap();

    validate_gmap(&map).unwrap();
    validate_solid_manifold(&map, result.solid).unwrap();
    ngk::topology::validation::validate_solid_orientation(&map, result.solid).unwrap();
    let solid = map.solid_unchecked(result.solid);
    assert_eq!(solid.shells().len(), 1);
    assert_eq!(
        solid
            .faces()
            .iter()
            .filter(|face| !face.inner_loops().is_empty())
            .count(),
        2,
        "both caps must carry the shaft as an inner loop"
    );
    assert_eq!(solid.faces().len(), 10);
    // A genus-one shell: the two holed caps each cost one unit of Euler characteristic.
    let shell = &solid.shells()[0];
    assert_eq!(
        shell.vertices().len() as isize - shell.edges().len() as isize
            + shell.faces().len() as isize,
        2
    );

    for (point, expected) in [
        (Point3::new(0.5, 0.5, 1.5), true),
        (Point3::new(1.5, 1.5, 1.5), false),
        (Point3::new(2.5, 2.5, 1.5), true),
    ] {
        assert_eq!(
            solid_contains_point(&map, result.solid, point, BooleanOptions::default()).unwrap(),
            expected,
            "membership at {point:?} after cutting the slot"
        );
    }
}

#[test]
fn block_fused_with_cylinder_tangent_to_block_faces() {
    let size = 2.0;
    let (mut map, block_key) = block_at(Point3::origin(), size);
    let face = builders::faces::add_circle(&mut map, Plane::xy(), size).expect("should build");
    let cylinder = add_extruded_face(&mut map, face, Vector3::new(0.0, 0.0, 2.0 * size))
        .expect("should build");

    let result = boolean(
        &mut map,
        block_key,
        cylinder,
        BooleanOperation::Intersection,
        BooleanOptions::default(),
    );
    show(&map);
    assert!(result.is_ok(), "boolean union failed: {result:?}");

    assert!(
        map.iter_solids().count() == 1,
        "result should be a single solid"
    );
}
