use nalgebra::Vector3;
use ngk::builders::boolean::{
    BooleanCell, BooleanOperand, BooleanOptions, BooleanSide, IntersectionSpanUse, prepare_boolean,
    prepare_boolean_with_external_tool,
};
use ngk::builders::edges::add_line;
use ngk::builders::solids::add_extruded_face;
use ngk::geometry::{LINEAR_TOLERANCE, Plane, Point3, PointCoincidence};
use ngk::modeling::{edges, faces};
use ngk::topology::TopologyEditError;
use ngk::topology::attributes::VertexAttr;
use ngk::topology::gmap::GMap;
use ngk::topology::shape_keys::SolidKey;
use ngk::topology::shape_keys::VertexKey;
use ngk::topology::validation::{validate_gmap, validate_solid_manifold};

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
    let (mut target_map, target_solid) = block_at(Point3::origin(), 1.0);
    let (tool_map, tool_solid) = block_at(Point3::new(0.5, 0.5, 0.5), 1.0);

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
