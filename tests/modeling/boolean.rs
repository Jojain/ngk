use std::collections::HashSet;

use nalgebra::Vector3;
use ngk::geometry::{CurveCurveIntersection, Interval, Plane, Point3, SurfaceSurfaceIntersection};
use ngk::modeling::boolean::{
    AppliedFaceSectionKind, BooleanSource, BooleanSplitPlan, BooleanWorkspace, EdgeHandle,
    EdgeSplit, FaceHandle, FaceSection, FaceSectionKind,
};
use ngk::modeling::solids::block;
use ngk::modeling::{faces, sweep};
use ngk::topology::gmap::{Cell0, Cell2, Dim, GMap};
use ngk::topology::payload::StandardPayload;
use ngk::topology::shape::{Shape, SolidTag};
use ngk::topology::shape_keys::{EdgeKey, FaceKey};

#[test]
fn boolean_workspace_collects_operands_from_solid_shells() {
    let object = block(1.0, 1.0, 1.0).expect("object block should build");
    let tool = block(1.0, 1.0, 1.0).expect("tool block should build");

    let workspace = BooleanWorkspace::from_solids(&object.solid(), &tool.solid())
        .expect("workspace should index valid solids");

    assert_eq!(workspace.object().faces().len(), 6);
    assert_eq!(workspace.object().edges().len(), 12);
    assert_eq!(workspace.tool().faces().len(), 6);
    assert_eq!(workspace.tool().edges().len(), 12);
    assert_eq!(workspace.face_pair_count(), 36);
    assert_eq!(workspace.edge_face_pair_count(), 144);
    assert_eq!(workspace.edge_pair_count(), 144);
}

#[test]
fn boolean_workspace_records_same_domain_face_candidates() {
    let object = block(1.0, 1.0, 1.0).expect("object block should build");
    let tool = block(1.0, 1.0, 1.0).expect("tool block should build");

    let workspace = BooleanWorkspace::from_solids(&object.solid(), &tool.solid())
        .expect("workspace should evaluate face intersections");

    assert!(
        workspace
            .face_face_interferences()
            .iter()
            .any(|interf| matches!(interf.intersection, SurfaceSurfaceIntersection::Region)),
        "coincident blocks should expose same-domain face candidates"
    );
}

#[test]
fn face_interferences_reference_faces_by_handle() {
    let object = block(1.0, 1.0, 1.0).expect("object block should build");
    let tool = block(1.0, 1.0, 1.0).expect("tool block should build");

    let workspace = BooleanWorkspace::from_solids(&object.solid(), &tool.solid())
        .expect("workspace should evaluate face intersections");

    for interference in workspace.face_face_interferences() {
        assert_eq!(interference.object.source, BooleanSource::Object);
        assert_eq!(interference.tool.source, BooleanSource::Tool);
        assert!(
            workspace.object().contains(interference.object),
            "object handle should resolve in the object operand"
        );
        assert!(
            workspace.tool().contains(interference.tool),
            "tool handle should resolve in the tool operand"
        );
    }
}

#[test]
fn boolean_workspace_records_edge_face_interferences() {
    let object = block(1.0, 1.0, 1.0).expect("object block should build");
    let tool = block(1.0, 1.0, 1.0).expect("tool block should build");

    let workspace = BooleanWorkspace::from_solids(&object.solid(), &tool.solid())
        .expect("workspace should evaluate edge-face intersections");

    assert!(
        workspace
            .edge_face_interferences()
            .iter()
            .any(|interf| matches!(
                interf.intersection,
                ngk::geometry::CurveSurfaceIntersection::Overlap { .. }
            )),
        "coincident blocks should expose edge-on-face candidates"
    );
}

#[test]
fn edge_face_interferences_reference_operands_by_handle() {
    let object = block(1.0, 1.0, 1.0).expect("object block should build");
    let tool = block(1.0, 1.0, 1.0).expect("tool block should build");

    let workspace = BooleanWorkspace::from_solids(&object.solid(), &tool.solid())
        .expect("workspace should evaluate edge-face intersections");

    for interference in workspace.edge_face_interferences() {
        assert_ne!(interference.edge.source, interference.face.source);
        match interference.edge.source {
            BooleanSource::Object => {
                assert!(workspace.object().contains_edge(interference.edge));
                assert!(workspace.tool().contains_face(interference.face));
            }
            BooleanSource::Tool => {
                assert!(workspace.tool().contains_edge(interference.edge));
                assert!(workspace.object().contains_face(interference.face));
            }
        }
    }
}

#[test]
fn boolean_workspace_records_edge_edge_interferences() {
    let object = block(1.0, 1.0, 1.0).expect("object block should build");
    let tool = block(1.0, 1.0, 1.0).expect("tool block should build");

    let workspace = BooleanWorkspace::from_solids(&object.solid(), &tool.solid())
        .expect("workspace should evaluate edge-edge intersections");

    assert!(
        workspace
            .edge_edge_interferences()
            .iter()
            .any(|interf| matches!(interf.intersection, CurveCurveIntersection::Overlap { .. })),
        "coincident blocks should expose edge-on-edge candidates"
    );
}

#[test]
fn edge_edge_interferences_reference_edges_by_handle() {
    let object = block(1.0, 1.0, 1.0).expect("object block should build");
    let tool = block(1.0, 1.0, 1.0).expect("tool block should build");

    let workspace = BooleanWorkspace::from_solids(&object.solid(), &tool.solid())
        .expect("workspace should evaluate edge-edge intersections");

    for interference in workspace.edge_edge_interferences() {
        assert_eq!(interference.object.source, BooleanSource::Object);
        assert_eq!(interference.tool.source, BooleanSource::Tool);
        assert!(workspace.object().contains_edge(interference.object));
        assert!(workspace.tool().contains_edge(interference.tool));
    }
}

#[test]
fn boolean_workspace_builds_split_plan_from_interferences() {
    let object = block(1.0, 1.0, 1.0).expect("object block should build");
    let tool = block(1.0, 1.0, 1.0).expect("tool block should build");

    let workspace = BooleanWorkspace::from_solids(&object.solid(), &tool.solid())
        .expect("workspace should build a split plan");
    let plan = workspace.split_plan();

    assert!(
        plan.edge_overlaps().iter().any(|overlap| {
            overlap.edge.source == BooleanSource::Object
                && workspace.object().contains_edge(overlap.edge)
        }),
        "coincident blocks should produce object edge overlap intervals"
    );
    assert!(
        plan.face_sections()
            .iter()
            .any(|section| matches!(section.kind, FaceSectionKind::SameDomainRegion)),
        "coincident blocks should produce same-domain face sections"
    );
}

#[test]
fn split_plan_deduplicates_edge_split_parameters() {
    let object = block(1.0, 1.0, 1.0).expect("object block should build");
    let tool = block(1.0, 1.0, 1.0).expect("tool block should build");

    let workspace = BooleanWorkspace::from_solids(&object.solid(), &tool.solid())
        .expect("workspace should build a split plan");
    let plan = workspace.split_plan();

    for edge in workspace
        .object()
        .edges()
        .iter()
        .chain(workspace.tool().edges())
    {
        let parameters = plan
            .edge_splits()
            .iter()
            .filter(|split| split.edge == edge.handle())
            .map(|split| split.parameter)
            .collect::<Vec<_>>();

        assert!(
            parameters.windows(2).all(|pair| pair[0] < pair[1]),
            "split parameters for each edge should be sorted and unique"
        );
    }
}

#[test]
fn split_plan_applies_multiple_splits_to_current_operand_edge_segments() {
    let mut object = block(2.0, 2.0, 2.0).expect("object block should build");
    let mut tool = block(2.0, 2.0, 2.0).expect("tool block should build");
    let (edge_key, edge_attr) = object
        .map()
        .iter_edges()
        .next()
        .expect("object should have edges");
    let edge = EdgeHandle {
        source: BooleanSource::Object,
        dart: edge_attr.dart,
    };
    let adjacent_faces = incident_face_keys(object.map(), edge_key);
    let domain = edge_domain(object.map(), edge_key);
    let first = domain.start + domain.length() / 3.0;
    let second = domain.start + 2.0 * domain.length() / 3.0;
    let plan = BooleanSplitPlan::from_edge_splits([
        EdgeSplit {
            edge,
            parameter: second,
        },
        EdgeSplit {
            edge,
            parameter: first,
        },
    ]);

    let applied = plan
        .apply_to_maps(object.map_mut(), tool.map_mut())
        .expect("split plan should apply to object map");

    assert_eq!(applied.edge_splits().len(), 2);
    assert_eq!(object.map().cells(Dim::One).count(), 14);
    assert_eq!(object.map().cells(Dim::Zero).count(), 10);
    assert_eq!(tool.map().cells(Dim::One).count(), 12);
    assert_eq!(tool.map().cells(Dim::Zero).count(), 8);

    for face in adjacent_faces {
        let attr = object.map().face(face).expect("face should remain");
        let edges = attr.face(object.map()).outer_loop().edges();
        assert_eq!(edges.len(), 6);
        assert_eq!(attr.pcurves.len(), 6);
        assert!(
            edges
                .iter()
                .all(|edge| attr.pcurves.contains_key(&edge.dart))
        );
    }
}

#[test]
fn workspace_splits_isolated_operand_maps_from_real_interferences() {
    let object = block(2.0, 2.0, 2.0).expect("object block should build");
    let tool = shifted_block(Point3::new(1.0, -0.5, -0.5), 2.0, 2.0, 3.0);
    let object_edge_count = object.map().cells(Dim::One).count();
    let object_vertex_count = object.map().cells(Dim::Zero).count();
    let tool_edge_count = tool.map().cells(Dim::One).count();
    let tool_vertex_count = tool.map().cells(Dim::Zero).count();

    let workspace = BooleanWorkspace::from_solids(&object.solid(), &tool.solid())
        .expect("workspace should evaluate shifted blocks");

    assert!(
        !workspace.split_plan().edge_splits().is_empty(),
        "shifted overlapping blocks should produce edge split points"
    );

    let split = workspace
        .split_solid_shapes(&object, &tool)
        .expect("workspace split plan should apply to isolated operand maps");

    assert!(
        !split.application().edge_splits().is_empty(),
        "at least one real split should be applied"
    );
    assert!(
        split.application().face_sections().iter().any(|section| {
            matches!(
                &section.kind,
                AppliedFaceSectionKind::Point { uv, .. }
                    if uv.x.is_finite() && uv.y.is_finite()
            ) || matches!(
                &section.kind,
                AppliedFaceSectionKind::Curve {
                    pcurve: ngk::geometry::Curve2::Polyline(polyline),
                    ..
                } if polyline.points.len() >= 2
                    && polyline.points.iter().all(|point| point.x.is_finite() && point.y.is_finite())
            )
        }),
        "face sections should be projected to finite face UV imprints"
    );
    assert_eq!(object.map().cells(Dim::One).count(), object_edge_count);
    assert_eq!(object.map().cells(Dim::Zero).count(), object_vertex_count);
    assert_eq!(tool.map().cells(Dim::One).count(), tool_edge_count);
    assert_eq!(tool.map().cells(Dim::Zero).count(), tool_vertex_count);
    assert!(
        split.object_map().cells(Dim::One).count() > object_edge_count
            || split.tool_map().cells(Dim::One).count() > tool_edge_count,
        "split operand maps should contain additional edge segments"
    );
}

#[test]
fn split_plan_applies_face_section_imprints_to_split_operand_faces() {
    let mut object = faces::rectangle(Plane::xy(), 2.0, 2.0).expect("object face should build");
    let mut tool = faces::rectangle(Plane::xy(), 1.0, 1.0).expect("tool face should build");
    let face = FaceHandle {
        source: BooleanSource::Object,
        dart: object.face().outer_loop().dart,
    };
    let plan = BooleanSplitPlan::from_face_sections([FaceSection {
        face,
        kind: FaceSectionKind::Curve {
            points: vec![Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 2.0, 0.0)],
        },
    }]);

    let applied = plan
        .apply_to_maps(object.map_mut(), tool.map_mut())
        .expect("face section plan should split object map");

    assert_eq!(applied.face_sections().len(), 1);
    assert_eq!(applied.face_splits().len(), 1);
    assert_eq!(object.map().iter_faces().count(), 2);
    assert_eq!(object.map().iter_edges().count(), 5);
    assert_eq!(tool.map().iter_faces().count(), 1);
}

fn incident_face_keys(g: &GMap<StandardPayload>, edge: EdgeKey) -> Vec<FaceKey> {
    let edge_dart = g.edge(edge).expect("edge should exist").dart;
    let mut seen = HashSet::new();
    g.orbit(edge_dart, g.orbit_indices(Dim::One))
        .filter_map(|dart| g.attribute::<Cell2>(dart).copied())
        .filter(|face| seen.insert(*face))
        .collect()
}

fn shifted_block(
    origin: Point3,
    x_size: f64,
    y_size: f64,
    z_size: f64,
) -> Shape<SolidTag, StandardPayload> {
    let base = faces::rectangle(
        Plane::from_xy(origin, Vector3::x(), Vector3::y()),
        x_size,
        y_size,
    )
    .expect("shifted rectangle should build");

    sweep::extrude_face(base, Vector3::new(0.0, 0.0, z_size)).expect("shifted block should extrude")
}

fn edge_domain(g: &GMap<StandardPayload>, edge: EdgeKey) -> Interval {
    let attr = g.edge(edge).expect("edge should exist");
    let start = g
        .attribute::<Cell0>(attr.dart)
        .expect("edge start should have a vertex")
        .point;
    let end_dart = g.alpha(Dim::Zero, attr.dart);
    let end = g
        .attribute::<Cell0>(end_dart)
        .expect("edge end should have a vertex")
        .point;
    attr.curve.parameters_between(start, end).ordered()
}
