use ngk::geometry::{CurveCurveIntersection, SurfaceSurfaceIntersection};
use ngk::modeling::boolean::{BooleanSource, BooleanWorkspace, FaceSectionKind};
use ngk::modeling::solids::block;

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
