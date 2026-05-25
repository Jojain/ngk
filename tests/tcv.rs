use ngk::geometry::{Plane, Point3};
use ngk::modeling::{edges, faces, profiles, solids};
use ngk::tcv::{TcvOptions, to_tcv};

fn only_leaf(root: &ngk::tcv::TcvNode) -> &ngk::tcv::TcvNode {
    let parts = root.parts.as_ref().expect("root should be a group");
    assert_eq!(parts.len(), 1);
    &parts[0]
}

#[test]
fn edge_tcv_exports_edge_leaf_with_segment_group() {
    let shape = edges::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0)).expect("line builds");
    let tcv = to_tcv(&shape, TcvOptions::named("line")).expect("edge should export");
    let leaf = only_leaf(&tcv);
    let data = leaf.shape.as_ref().expect("leaf should carry geometry");

    assert_eq!(leaf.kind.as_deref(), Some("edges"));
    assert_eq!(leaf.state, Some([3, 1]));
    assert_eq!(data.segments_per_edge.len(), 1);
    assert!(data.segments_per_edge[0] > 0);
    assert_eq!(data.obj_vertices.len(), 6);
}

#[test]
fn profile_tcv_exports_one_segment_group_per_edge() {
    let shape = profiles::rectangle(Plane::xy(), 2.0, 3.0).expect("profile builds");
    let tcv = to_tcv(&shape, TcvOptions::named("profile")).expect("profile should export");
    let leaf = only_leaf(&tcv);
    let data = leaf.shape.as_ref().expect("leaf should carry geometry");

    assert_eq!(leaf.kind.as_deref(), Some("edges"));
    assert_eq!(data.segments_per_edge.len(), 4);
    assert!(data.segments_per_edge.iter().all(|count| *count > 0));
    assert_eq!(data.obj_vertices.len(), 12);
}

#[test]
fn face_tcv_exports_face_mesh_and_boundary_edges() {
    let shape = faces::rectangle(Plane::xy(), 2.0, 3.0).expect("face builds");
    let tcv = to_tcv(&shape, TcvOptions::named("face")).expect("face should export");
    let leaf = only_leaf(&tcv);
    let data = leaf.shape.as_ref().expect("leaf should carry geometry");

    assert_eq!(leaf.kind.as_deref(), Some("shapes"));
    assert_eq!(leaf.subtype.as_deref(), Some("face"));
    assert_eq!(data.triangles_per_face.len(), 1);
    assert!(!data.vertices.is_empty());
    assert!(!data.triangles.is_empty());
    assert_eq!(data.segments_per_edge.len(), 4);
    assert!(data.segments_per_edge.iter().all(|count| *count > 0));
}

#[test]
fn solid_tcv_exports_block_topology_groups_and_bbox() {
    let shape = solids::block(1.0, 2.0, 3.0).expect("block builds");
    let tcv = to_tcv(&shape, TcvOptions::named("block")).expect("solid should export");
    let leaf = only_leaf(&tcv);
    let data = leaf.shape.as_ref().expect("leaf should carry geometry");
    let bb = tcv.bb.as_ref().expect("root should carry bounding box");

    assert_eq!(leaf.kind.as_deref(), Some("shapes"));
    assert_eq!(leaf.subtype.as_deref(), Some("solid"));
    assert_eq!(data.triangles_per_face.len(), 6);
    assert_eq!(data.segments_per_edge.len(), 12);
    assert_eq!(data.obj_vertices.len(), 24);
    assert!(bb.xmax > bb.xmin);
    assert!(bb.ymax > bb.ymin);
    assert!(bb.zmax > bb.zmin);
}
