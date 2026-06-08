use ngk::modeling::solids::block;
use ngk::viz::ocp_vscode::{OcpViewerOptions, payload_for_display, payload_for_shape};

#[test]
fn ocp_payload_for_shape_contains_three_cad_viewer_geometry() {
    let shape = block(1.0, 2.0, 3.0).expect("block primitive should build");
    let payload = payload_for_shape(&shape, &OcpViewerOptions::default())
        .expect("block should serialize to an OCP viewer payload");

    assert_eq!(payload.message_type, "data");
    assert_eq!(payload.count, 1);
    assert!(payload.config.render_edges);
    assert_eq!(payload.data.shapes.version, 3);
    assert_eq!(payload.data.shapes.parts.len(), 1);
    assert_eq!(payload.data.instances.len(), 1);

    let part = &payload.data.shapes.parts[0];
    assert_eq!(part.id, "/NGK/shape");
    assert_eq!(part.name, "shape");
    assert_eq!(part.kind, "shapes");
    assert_eq!(part.subtype, "solid");
    assert_eq!(part.shape.reference, 0);
    assert_eq!(part.state, [1, 1]);
    assert!(part.renderback);

    let geometry = &payload.data.instances[0];
    assert_eq!(geometry.vertices.dtype, "float32");
    assert_eq!(geometry.triangles.dtype, "int32");
    assert_eq!(geometry.vertices.codec, "b64");
    assert_eq!(geometry.triangles_per_face.shape[0], 6);
    assert_eq!(geometry.segments_per_edge.shape[0], 12);
    assert_eq!(geometry.triangles.shape[0] % 3, 0);
    assert_eq!(geometry.vertices.shape[0] % 3, 0);
    assert_eq!(geometry.normals.shape[0], geometry.vertices.shape[0]);
    assert_eq!(geometry.edges.shape[0] % 6, 0);
    assert_eq!(geometry.obj_vertices.shape[0], 8 * 3);
    assert!(!geometry.vertices.buffer.is_empty());
}

#[test]
fn ocp_payload_uses_options_for_name_port_and_camera() {
    let shape = block(1.0, 1.0, 1.0).expect("block primitive should build");
    let options = OcpViewerOptions {
        name: "debug_block".to_owned(),
        port: 3940,
        reset_camera: "keep".to_owned(),
        ..OcpViewerOptions::default()
    };

    let payload = payload_for_shape(&shape, &options)
        .expect("block should serialize to an OCP viewer payload");

    assert_eq!(payload.config.reset_camera, "keep");
    assert_eq!(payload.data.shapes.name, "NGK");
    assert_eq!(payload.data.shapes.parts[0].id, "/NGK/debug_block");
    assert_eq!(payload.data.shapes.parts[0].name, "debug_block");
}

#[test]
fn ocp_payload_accepts_slice_of_topology_views() {
    let shape = block(1.0, 2.0, 3.0).expect("block primitive should build");
    let faces = shape.solid().faces();
    let options = OcpViewerOptions {
        name: "block_faces".to_owned(),
        ..OcpViewerOptions::default()
    };

    let payload = payload_for_display(&faces[0..5], &options)
        .expect("face slice should serialize to an OCP viewer payload");

    assert_eq!(payload.count, 5);
    assert_eq!(payload.data.instances.len(), 5);
    assert_eq!(payload.data.shapes.parts.len(), 5);
    for (index, part) in payload.data.shapes.parts.iter().enumerate() {
        assert_eq!(part.id, format!("/NGK/block_faces_{index}"));
        assert_eq!(part.name, format!("block_faces_{index}"));
        assert_eq!(part.shape.reference, index as u32);
        assert!(part.renderback);
    }
    assert!(
        payload
            .data
            .instances
            .iter()
            .all(|geometry| geometry.triangles_per_face.shape[0] == 1)
    );
}

#[test]
fn ocp_payload_accepts_vec_of_topology_views() {
    let shape = block(1.0, 2.0, 3.0).expect("block primitive should build");
    let faces = shape.solid().faces();

    let payload = payload_for_display(&faces, &OcpViewerOptions::default())
        .expect("face vec should serialize to an OCP viewer payload");

    assert_eq!(payload.count, 6);
    assert_eq!(payload.data.instances.len(), 6);
    assert_eq!(payload.data.shapes.parts.len(), 6);
}

#[test]
fn ocp_payload_accepts_vec_of_vertices_as_point_part() {
    let shape = block(1.0, 2.0, 3.0).expect("block primitive should build");
    let vertices = shape.solid().vertices();

    let payload = payload_for_display(&vertices, &OcpViewerOptions::default())
        .expect("vertex vec should serialize to an OCP viewer payload");

    assert_eq!(payload.count, 8);
    for (part, geometry) in payload
        .data
        .shapes
        .parts
        .iter()
        .zip(&payload.data.instances)
    {
        assert_eq!(part.kind, "vertices");
        assert_eq!(part.subtype, "vertex");
        assert_eq!(part.state, [3, 1]);
        assert_eq!(part.size, Some(6.0));
        assert_eq!(geometry.obj_vertices.shape[0], 3);
        assert_eq!(geometry.vertices.shape[0], 0);
        assert_eq!(geometry.edges.shape[0], 0);
    }
}

#[test]
fn ocp_payload_accepts_vec_of_edges_as_edge_part() {
    let shape = block(1.0, 2.0, 3.0).expect("block primitive should build");
    let edges = shape.solid().edges();

    let payload = payload_for_display(&edges, &OcpViewerOptions::default())
        .expect("edge vec should serialize to an OCP viewer payload");

    assert_eq!(payload.count, 12);
    for (part, geometry) in payload
        .data
        .shapes
        .parts
        .iter()
        .zip(&payload.data.instances)
    {
        assert_eq!(part.kind, "edges");
        assert_eq!(part.subtype, "edge");
        assert_eq!(part.state, [3, 1]);
        assert_eq!(part.width, Some(2.0));
        assert_eq!(geometry.segments_per_edge.shape[0], 1);
        assert_eq!(geometry.vertices.shape[0], 0);
    }
}
