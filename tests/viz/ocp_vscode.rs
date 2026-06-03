use ngk::modeling::solids::block;
use ngk::viz::ocp_vscode::show;
use ngk::viz::ocp_vscode::{OcpViewerOptions, payload_for_shape};

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

