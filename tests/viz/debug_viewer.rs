use std::io::{Read, Write};
use std::net::TcpListener;
use std::thread;

use nalgebra::Vector3;
use radians::Rad64;

use ngk::builders::edges::add_edge;
use ngk::builders::faces::add_polygon_with_holes;
use ngk::builders::revolve::add_revolved_edge;
use ngk::geometry::axis::Axis3;
use ngk::geometry::{Curve, LINEAR_TOLERANCE, Plane, Point3};
use ngk::modeling::solids::block;
use ngk::topology::gmap::{Cell0, Cell1, Dart, Dim, GMap, TopologyEditError};
use ngk::topology::payload::StandardPayload;
use ngk::viz::debug_viewer::{
    DebugViewerOptions, DebugViewerPayload, payload_for_gmap, show_gmap_with_options,
};

fn two_faces_gmap() -> GMap<StandardPayload> {
    let mut g = GMap::<StandardPayload>::new();
    let p1 = Point3::new(0.0, 0.0, 0.0);
    let p2 = Point3::new(1.0, 0.0, 0.0);
    let p3 = Point3::new(1.0, 1.0, 0.0);
    let p4 = Point3::new(0.0, 1.0, 0.0);
    let p5 = Point3::new(2.0, 0.0, 0.0);
    let p6 = Point3::new(2.0, 1.0, 0.0);

    add_polygon_with_holes(&mut g, Plane::xy(), &[p1, p2, p3, p4], &[])
        .expect("left face should build");
    add_polygon_with_holes(&mut g, Plane::xy(), &[p2, p5, p6, p3], &[])
        .expect("right face should build");
    let left_edge = g.cell_key_unchecked::<Cell1>(Dart::new(2));
    let right_edge = g.cell_key_unchecked::<Cell1>(Dart::new(15));
    let left_start = g.cell_key_unchecked::<Cell0>(Dart::new(2));
    let right_start = g.cell_key_unchecked::<Cell0>(Dart::new(15));
    let left_end = g.cell_key_unchecked::<Cell0>(Dart::new(3));
    let right_end = g.cell_key_unchecked::<Cell0>(Dart::new(14));
    g.transaction(|edit| {
        edit.sew(Dim::Two, Dart::new(2), Dart::new(15))?;
        edit.merge_edges_into(left_edge, right_edge);
        edit.merge_vertices_into(left_start, right_start);
        edit.merge_vertices_into(left_end, right_end);
        Ok::<_, TopologyEditError>(())
    })
    .expect("shared edge should sew");
    g
}

fn full_turn_revolved_edge_payload(start: Point3, end: Point3) -> DebugViewerPayload {
    let mut g = GMap::<StandardPayload>::new();
    let edge =
        add_edge(&mut g, start, end, Curve::line(start, end)).expect("source edge should build");
    add_revolved_edge(
        &mut g,
        edge,
        Axis3::new(Point3::origin(), Vector3::z()),
        Rad64::FULL_TURN,
    )
    .expect("full revolution should build");
    payload_for_gmap(&g, &DebugViewerOptions::default())
}

fn assert_full_turn_revolve_is_visible(payload: &DebugViewerPayload, expected_edges: usize) {
    assert_eq!(payload.scene.edges.len(), expected_edges);
    assert!(payload.scene.edges.iter().all(|edge| {
        edge.polyline.windows(2).any(|segment| {
            let start = Point3::from(segment[0]);
            let end = Point3::from(segment[1]);
            (end - start).norm() > LINEAR_TOLERANCE
        })
    }));

    let face = payload
        .scene
        .faces
        .first()
        .expect("revolved face should be rendered");
    assert!(face.indices.chunks_exact(3).any(|triangle| {
        let a = Point3::from(face.positions[triangle[0] as usize]);
        let b = Point3::from(face.positions[triangle[1] as usize]);
        let c = Point3::from(face.positions[triangle[2] as usize]);
        (b - a).cross(&(c - a)).norm() > LINEAR_TOLERANCE
    }));
}

#[test]
fn debug_scene_renders_full_turn_revolved_edge_with_inner_loop() {
    let payload =
        full_turn_revolved_edge_payload(Point3::new(1.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0));

    assert_full_turn_revolve_is_visible(&payload, 2);
}

#[test]
fn debug_scene_renders_full_turn_revolved_edge_without_inner_loop() {
    let payload = full_turn_revolved_edge_payload(Point3::origin(), Point3::new(2.0, 0.0, 0.0));

    assert_full_turn_revolve_is_visible(&payload, 1);
}

#[test]
fn debug_payload_contains_scene_gmap_and_inspection_metadata() {
    let g = two_faces_gmap();
    let payload = payload_for_gmap(&g, &DebugViewerOptions::default());

    assert_eq!(payload.kind, "ngk.debug.v1");
    assert_eq!(payload.name, "shape");
    assert_eq!(payload.scene.faces.len(), 2);
    assert_eq!(payload.gmap.dimension, 4);
    assert_eq!(payload.gmap.dart_count, 16);
    assert_eq!(payload.gmap.alphas[2][2], 15);
    assert_eq!(payload.gmap.alphas[2][15], 2);

    assert_eq!(payload.metadata.faces.len(), 2);
    assert_eq!(payload.metadata.edges.len(), 7);
    assert_eq!(payload.metadata.vertices.len(), 6);
    assert!(
        payload
            .metadata
            .faces
            .iter()
            .all(|face| face.surface.kind == "plane")
    );
    assert!(
        payload
            .metadata
            .faces
            .iter()
            .all(|face| !face.normals.is_empty())
    );
    assert!(
        payload
            .metadata
            .faces
            .iter()
            .all(|face| face.normals.len() == 100)
    );
    assert!(
        payload
            .metadata
            .faces
            .iter()
            .all(|face| face.outer_loop.len() == 4)
    );
    assert_eq!(payload.selection.faces.len(), payload.scene.faces.len());
    assert_eq!(payload.selection.edges.len(), payload.scene.edges.len());
    assert_eq!(
        payload.selection.vertices.len(),
        payload.scene.vertices.len()
    );
    for face in &payload.metadata.faces {
        for pcurve in &face.pcurves {
            assert!(
                payload
                    .metadata
                    .edges
                    .iter()
                    .any(|edge| edge.key == pcurve.edge_key)
            );
            assert!(
                payload
                    .metadata
                    .vertices
                    .iter()
                    .any(|vertex| vertex.key == pcurve.start_vertex_key)
            );
            assert!(
                payload
                    .metadata
                    .vertices
                    .iter()
                    .any(|vertex| vertex.key == pcurve.end_vertex_key)
            );
        }
    }
}

#[test]
fn block_debug_metadata_uses_oriented_normals_and_boundary_ordered_pcurves() {
    let block = block(1.0, 2.0, 3.0).expect("block should build");
    let payload = payload_for_gmap(block.map(), &DebugViewerOptions::default());

    for face in &payload.metadata.faces {
        let pcurve_darts = face
            .pcurves
            .iter()
            .map(|pcurve| pcurve.dart)
            .collect::<Vec<_>>();
        assert_eq!(
            pcurve_darts, face.outer_loop,
            "face {} pcurves should follow outer-loop order",
            face.key
        );
    }

    let bottom = payload
        .metadata
        .faces
        .iter()
        .find(|face| {
            !face.normals.is_empty()
                && face
                    .normals
                    .iter()
                    .all(|sample| sample.origin[2].abs() <= 1.0e-9)
        })
        .expect("block should expose its bottom face");

    assert!(
        bottom
            .normals
            .iter()
            .all(|sample| sample.direction[2] < 0.0),
        "bottom-face debug normals should point outward"
    );
}

#[test]
fn debug_payload_includes_profile_and_sheet_topology() {
    let block = block(1.0, 2.0, 3.0).expect("block should build");
    let payload = payload_for_gmap(block.map(), &DebugViewerOptions::default());

    assert_eq!(
        payload.metadata.profiles.len(),
        block.map().iter_profiles().count()
    );
    assert_eq!(
        payload.metadata.sheets.len(),
        block.map().iter_sheets().count()
    );
    assert!(payload.metadata.profiles.iter().all(|profile| {
        profile.darts.contains(&profile.representative_dart)
            && payload.gmap.darts[profile.representative_dart as usize]
                .profile
                .as_deref()
                == Some(profile.key.as_str())
    }));
    assert!(payload.metadata.sheets.iter().all(|sheet| {
        sheet.darts.contains(&sheet.representative_dart)
            && payload.gmap.darts[sheet.representative_dart as usize]
                .sheet
                .as_deref()
                == Some(sheet.key.as_str())
    }));
}

#[test]
fn debug_show_posts_json_to_configured_endpoint() {
    let listener = TcpListener::bind("127.0.0.1:0").expect("test listener should bind");
    let port = listener
        .local_addr()
        .expect("listener has local addr")
        .port();

    let handle = thread::spawn(move || {
        let (mut stream, _) = listener.accept().expect("show should connect");
        let mut request = Vec::new();
        let mut buffer = [0_u8; 4096];
        loop {
            let read = stream.read(&mut buffer).expect("request should read");
            if read == 0 {
                break;
            }
            request.extend_from_slice(&buffer[..read]);
            let text = String::from_utf8_lossy(&request);
            if let Some(header_end) = text.find("\r\n\r\n") {
                let headers = &text[..header_end];
                let content_length = headers
                    .lines()
                    .find_map(|line| line.strip_prefix("Content-Length: "))
                    .and_then(|value| value.parse::<usize>().ok())
                    .expect("request should include content length");
                if request.len() >= header_end + 4 + content_length {
                    break;
                }
            }
        }
        stream
            .write_all(b"HTTP/1.1 204 No Content\r\nContent-Length: 0\r\n\r\n")
            .expect("response should write");
        String::from_utf8(request).expect("request should be utf8")
    });

    let g = two_faces_gmap();
    let options = DebugViewerOptions {
        port,
        name: "two_faces".to_owned(),
        ..DebugViewerOptions::default()
    };
    show_gmap_with_options(&g, &options).expect("debug show should post payload");

    let request = handle.join().expect("listener thread should finish");
    assert!(request.starts_with("POST /__ngk_debug/dumps HTTP/1.1"));
    assert!(request.contains("Content-Type: application/json"));
    assert!(request.contains("\"kind\":\"ngk.debug.v1\""));
    assert!(request.contains("\"name\":\"two_faces\""));
}
