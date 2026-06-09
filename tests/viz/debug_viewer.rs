use std::io::{Read, Write};
use std::net::TcpListener;
use std::thread;

use ngk::builders::faces::add_polygon_with_holes;
use ngk::geometry::{Plane, Point3};
use ngk::topology::gmap::{Dart, Dim, GMap};
use ngk::topology::payload::StandardPayload;
use ngk::viz::debug_viewer::{DebugViewerOptions, payload_for_gmap, show_gmap_with_options};

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
    g.sew(Dim::Two, Dart::new(2), Dart::new(15))
        .expect("shared edge should sew");
    g
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
    assert_eq!(payload.metadata.edges.len(), 8);
    assert_eq!(payload.metadata.vertices.len(), 8);
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
