use ngk::geometry::{Curve, Point3};
use ngk::modeling::solids::block;
use ngk::topology::StandardPayload;
use ngk::topology::attributes::{EdgeAttr, VertexAttr};
use ngk::topology::gmap::{Dim, GMap, TopologyEditError};
use ngk::topology::payload::Payload;

#[derive(Clone)]
struct SerializablePayload;

impl Payload for SerializablePayload {
    type V = u32;
    type E = u32;
    type Profile = u32;
    type F = u32;
    type Sheet = u32;
    type S = u32;
}

#[test]
fn gmap_round_trips_through_json_without_losing_state() {
    let block = block(1.0, 2.0, 3.0).expect("block should build");
    let serialized = serde_json::to_string(block.map()).expect("gmap should serialize");

    let restored: GMap<StandardPayload> =
        serde_json::from_str(&serialized).expect("gmap should deserialize");
    let original_value: serde_json::Value =
        serde_json::from_str(&serialized).expect("serialized gmap should be valid JSON");
    let restored_value = serde_json::to_value(&restored).expect("restored gmap should serialize");

    assert_eq!(restored_value, original_value);
    assert_eq!(restored.dart_count(), block.map().dart_count());
    assert_eq!(
        restored.iter_vertices().count(),
        block.map().iter_vertices().count()
    );
    assert_eq!(
        restored.iter_edges().count(),
        block.map().iter_edges().count()
    );
    assert_eq!(
        restored.iter_profiles().count(),
        block.map().iter_profiles().count()
    );
    assert_eq!(
        restored.iter_faces().count(),
        block.map().iter_faces().count()
    );
    assert_eq!(
        restored.iter_sheets().count(),
        block.map().iter_sheets().count()
    );
    assert_eq!(
        restored.iter_solids().count(),
        block.map().iter_solids().count()
    );
}

#[test]
fn gmap_round_trips_serializable_custom_payloads() {
    let mut gmap = GMap::<SerializablePayload>::new();
    gmap.transaction(|edit| {
        let start = edit.add_dart();
        let end = edit.add_dart();
        edit.link(Dim::Zero, start, end)?;
        edit.add_vertex(VertexAttr::new(start, Point3::origin(), 10));
        edit.add_vertex(VertexAttr::new(end, Point3::new(1.0, 0.0, 0.0), 20));
        edit.add_edge(EdgeAttr::new(
            start,
            Curve::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0)),
            30,
        ));
        Ok::<_, TopologyEditError>(())
    })
    .expect("custom-payload edge should build");

    let serialized = serde_json::to_string(&gmap).expect("gmap should serialize");
    let restored: GMap<SerializablePayload> =
        serde_json::from_str(&serialized).expect("gmap should deserialize");

    let mut vertex_payloads = restored
        .iter_vertices()
        .map(|(_, attr)| attr.data)
        .collect::<Vec<_>>();
    vertex_payloads.sort_unstable();
    assert_eq!(vertex_payloads, vec![10, 20]);
    assert_eq!(
        restored
            .iter_edges()
            .next()
            .expect("edge should round-trip")
            .1
            .data,
        30
    );
}
