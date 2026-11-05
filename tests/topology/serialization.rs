use ngk::geometry::{Curve, Frame, Point3};
use ngk::model::Model;
use ngk::modeling::solids::{self, block};
use ngk::topology::StandardPayload;
use ngk::topology::attributes::{EdgeAttr, VertexAttr};
use ngk::topology::edit::{ModelEditError, PreservePayload};
use ngk::topology::gmap::Dim;
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

    type Policy = PreservePayload;
}

#[test]
fn a_model_round_trips_through_json_without_losing_state() {
    let block = block(1.0, 2.0, 3.0).expect("block should build");
    let serialized = serde_json::to_string(block.model()).expect("a model should serialize");

    let restored: Model<StandardPayload> =
        serde_json::from_str(&serialized).expect("a model should deserialize");
    let original_value: serde_json::Value =
        serde_json::from_str(&serialized).expect("a serialized model should be valid JSON");
    let restored_value =
        serde_json::to_value(&restored).expect("a restored model should serialize");

    assert_eq!(restored_value, original_value);
    assert_eq!(restored.dart_count(), block.model().dart_count());
    assert_eq!(
        restored.iter_vertices().count(),
        block.model().iter_vertices().count()
    );
    assert_eq!(
        restored.iter_edges().count(),
        block.model().iter_edges().count()
    );
    assert_eq!(
        restored.iter_profiles().count(),
        block.model().iter_profiles().count()
    );
    assert_eq!(
        restored.iter_faces().count(),
        block.model().iter_faces().count()
    );
    assert_eq!(
        restored.iter_sheets().count(),
        block.model().iter_sheets().count()
    );
    assert_eq!(
        restored.iter_solids().count(),
        block.model().iter_solids().count()
    );
}

#[test]
fn the_same_model_built_twice_serializes_to_the_same_string() {
    // Each build fills its own hash maps, whose iteration order differs from
    // one map to the next; the written string must not depend on it.
    let serialize = |model: &Model<StandardPayload>| {
        serde_json::to_string(model).expect("a model should serialize")
    };
    let build_block = || block(30.0, 20.0, 10.0).expect("block should build");
    let build_cut = || {
        let tool = solids::cylinder_at(Frame::at(Point3::new(15.0, 10.0, -1.0)), 4.0, 12.0)
            .expect("cylinder should build");
        solids::cut(build_block(), [&tool]).expect("the cylinder should cut the block")
    };

    let blocks: Vec<_> = (0..5).map(|_| serialize(build_block().model())).collect();
    let cuts: Vec<_> = (0..5).map(|_| serialize(build_cut().model())).collect();

    assert!(blocks.windows(2).all(|pair| pair[0] == pair[1]));
    assert!(cuts.windows(2).all(|pair| pair[0] == pair[1]));
}

#[test]
fn a_model_round_trips_serializable_custom_payloads() {
    let mut model = Model::<SerializablePayload>::new();
    let (start, end, edge) = model
        .transaction(|edit| {
            edit.deferred(|raw| {
                let start_dart = raw.add_dart();
                let end_dart = raw.add_dart();
                raw.link(Dim::Zero, start_dart, end_dart)?;
                let start = raw.add_vertex(VertexAttr::new(start_dart, Point3::origin()));
                let end = raw.add_vertex(VertexAttr::new(end_dart, Point3::new(1.0, 0.0, 0.0)));
                let edge = raw.add_edge(EdgeAttr::new(
                    start_dart,
                    Curve::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0)),
                ));
                Ok::<_, ModelEditError>((start, end, edge))
            })
        })
        .expect("custom-payload edge should build");
    // A fresh creation's payload is the default policy's to decide, so it
    // defaults regardless of what the constructor above was given; setting it
    // afterwards is a payload-only mutation, which reaches no hook.
    model
        .transaction(|edit| {
            *edit.vertex_attr_mut_unchecked(start).data_mut() = 10;
            *edit.vertex_attr_mut_unchecked(end).data_mut() = 20;
            *edit.edge_attr_mut_unchecked(edge).data_mut() = 30;
            Ok::<_, ModelEditError>(())
        })
        .expect("naming the custom payloads should commit");

    let serialized = serde_json::to_string(&model).expect("a model should serialize");
    let restored: Model<SerializablePayload> =
        serde_json::from_str(&serialized).expect("a model should deserialize");

    let mut vertex_payloads = restored
        .iter_vertices()
        .map(|(_, attr)| *attr.data())
        .collect::<Vec<_>>();
    vertex_payloads.sort_unstable();
    assert_eq!(vertex_payloads, vec![10, 20]);
    assert_eq!(
        *restored
            .iter_edges()
            .next()
            .expect("edge should round-trip")
            .1
            .data(),
        30
    );
}
