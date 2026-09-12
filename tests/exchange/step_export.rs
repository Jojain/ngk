//! Writing a solid as STEP.
//!
//! These assert on the structure NGK emits. What they cannot assert is that
//! another kernel accepts it — that check lives in
//! `../fixtures/step/validate_ngk_export.py`, which reads these same shapes
//! back through OpenCascade and compares volumes. Both are needed: a file can
//! satisfy every invariant below and still be rejected by a real reader.

use std::collections::HashMap;

use nalgebra::Vector3;
use ngk::builders::solids::add_extruded_face;
use ngk::exchange::step::part21::{EntityId, Instance, StepExchange, Value, parse_exchange};
use ngk::exchange::step::{
    StepError, StepWriteOptions, TopologyError, map_to_exchange, solid_to_exchange, step_to_string,
};
use ngk::geometry::{Plane, Point3};
use ngk::modeling::faces;
use ngk::modeling::solids;
use ngk::topology::StandardPayload;
use ngk::topology::shape::{Shape, SolidTag};

/// Exports a 10 × 20 × 30 block.
fn block() -> Shape<SolidTag, StandardPayload> {
    solids::block(10.0, 20.0, 30.0).expect("a block should build")
}

fn exported(shape: &Shape<SolidTag, StandardPayload>) -> StepExchange {
    solid_to_exchange(shape, &StepWriteOptions::named("TEST"))
        .expect("a planar solid should export")
}

/// Counts instances carrying `keyword`.
fn count(exchange: &StepExchange, keyword: &str) -> usize {
    exchange.instances_of(keyword).count()
}

/// Returns the sole record of an instance, which the exporter always writes
/// simple except for the unit block.
fn record(instance: &Instance) -> &ngk::exchange::step::part21::Record {
    instance
        .simple()
        .expect("exported instance should be simple")
}

#[test]
fn a_block_exports_as_one_closed_shell_of_six_planar_faces() {
    let exchange = exported(&block());

    assert_eq!(count(&exchange, "MANIFOLD_SOLID_BREP"), 1);
    assert_eq!(count(&exchange, "CLOSED_SHELL"), 1);
    assert_eq!(count(&exchange, "ADVANCED_FACE"), 6);
    assert_eq!(count(&exchange, "PLANE"), 6);
    assert_eq!(count(&exchange, "EDGE_LOOP"), 6);
}

#[test]
fn a_block_exports_twelve_edges_and_eight_corners() {
    let exchange = exported(&block());

    assert_eq!(count(&exchange, "EDGE_CURVE"), 12);
    assert_eq!(count(&exchange, "LINE"), 12);
    assert_eq!(count(&exchange, "VERTEX_POINT"), 8);
    // Four oriented uses per face, six faces.
    assert_eq!(count(&exchange, "ORIENTED_EDGE"), 24);
}

#[test]
fn every_edge_curve_is_used_by_exactly_two_faces() {
    // The manifold invariant, and the single strongest statement that the
    // shell is sewn rather than a heap of loose faces: an edge used once
    // leaves the shell open, and one used three times is not a 3-GMap at all.
    let exchange = exported(&block());

    let mut uses: HashMap<EntityId, usize> = HashMap::new();
    for oriented in exchange.instances_of("ORIENTED_EDGE") {
        let edge = record(oriented).params[3]
            .as_reference()
            .expect("an oriented edge should reference its edge curve");
        *uses.entry(edge).or_default() += 1;
    }

    assert_eq!(uses.len(), 12, "every edge curve should be reached");
    for (edge, count) in uses {
        assert_eq!(count, 2, "{edge} is used {count} times, not twice");
    }
}

#[test]
fn an_edge_is_walked_once_each_way_around_the_shell() {
    // The other half of the sewing claim: the two faces meeting at an edge
    // traverse it in opposite directions. Both agreeing would fold the shell.
    let exchange = exported(&block());

    let mut senses: HashMap<EntityId, Vec<&str>> = HashMap::new();
    for oriented in exchange.instances_of("ORIENTED_EDGE") {
        let params = &record(oriented).params;
        let edge = params[3].as_reference().expect("an edge curve reference");
        let sense = params[4].as_enum().expect("an orientation flag");
        senses.entry(edge).or_default().push(sense);
    }

    for (edge, mut flags) in senses {
        flags.sort_unstable();
        assert_eq!(flags, ["F", "T"], "{edge} is walked the same way twice");
    }
}

#[test]
fn a_position_is_written_once_however_many_entities_share_it() {
    // A box names eight corners; the planes and lines anchored at those same
    // corners reuse them rather than restating the coordinates.
    let exchange = exported(&block());

    assert_eq!(count(&exchange, "CARTESIAN_POINT"), 8);
}

#[test]
fn a_corner_is_never_merged_with_another_at_the_same_position() {
    // The counterpart to sharing positions: a VERTEX_POINT is a corner of the
    // solid, not a position, so each of the eight is its own instance even
    // though each merely points at a shared CARTESIAN_POINT.
    let exchange = exported(&block());

    let mut corners: Vec<EntityId> = exchange
        .instances_of("VERTEX_POINT")
        .map(|instance| instance.id)
        .collect();
    corners.sort_unstable();
    corners.dedup();
    assert_eq!(corners.len(), 8);
}

#[test]
fn an_exported_file_references_nothing_it_does_not_define() {
    let exchange = exported(&block());

    assert_eq!(exchange.dangling_references(), Vec::new());
}

#[test]
fn an_exported_file_reads_back_as_the_same_structure() {
    let block = block();
    let text = step_to_string(&block, &StepWriteOptions::named("TEST"))
        .expect("a planar solid should export");
    let reparsed = parse_exchange(&text).expect("exported text should parse");

    assert_eq!(
        reparsed.instances().len(),
        exported(&block).instances().len(),
    );
    assert_eq!(count(&reparsed, "ADVANCED_FACE"), 6);
    assert!(reparsed.dangling_references().is_empty());
}

#[test]
fn every_coordinate_is_written_as_a_real() {
    // A whole-number coordinate written without its point reads back as an
    // *integer*, which changes the attribute's type and is exactly the kind of
    // corruption that produces a file looking entirely correct.
    let text = step_to_string(&block(), &StepWriteOptions::named("TEST"))
        .expect("a planar solid should export");
    let exchange = parse_exchange(&text).expect("exported text should parse");

    for keyword in ["CARTESIAN_POINT", "DIRECTION"] {
        for instance in exchange.instances_of(keyword) {
            let coordinates = record(instance).params[1]
                .as_list()
                .expect("coordinates should be a list");
            for coordinate in coordinates {
                assert!(
                    matches!(coordinate, Value::Real(_)),
                    "{keyword} {} carries {coordinate:?}, not a real",
                    instance.id,
                );
            }
        }
    }
}

#[test]
fn exporting_the_same_model_twice_produces_the_same_text() {
    // Instance names are positional, so anything that reorders emission —
    // iterating a hash map, say — silently renames every entity in the file
    // and makes two exports of one model impossible to compare.
    let block = block();
    let options = StepWriteOptions::named("TEST");

    let first = step_to_string(&block, &options).expect("a planar solid should export");
    let second = step_to_string(&block, &options).expect("a planar solid should export");

    assert_eq!(first, second);
}

#[test]
fn a_hole_is_written_as_an_inner_bound() {
    // A hole exported as an outer bound, or dropped, leaves a reader with a
    // solid of the wrong volume rather than with an error.
    let exchange = holed_solid();

    assert_eq!(count(&exchange, "ADVANCED_FACE"), 10);
    // Ten faces each carry an outer bound; the two the hole passes through
    // carry an inner bound as well.
    assert_eq!(count(&exchange, "FACE_OUTER_BOUND"), 10);
    assert_eq!(count(&exchange, "FACE_BOUND"), 2);
}

/// A 4 × 3 × 3 slab with a 1 × 1 hole through it, all faces planar.
fn holed_solid() -> StepExchange {
    let outer = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(4.0, 0.0, 0.0),
        Point3::new(4.0, 3.0, 0.0),
        Point3::new(0.0, 3.0, 0.0),
    ];
    let hole = [
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(1.0, 2.0, 0.0),
        Point3::new(2.0, 2.0, 0.0),
        Point3::new(2.0, 1.0, 0.0),
    ];
    let profile = faces::polygon_with_holes(Plane::xy(), &outer, &[&hole])
        .expect("a holed face should build");
    let (mut map, face) = profile.into_map();
    let solid =
        add_extruded_face(&mut map, face, Vector3::new(0.0, 0.0, 3.0)).expect("it should extrude");

    map_to_exchange(&map, &[solid], &StepWriteOptions::named("HOLED"))
        .expect("a planar solid should export")
}

#[test]
fn the_document_declares_millimetres_and_its_uncertainty() {
    let text = step_to_string(&block(), &StepWriteOptions::named("TEST"))
        .expect("a planar solid should export");

    assert!(text.contains("SI_UNIT(.MILLI.,.METRE.)"), "got {text}");
    assert!(text.contains("LENGTH_MEASURE(1.E-7)"), "got {text}");
    assert!(
        text.contains("GEOMETRIC_REPRESENTATION_CONTEXT(3)"),
        "got {text}",
    );
}

#[test]
fn the_product_structure_names_the_model() {
    let text = step_to_string(&block(), &StepWriteOptions::named("WIDGET"))
        .expect("a planar solid should export");

    assert!(text.contains("PRODUCT('WIDGET','WIDGET'"), "got {text}");
    assert!(
        text.contains("FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'))"),
        "got {text}",
    );
    assert!(
        text.contains("SHAPE_DEFINITION_REPRESENTATION"),
        "got {text}"
    );
}

#[test]
fn a_cylinder_is_refused_by_name_rather_than_approximated() {
    // Its wall is one ring face whose circular edge closes on itself, which
    // STEP can only carry once a seam has been synthesized for it.
    let cylinder = solids::cylinder(5.0, 10.0).expect("a cylinder should build");
    let error = step_to_string(&cylinder, &StepWriteOptions::default())
        .expect_err("a periodic support should be refused");

    assert!(
        matches!(error, StepError::Topology(TopologyError::ClosedEdge { .. })),
        "got {error}",
    );
    assert!(error.to_string().contains("seam"), "got {error}");
}

#[test]
fn a_sphere_is_refused_by_name_rather_than_approximated() {
    // A sphere is one boundaryless face: zero loops, edges and vertices, so
    // there is not even a boundary to walk until stage 5 synthesizes one.
    let sphere = solids::sphere(5.0).expect("a sphere should build");
    let error = step_to_string(&sphere, &StepWriteOptions::default())
        .expect_err("a boundaryless face should be refused");

    assert!(
        matches!(
            error,
            StepError::Topology(TopologyError::BoundarylessFace { .. })
        ),
        "got {error}",
    );
}
