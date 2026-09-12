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
use ngk::exchange::step::{StepWriteOptions, map_to_exchange, solid_to_exchange, step_to_string};
use ngk::geometry::{Plane, Point3};
use ngk::modeling::faces;
use ngk::modeling::solids;
use ngk::topology::StandardPayload;
use ngk::topology::shape::{Shape, SolidTag};

use ngk::topology::validation::{validate_all_solid_manifolds, validate_all_solid_orientations};

use crate::hollow::hollow_sphere;

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

/// Exports a cylinder of radius 5 and height 10.
fn cylinder() -> Shape<SolidTag, StandardPayload> {
    solids::cylinder(5.0, 10.0).expect("a cylinder should build")
}

#[test]
fn a_cylinder_exports_as_two_caps_and_a_wall() {
    // NGK stores the wall as one ring face with two wrapping loops and no
    // seam. STEP has no such face, so the wall has to arrive with its domain
    // cut open — as one `ADVANCED_FACE` all the same, which is what says the
    // cut was synthesized rather than the wall split in two.
    let exchange = exported(&cylinder());

    assert_eq!(count(&exchange, "ADVANCED_FACE"), 3);
    assert_eq!(count(&exchange, "CLOSED_SHELL"), 1);
    assert_eq!(count(&exchange, "PLANE"), 2);
    assert_eq!(count(&exchange, "CYLINDRICAL_SURFACE"), 1);
}

#[test]
fn a_cylinders_rims_are_closed_edges_naming_one_corner_twice() {
    // A full circle is one edge with one corner, and `EDGE_CURVE` spells that
    // by naming the same `VERTEX_POINT` at both ends. Splitting it into two
    // half-circles instead would be a different shape's worth of topology.
    let exchange = exported(&cylinder());

    assert_eq!(count(&exchange, "VERTEX_POINT"), 2);
    assert_eq!(count(&exchange, "CIRCLE"), 2);

    let closed = exchange
        .instances_of("EDGE_CURVE")
        .filter(|edge| {
            let params = &record(edge).params;
            params[1].as_reference() == params[2].as_reference()
        })
        .count();
    assert_eq!(closed, 2, "both rims should close on one corner");
}

#[test]
fn a_cylinders_seam_is_one_edge_the_wall_walks_both_ways() {
    // The cut is reached from either side of the unwrapped domain, and both
    // sides are the same edge of the shell. Writing two would leave the wall
    // unsewn along its own seam, which no volume check downstream would catch
    // — the shape would still measure right.
    let exchange = exported(&cylinder());

    let mut uses: HashMap<EntityId, Vec<&str>> = HashMap::new();
    for oriented in exchange.instances_of("ORIENTED_EDGE") {
        let params = &record(oriented).params;
        let edge = params[3].as_reference().expect("an edge curve reference");
        let sense = params[4].as_enum().expect("an orientation flag");
        uses.entry(edge).or_default().push(sense);
    }

    assert_eq!(uses.len(), 3, "two rims and one seam");
    for (edge, mut flags) in uses {
        flags.sort_unstable();
        assert_eq!(flags, ["F", "T"], "{edge} is walked the same way twice");
    }

    // The seam runs along the axis, so it is the one edge carrying a line.
    assert_eq!(count(&exchange, "LINE"), 1);
}

#[test]
fn a_cylinders_wall_is_bounded_by_four_oriented_edges() {
    // Bottom rim, seam up, top rim, seam down: the rectangle the unwrapped
    // domain closes, which is exactly what a stored seam used to spell out.
    let exchange = exported(&cylinder());

    let walls: Vec<_> = exchange
        .instances_of("EDGE_LOOP")
        .filter(|loop_| {
            record(loop_).params[1]
                .as_list()
                .is_some_and(|edges| edges.len() == 4)
        })
        .collect();
    assert_eq!(walls.len(), 1, "only the wall needs a cut");
}

#[test]
fn a_sphere_is_cut_open_into_two_meridians_between_its_poles() {
    // A sphere is one boundaryless face: zero loops, edges and vertices, so
    // every piece of the bound it is written with is synthesized. Cutting the
    // domain rectangle open leaves the two polar sides collapsed and the two
    // meridian sides standing, which is one edge walked both ways between the
    // two poles.
    let sphere = solids::sphere(5.0).expect("a sphere should build");
    let exchange = exported(&sphere);

    assert_eq!(count(&exchange, "ADVANCED_FACE"), 1);
    assert_eq!(count(&exchange, "SPHERICAL_SURFACE"), 1);
    assert_eq!(count(&exchange, "EDGE_LOOP"), 1);
    assert_eq!(count(&exchange, "EDGE_CURVE"), 1, "the cut is one edge");
    assert_eq!(count(&exchange, "ORIENTED_EDGE"), 2, "walked both ways");
    assert_eq!(count(&exchange, "VERTEX_POINT"), 2, "a pole at each end");
}

#[test]
fn a_torus_is_cut_open_twice_and_meets_itself_at_one_corner() {
    // A torus closes in both parameters, so the rectangle keeps all four of
    // its sides: two edges, each walked both ways. All four corners are the
    // same point of the surface, and writing them as four vertices would leave
    // a reader unable to sew the shell back up.
    let torus = solids::torus(3.0, 1.0).expect("a torus should build");
    let exchange = exported(&torus);

    assert_eq!(count(&exchange, "ADVANCED_FACE"), 1);
    assert_eq!(count(&exchange, "TOROIDAL_SURFACE"), 1);
    assert_eq!(count(&exchange, "EDGE_LOOP"), 1);
    assert_eq!(count(&exchange, "EDGE_CURVE"), 2, "one cut per parameter");
    assert_eq!(count(&exchange, "ORIENTED_EDGE"), 4);
    assert_eq!(count(&exchange, "VERTEX_POINT"), 1, "the cuts meet at one");
}

#[test]
fn a_boundaryless_bound_walks_each_of_its_cuts_once_each_way() {
    // The same manifold invariant the block is checked against, asked of a
    // shell whose every edge was synthesized: a cut walked twice the same way
    // is a boundary that does not close.
    let torus = solids::torus(3.0, 1.0).expect("a torus should build");
    let exchange = exported(&torus);

    let mut walks: HashMap<EntityId, Vec<bool>> = HashMap::new();
    for oriented in exchange.instances_of("ORIENTED_EDGE") {
        let params = &record(oriented).params;
        let edge = params[3]
            .as_reference()
            .expect("an oriented edge should reference its edge curve");
        let forward = params[4] == Value::Enum("T".to_string());
        walks.entry(edge).or_default().push(forward);
    }

    assert_eq!(walks.len(), 2);
    for (edge, mut directions) in walks {
        directions.sort_unstable();
        assert_eq!(
            directions,
            vec![false, true],
            "{edge} is not walked both ways"
        );
    }
}

#[test]
fn a_hollow_solid_is_well_formed_before_it_is_written() {
    // The fixture is hand-built, so the round trip below would be comparing a
    // file against a model no builder vouches for. The inner shell has to face
    // into its own cavity for the validator to accept it, which is the same
    // statement the export then has to carry.
    let hollow = hollow_sphere(5.0, 2.0);
    validate_all_solid_manifolds(hollow.map()).expect("both shells should be closed");
    validate_all_solid_orientations(hollow.map()).expect("both shells should face outward");
}

#[test]
fn a_cavity_is_written_as_a_void_rather_than_a_second_solid() {
    // A void written as an outer shell of its own is a second solid sitting
    // inside the first, which is a different shape and one no validator
    // downstream distinguishes from this one.
    let exchange = exported(&hollow_sphere(5.0, 2.0));

    assert_eq!(count(&exchange, "MANIFOLD_SOLID_BREP"), 0);
    assert_eq!(count(&exchange, "BREP_WITH_VOIDS"), 1);
    assert_eq!(count(&exchange, "CLOSED_SHELL"), 2);
    assert_eq!(count(&exchange, "ORIENTED_CLOSED_SHELL"), 1);
    assert_eq!(count(&exchange, "ADVANCED_FACE"), 2);
}

#[test]
fn a_cavity_faces_into_itself_and_the_outer_shell_away_from_the_material() {
    // Both shells bound the material from outside it, which for a cavity means
    // its face points inward — the opposite of the outer shell's, on the same
    // kind of support. Writing both with the same flag is the mistake this
    // catches, and it is one that leaves a perfectly readable file describing
    // a solid ball with a ghost sphere in it.
    let exchange = exported(&hollow_sphere(5.0, 2.0));

    let senses: Vec<bool> = exchange
        .instances_of("ADVANCED_FACE")
        .map(|face| record(face).params[3] == Value::Enum("T".to_string()))
        .collect();
    assert_eq!(senses.len(), 2);
    assert_ne!(senses[0], senses[1], "both shells face the same way");

    // The void is named in its own direction, so the flag it carries states
    // that nothing about the shell is to be flipped on the way in.
    let void = exchange
        .instances_of("ORIENTED_CLOSED_SHELL")
        .next()
        .expect("a hollow solid has a void");
    assert_eq!(record(void).params[3], Value::Enum("T".to_string()));
}
