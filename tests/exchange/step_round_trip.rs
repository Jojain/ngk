//! Writing a solid and reading it back.
//!
//! No snapshot framework exists in this crate and adding one would be a
//! first, so a round trip is checked **by invariant, never by text diff** —
//! the precedent is `tests/topology/serialization.rs`. What the contract
//! promises is the point set of every curve and surface to within the
//! document tolerance, the topology exactly, and the analytic *type* only
//! where NGK has a matching representation; nothing about entity ids or the
//! order they were written in.
//!
//! Every shape goes through the same [`round_trip`] helper, so a new
//! primitive is one call rather than a new set of assertions to get wrong.

use std::collections::BTreeMap;

use nalgebra::Vector3;
use ngk::builders::solids::add_extruded_face;
use ngk::exchange::step::{
    ImportSkipReason, StepReadOptions, StepWriteOptions, read_step, step_to_string,
};
use ngk::geometry::{LINEAR_TOLERANCE, Plane, Point3, PointCoincidence, Surface};
use ngk::modeling::{faces, solids};
use ngk::topology::StandardPayload;
use ngk::topology::shape::{Shape, SolidTag};
use ngk::topology::validation::{
    validate_all_solid_manifolds, validate_all_solid_orientations, validate_gmap,
};

use crate::hollow::hollow_sphere;

type Solid = Shape<SolidTag, StandardPayload>;

/// Writes `shape`, reads it back, and asserts everything the contract promises.
///
/// The orientation validator does most of the work: it checks per-edge
/// winding agreement *and* the global signed-volume sign, which is exactly
/// the pair a broken sense mapping violates. The rest pins what it cannot
/// see — that the corners are in the same places and the supports are still
/// the same kinds of surface.
fn round_trip(shape: &Solid) -> Solid {
    let text = step_to_string(shape, &StepWriteOptions::named("ROUND_TRIP"))
        .expect("a planar solid should export");
    let import = read_step(&text, &StepReadOptions::default()).expect("our own file should import");

    assert_eq!(import.shapes.len(), 1, "one solid out, one solid back");
    assert!(
        import.report.is_clean(),
        "nothing should have been given up: {:?}",
        import.report.skipped,
    );
    // Stated separately from `is_clean` because it is the D5 checksum rather
    // than a count: a sense mismatch means the winding we rebuilt disagrees
    // with the flag we wrote, so one of the two directions is wrong.
    assert_eq!(
        import
            .report
            .matching(|reason| matches!(reason, ImportSkipReason::SenseMismatch))
            .count(),
        0,
        "the winding read back should agree with the sense written",
    );

    let returned = import.shapes.into_iter().next().expect("one solid");
    let map = returned.map();
    validate_gmap(map).expect("the sewn map should satisfy the GMap axioms");
    validate_all_solid_manifolds(map).expect("the shell should be closed");
    validate_all_solid_orientations(map).expect("every face should point outward");

    assert_cells_match(shape, &returned);
    assert_corners_match(shape, &returned);
    assert_surface_kinds_match(shape, &returned);
    returned
}

fn assert_cells_match(before: &Solid, after: &Solid) {
    let (first, second) = (before.solid(), after.solid());
    assert_eq!(first.faces().len(), second.faces().len(), "face count");
    assert_eq!(first.edges().len(), second.edges().len(), "edge count");
    assert_eq!(
        first.vertices().len(),
        second.vertices().len(),
        "vertex count"
    );
}

/// Asserts that every corner came back, at the same place.
///
/// By position rather than by key: keys are not preserved across a file and
/// are not meant to be, but a corner that moved — or one that two corners
/// were welded into — is a corrupted shape.
fn assert_corners_match(before: &Solid, after: &Solid) {
    let corners = |shape: &Solid| {
        shape
            .solid()
            .vertices()
            .iter()
            .map(|vertex| *vertex.point().expect("a corner carries a point"))
            .collect::<Vec<Point3>>()
    };
    let (first, second) = (corners(before), corners(after));

    for point in &first {
        assert!(
            second
                .iter()
                .any(|other| point.coincides(other, LINEAR_TOLERANCE)),
            "{point:?} did not come back",
        );
    }
}

/// Asserts no support was demoted to a kind NGK did not start with.
fn assert_surface_kinds_match(before: &Solid, after: &Solid) {
    let kinds = |shape: &Solid| {
        let mut counts: BTreeMap<&'static str, usize> = BTreeMap::new();
        for face in shape.solid().faces() {
            *counts.entry(surface_kind(face.surface())).or_default() += 1;
        }
        counts
    };
    assert_eq!(kinds(before), kinds(after), "surface kinds");
}

fn surface_kind(surface: &Surface) -> &'static str {
    match surface {
        Surface::Plane(_) => "Plane",
        Surface::Cylinder(_) => "Cylinder",
        Surface::Sphere(_) => "Sphere",
        Surface::Cone(_) => "Cone",
        Surface::Torus(_) => "Torus",
        Surface::Ruled(_) => "Ruled",
        Surface::Revolution(_) => "Revolution",
        Surface::Nurbs(_) => "Nurbs",
    }
}

#[test]
fn a_block_survives_a_round_trip() {
    let block = solids::block(10.0, 20.0, 30.0).expect("a block should build");
    round_trip(&block);
}

#[test]
fn a_block_survives_two_round_trips_unchanged() {
    // The drift test. One pass can hide a systematic error that cancels
    // between the writer and the reader; a second pass over the *imported*
    // shape asks the exporter to describe topology it did not build itself.
    let block = solids::block(10.0, 20.0, 30.0).expect("a block should build");
    let once = round_trip(&block);
    round_trip(&once);
}

#[test]
fn a_slab_with_a_hole_survives_a_round_trip() {
    // The inner-bound path: the two faces the hole passes through each carry
    // a second loop, which must come back as a hole and not as a second
    // outer boundary — a distinction no validator makes, since both wind
    // legally.
    let slab = holed_slab();
    let after = round_trip(&slab);

    let holed = after
        .solid()
        .faces()
        .iter()
        .filter(|face| face.loops().len() == 2)
        .count();
    assert_eq!(holed, 2, "both pierced faces should keep their hole");
}

#[test]
fn an_extruded_triangle_survives_a_round_trip() {
    // An odd face count and an odd edge count, so an off-by-one in the loop
    // walk that a quad's symmetry would hide has somewhere to show.
    let triangle = faces::polygon(&[
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(4.0, 0.0, 0.0),
        Point3::new(0.0, 3.0, 0.0),
    ])
    .expect("a triangle should build");
    let (mut map, face) = triangle.into_map();
    let solid = add_extruded_face(&mut map, face, Vector3::new(0.0, 0.0, 2.0))
        .expect("a triangle should extrude");

    round_trip(&Shape::new(map, solid));
}

#[test]
fn a_read_of_our_own_file_declares_the_same_uncertainty_we_wrote() {
    // The unit round trip, which no geometry assertion reaches: a file whose
    // declared tolerance is lost still imports perfectly and then stitches
    // the *next* file at the wrong scale.
    let block = solids::block(1.0, 1.0, 1.0).expect("a block should build");
    let options = StepWriteOptions {
        uncertainty: 1.0e-5,
        ..StepWriteOptions::named("TOLERANT")
    };
    let text = step_to_string(&block, &options).expect("a planar solid should export");

    let exchange = ngk::exchange::step::part21::parse_exchange(&text).expect("it should parse");
    let units = ngk::exchange::step::schema::units::read_units(&exchange).expect("units read");

    assert_eq!(units.length, 1.0);
    assert_eq!(units.angle, 1.0);
    assert_eq!(units.uncertainty, 1.0e-5);
}

/// A 4 × 3 × 3 slab with a 1 × 1 hole through it, all faces planar.
fn holed_slab() -> Solid {
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
    Shape::new(map, solid)
}

#[test]
fn a_cylinder_survives_a_round_trip() {
    // The seam case, and the strongest single statement in this file: the
    // wall goes out with a synthesized cut in it and has to come back as one
    // ring face with no seam edge, which only happens if the cut was written
    // where the domain said, read back as a doubly-walked edge, and healed
    // away again. A cell count that matches is what says all three happened.
    round_trip(&solids::cylinder(5.0, 10.0).expect("a cylinder should build"));
}

#[test]
fn a_sphere_survives_a_round_trip() {
    // The boundaryless case: a sphere has nothing in the map to write at all
    // — no loop, no edge, no vertex — so the file is entirely synthesized, and
    // coming back as one face with none of them again says the cut was read
    // as the cut it was and healed away rather than kept.
    round_trip(&solids::sphere(5.0).expect("a sphere should build"));
}

#[test]
fn a_torus_survives_a_round_trip() {
    // Two cuts on one face, which is what makes a torus different from a
    // sphere rather than merely rounder: the file walks two edges twice each
    // around one vertex, and both removals have to compose on the way back in
    // for the face to come back boundaryless.
    round_trip(&solids::torus(3.0, 1.0).expect("a torus should build"));
}

#[test]
fn a_hollow_sphere_keeps_its_cavity_as_a_void() {
    // The `BREP_WITH_VOIDS` path. A cavity that comes back as a second outer
    // shell is a different shape — a ball with a ghost sphere inside it —
    // and every structural validator accepts both, so the void has to be
    // asserted by name.
    let after = round_trip(&hollow_sphere(5.0, 2.0));

    let solid = after.solid();
    let voids = solid.inner_shells().expect("the cavity should come back");
    assert_eq!(voids.len(), 1);
    assert_eq!(solid.faces().len(), 2, "one shell each, one face each");
}

#[test]
fn a_boolean_result_survives_a_round_trip() {
    // The shape that needed the spline writer. Cutting one block with another
    // leaves every support planar and yet most of the edges free-form, because
    // an imprint's section is fitted rather than recognized — so this is a
    // solid whose geometry is NGK's own NURBS rather than a file's.
    let block = solids::block(10.0, 10.0, 10.0).expect("a block should build");
    let tool = solids::block(4.0, 4.0, 30.0).expect("a tool should build");
    let cut = solids::cut(block, tool).expect("a through cut should build");

    let splines = cut
        .solid()
        .edges()
        .iter()
        .filter(|edge| matches!(edge.curve(), Some(ngk::geometry::Curve::Nurbs(_))))
        .count();
    assert!(splines > 0, "a cut leaves spline edges behind");

    round_trip(&cut);
}

#[test]
fn a_spline_solid_read_from_a_file_survives_being_written_back() {
    // The other direction of the same claim, and the stronger one: this is
    // OpenCascade's geometry, in the rational spelling, on a periodic patch
    // with unclamped knots. Getting it back out means the knot run-length
    // coding, the weight split and the control-net transposition all inverted.
    let text = include_str!("../fixtures/step/swept_circle.step");
    let import = read_step(text, &StepReadOptions::default()).expect("a foreign spline solid");
    let shape = import.shapes.into_iter().next().expect("one solid");

    round_trip(&shape);
}
