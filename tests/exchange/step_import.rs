//! Reading STEP text into solids.
//!
//! The subject here is the *stitching*: STEP hands over loose faces that
//! name shared edges by `#N`, and what must come back is one sewn 3-gmap. So
//! these assert on the map — cell counts, then the three validators — rather
//! than on anything about the text, which is `part21_parse`'s business.
//!
//! `step_round_trip` covers the other half: that what we write comes back as
//! what we wrote. Both are needed, because a reader can be self-consistent
//! with a writer and still disagree with every other kernel — which is why
//! the box fixture below is one OpenCascade wrote, not one we did.

use std::collections::HashSet;

use ngk::exchange::step::{
    ImportSkipReason, StepError, StepReadOptions, read_step, schema::units::Units,
};
use ngk::geometry::{LINEAR_TOLERANCE, PointCoincidence};
use ngk::topology::validation::{
    validate_all_solid_manifolds, validate_all_solid_orientations, validate_gmap,
};

/// A 10 × 20 × 30 box written by OpenCascade through build123d.
///
/// Regenerate with `uv run python tests/exchange/foreign/generate/generate_box.py`.
const OCCT_BOX: &str = include_str!("foreign/files/box.step");

fn read(text: &str) -> ngk::exchange::step::StepImport {
    read_step(text, &StepReadOptions::default()).expect("a planar solid should import")
}

#[test]
fn a_foreign_box_imports_as_one_solid() {
    let import = read(OCCT_BOX);

    assert_eq!(import.shapes.len(), 1);
    assert!(
        import.report.is_clean(),
        "nothing should have been given up: {:?}",
        import.report.skipped
    );
}

#[test]
fn a_foreign_box_imports_with_a_box_worth_of_cells() {
    // The count that proves stitching happened: read without sewing, six
    // quads carry 24 corners and 24 edges rather than 8 and 12.
    let import = read(OCCT_BOX);
    let shape = &import.shapes[0];
    let solid = shape.solid();

    assert_eq!(solid.faces().len(), 6);
    assert_eq!(solid.edges().len(), 12);
    assert_eq!(solid.vertices().len(), 8);
}

#[test]
fn a_foreign_box_imports_as_a_valid_oriented_solid() {
    // `validate_all_solid_orientations` does most of the work of this file:
    // it checks per-edge winding agreement *and* the global signed-volume
    // sign, which is exactly the pair a mis-sewn or inverted face violates.
    let import = read(OCCT_BOX);
    let map = import.shapes[0].model();

    validate_gmap(map.topology()).expect("the sewn map should satisfy the gmap axioms");
    validate_all_solid_manifolds(map).expect("the shell should be closed");
    validate_all_solid_orientations(map).expect("every face should point outward");
}

#[test]
fn a_foreign_box_imports_at_its_own_corners() {
    // build123d centres a `Box`, so the corners are at ±half each extent.
    let import = read(OCCT_BOX);
    let shape = &import.shapes[0];

    let mut expected = HashSet::new();
    for x in [-5.0_f64, 5.0] {
        for y in [-10.0_f64, 10.0] {
            for z in [-15.0_f64, 15.0] {
                expected.insert((x.to_bits(), y.to_bits(), z.to_bits()));
            }
        }
    }

    for vertex in shape.solid().vertices() {
        let point = vertex.point().expect("an imported vertex carries a point");
        let found = expected.iter().any(|(x, y, z)| {
            let corner = ngk::geometry::Point3::new(
                f64::from_bits(*x),
                f64::from_bits(*y),
                f64::from_bits(*z),
            );
            point.coincides(&corner, LINEAR_TOLERANCE)
        });
        assert!(found, "{point:?} is not a corner of the box");
    }
}

#[test]
fn every_imported_face_carries_a_pcurve_per_boundary_dart() {
    // `FaceAttr` requires one, and STEP does not have to supply it — so the
    // importer rebuilds it by projection. A missing one does not fail
    // any validator; it surfaces much later as a face with no winding.
    let import = read(OCCT_BOX);
    let shape = &import.shapes[0];

    for face in shape.solid().faces() {
        for boundary in face.loops() {
            for edge in boundary.edges() {
                assert!(
                    face.pcurve(edge.dart()).is_some(),
                    "face {:?} has no pcurve at {:?}",
                    face.key(),
                    edge.dart()
                );
            }
        }
    }
}

#[test]
fn a_file_with_no_solids_imports_as_nothing() {
    // Not an error: a STEP file is a document, and one holding only product
    // structure is well formed and simply has no B-Rep in it.
    let text = "\
ISO-10303-21;
HEADER;
FILE_DESCRIPTION((''),'2;1');
FILE_NAME('','',(''),(''),'','','');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));
ENDSEC;
DATA;
#1 = APPLICATION_CONTEXT('core data for automotive mechanical design processes');
ENDSEC;
END-ISO-10303-21;
";
    let import = read(text);
    assert!(import.shapes.is_empty());
    assert!(import.report.is_clean());
}

#[test]
fn a_malformed_file_is_refused_with_a_line_number() {
    let text = "\
ISO-10303-21;
HEADER;
FILE_DESCRIPTION((''),'2;1');
ENDSEC;
DATA;
#1 = CARTESIAN_POINT('',(0.,0.,
ENDSEC;
END-ISO-10303-21;
";
    let error = read_step(text, &StepReadOptions::default()).expect_err("this is not parseable");
    assert!(
        matches!(error, StepError::Syntax(_)),
        "expected a syntax error, got {error}"
    );
    assert!(
        error.to_string().contains("line 6"),
        "the message should name the bad line: {error}"
    );
}

#[test]
fn a_dangling_reference_names_both_ends() {
    let text = box_with(&[("#33", "#999")]);
    let import = read_step(&text, &StepReadOptions::default()).expect("the read itself succeeds");

    // Best-effort: the face that could not be read is dropped and named, and
    // the rest of the file still arrives.
    let detail = import
        .report
        .skipped
        .iter()
        .find_map(|skip| match &skip.reason {
            ImportSkipReason::FaceNotConstructible { detail } => Some(detail.clone()),
            _ => None,
        })
        .expect("the broken face should be reported");
    assert!(
        detail.contains("#999"),
        "the message should name the missing entity: {detail}"
    );
}

#[test]
fn strict_mode_refuses_what_lenient_mode_reports() {
    let text = box_with(&[("#33", "#999")]);
    let error = read_step(&text, &StepReadOptions::strict()).expect_err("strict should refuse");
    assert!(
        matches!(error, StepError::Schema(_)),
        "expected a schema error, got {error}"
    );
}

#[test]
fn a_file_in_inches_arrives_in_millimetres() {
    // The unit block is the one place a file can lie about scale without any
    // geometry looking wrong, so the conversion is asserted on a corner
    // rather than on the declared scale.
    let text = OCCT_BOX
        .replace(
            "#346 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );",
            "#346 = ( CONVERSION_BASED_UNIT('INCH',#901) LENGTH_UNIT() NAMED_UNIT(#902) );\n\
             #900 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );\n\
             #901 = MEASURE_WITH_UNIT(LENGTH_MEASURE(25.4),#900);\n\
             #902 = DIMENSIONAL_EXPONENTS(1.,0.,0.,0.,0.,0.,0.);",
        )
        .replace("LENGTH_MEASURE(1.E-07),#346", "LENGTH_MEASURE(1.E-07),#900");

    let import = read(&text);
    let shape = &import.shapes[0];
    let furthest = shape
        .solid()
        .vertices()
        .iter()
        .filter_map(|vertex| vertex.point().map(|point| point.x.abs()))
        .fold(0.0_f64, f64::max);

    assert!(
        (furthest - 5.0 * 25.4).abs() < 1.0e-9,
        "an inch box should arrive 25.4 times larger, got {furthest}"
    );
}

#[test]
fn a_file_with_no_unit_block_is_read_as_millimetres() {
    // Leniency by default: product structure is where vendor files diverge
    // most, and a missing context should not cost the geometry.
    let units = Units::default();
    assert_eq!(units.to_mm(1.0), 1.0);
    assert_eq!(units.to_radians(1.0), 1.0);
}

/// Returns the box fixture with some entity references rewritten.
fn box_with(edits: &[(&str, &str)]) -> String {
    let mut text = OCCT_BOX.to_string();
    for (from, to) in edits {
        // Only the definition site is rewritten — `#33 =` stays put and the
        // *use* of it inside the plane becomes the dangling one.
        text = text.replace(&format!("PLANE('',{from})"), &format!("PLANE('',{to})"));
    }
    text
}

/// A 4 × 3 × 3 slab bored through, written by OpenCascade through build123d.
///
/// Regenerate with `uv run python tests/exchange/foreign/generate/generate_holed_slab.py`.
const OCCT_HOLED_SLAB: &str = include_str!("foreign/files/holed_slab.step");

#[test]
fn a_foreign_face_with_two_bounds_and_no_outer_one_picks_the_larger() {
    // `FACE_OUTER_BOUND` is optional and OpenCascade writes none at all, so a
    // pierced face arrives as two indistinguishable `FACE_BOUND`s. Taking the
    // wrong one as the outer boundary inverts nothing a validator checks —
    // both windings are legal — and yields a face that is the hole.
    let import = read(OCCT_HOLED_SLAB);
    assert_eq!(import.shapes.len(), 1);

    let shape = &import.shapes[0];
    let solid = shape.solid();
    assert_eq!(solid.faces().len(), 10);
    assert_eq!(solid.edges().len(), 24);
    assert_eq!(solid.vertices().len(), 16);

    let pierced = solid
        .faces()
        .iter()
        .filter(|face| face.loops().len() == 2)
        .count();
    assert_eq!(pierced, 2, "both pierced faces should keep their hole");
}

#[test]
fn a_foreign_slab_with_a_hole_is_a_valid_oriented_solid() {
    let import = read(OCCT_HOLED_SLAB);
    let map = import.shapes[0].model();

    validate_gmap(map.topology()).expect("the sewn map should satisfy the gmap axioms");
    validate_all_solid_manifolds(map).expect("the shell should be closed");
    validate_all_solid_orientations(map).expect("every face should point outward");
}

#[test]
fn guessing_an_outer_bound_is_reported_rather_than_silent() {
    // The guess is only as good as the winding, so it is said out loud —
    // once per face that needed it, and never for a face with one bound.
    let import = read(OCCT_HOLED_SLAB);

    let guesses: Vec<_> = import
        .report
        .matching(|reason| matches!(reason, ImportSkipReason::GuessedOuterBound { .. }))
        .collect();
    assert_eq!(guesses.len(), 2, "got {:?}", import.report.skipped);
    for guess in guesses {
        assert!(matches!(
            guess.reason,
            ImportSkipReason::GuessedOuterBound { bounds: 2 }
        ));
        assert!(guess.entity.is_some(), "a guess should name its face");
    }
}

/// A radius-5, height-10 cylinder written by OpenCascade through build123d.
///
/// Regenerate with `uv run python tests/exchange/foreign/generate/generate_cylinder.py`.
const OCCT_CYLINDER: &str = include_str!("foreign/files/cylinder.step");

#[test]
fn a_foreign_cylinder_arrives_as_a_ring_face_between_two_caps() {
    // The whole seam story read backwards. OpenCascade writes the wall with
    // its domain cut open: a rim, a seam up, the other rim, the seam back
    // down, over a `SEAM_CURVE` the same loop walks twice. NGK stores no seam,
    // so the counts here are what say the cut was understood and then healed
    // away rather than left in the map as a spurious edge.
    //
    // The corners go the same way. The file gives each rim a `VERTEX_POINT`
    // because `EDGE_CURVE` names two ends and a circle has none to name; once
    // the seam that also reached it is gone, nothing meets there and the point
    // is classified inside the rim. An imported cylinder is then the same
    // shape as a built one, down to the counts.
    let import = read(OCCT_CYLINDER);
    let shape = &import.shapes[0];
    let solid = shape.solid();

    assert_eq!(solid.faces().len(), 3);
    assert_eq!(solid.edges().len(), 2, "the seam should be gone");
    assert_eq!(
        solid.vertices().len(),
        0,
        "the rims close on themselves, so there is no corner to keep"
    );
}

#[test]
fn a_foreign_cylinder_keeps_its_analytic_supports() {
    // The file names a `CYLINDRICAL_SURFACE` and two `PLANE`s, and each has a
    // matching NGK type — so nothing here should arrive as NURBS.
    let import = read(OCCT_CYLINDER);
    let mut cylinders = 0;
    let mut planes = 0;
    for face in import.shapes[0].solid().faces() {
        match face.surface() {
            ngk::geometry::Surface::Cylinder(_) => cylinders += 1,
            ngk::geometry::Surface::Plane(_) => planes += 1,
            other => panic!("unexpected support {other:?}"),
        }
    }
    assert_eq!((cylinders, planes), (1, 2));
}

#[test]
fn a_foreign_cylinder_is_a_valid_oriented_solid() {
    let import = read(OCCT_CYLINDER);
    let map = import.shapes[0].model();

    validate_gmap(map.topology()).expect("the sewn map should satisfy the gmap axioms");
    validate_all_solid_manifolds(map).expect("the shell should be closed");
    validate_all_solid_orientations(map).expect("every face should point outward");
}

/// A truncated cone written by OpenCascade through build123d.
///
/// Regenerate with `uv run python tests/exchange/foreign/generate/generate_frustum.py`.
const OCCT_FRUSTUM: &str = include_str!("foreign/files/frustum.step");

#[test]
fn a_foreign_cone_arrives_as_a_cone() {
    // The cone is the one analytic support NGK parameterizes differently from
    // STEP — `v` along the generatrix rather than along the axis — so reading
    // one is where that difference either is handled or is quietly wrong. The
    // shape being right is not enough to show it: the surface *type* has to
    // survive too, or the difference was dodged by demoting to NURBS.
    let import = read(OCCT_FRUSTUM);
    let solid = import.shapes[0].solid();

    assert_eq!(solid.faces().len(), 3);
    assert_eq!(solid.edges().len(), 2, "the seam should be gone");
    assert!(
        solid
            .faces()
            .iter()
            .any(|face| matches!(face.surface(), ngk::geometry::Surface::Cone(_))),
        "the wall should still be a cone",
    );
}

#[test]
fn a_foreign_cones_sense_agrees_with_the_winding_rebuilt_for_it() {
    // The D5 checksum, on the surface most likely to fail it. A wrong `v`
    // scale, or parameter curves left folded into one period rather than
    // placed on one branch, inverts the winding — and the file's own flag is
    // what says so, at the face that caused it.
    let import = read(OCCT_FRUSTUM);
    assert_eq!(
        import
            .report
            .matching(|reason| matches!(reason, ImportSkipReason::SenseMismatch))
            .count(),
        0,
        "got {:?}",
        import.report.skipped,
    );

    let map = import.shapes[0].model();
    validate_gmap(map.topology()).expect("the sewn map should satisfy the gmap axioms");
    validate_all_solid_manifolds(map).expect("the shell should be closed");
    validate_all_solid_orientations(map).expect("every face should point outward");
}

/// A torus of major radius 3 and minor radius 1, written by OpenCascade.
///
/// Regenerate with `uv run python tests/exchange/foreign/generate/generate_torus.py`.
const OCCT_TORUS: &str = include_str!("foreign/files/torus.step");

#[test]
fn a_foreign_torus_arrives_as_one_face_with_no_boundary() {
    // Two cuts rather than one, which is what makes a torus the hard case:
    // OpenCascade writes the face as a rectangle whose four sides are two
    // `SEAM_CURVE`s walked twice each, meeting at a single vertex. Taking one
    // cut off leaves a face that is still seamed, so a count of zero here is
    // the statement that the removals composed.
    let import = read(OCCT_TORUS);
    let shape = &import.shapes[0];
    let solid = shape.solid();

    assert!(
        import.report.is_clean(),
        "nothing should have been given up: {:?}",
        import.report.skipped
    );
    assert_eq!(solid.faces().len(), 1);
    assert_eq!(solid.edges().len(), 0, "both cuts should be gone");
    assert_eq!(solid.vertices().len(), 0);
    assert!(matches!(
        solid.faces()[0].surface(),
        ngk::geometry::Surface::Torus(_)
    ));
}

#[test]
fn a_foreign_torus_is_a_valid_oriented_solid() {
    let import = read(OCCT_TORUS);
    let map = import.shapes[0].model();

    validate_gmap(map.topology()).expect("the sewn map should satisfy the gmap axioms");
    validate_all_solid_manifolds(map).expect("the shell should be closed");
    validate_all_solid_orientations(map).expect("the torus should face outward");
}

/// A sphere of radius 5, written by OpenCascade.
///
/// Regenerate with `uv run python tests/exchange/foreign/generate/generate_sphere.py`.
const OCCT_SPHERE: &str = include_str!("foreign/files/sphere.step");

#[test]
fn a_foreign_sphere_arrives_as_one_face_with_no_boundary() {
    // OpenCascade writes a whole sphere with no cut in it at all: the face's
    // one `FACE_BOUND` holds a `VERTEX_LOOP`, which names a point on the face
    // rather than a loop around it. That is a boundary only in the schema's
    // sense, and taking it as one would put a vertex and a loop into the map
    // where the shape has neither.
    let import = read(OCCT_SPHERE);
    let shape = &import.shapes[0];
    let solid = shape.solid();

    assert!(
        import.report.is_clean(),
        "nothing should have been given up: {:?}",
        import.report.skipped
    );
    assert_eq!(solid.faces().len(), 1);
    assert_eq!(solid.edges().len(), 0);
    assert_eq!(solid.vertices().len(), 0);
    assert!(matches!(
        solid.faces()[0].surface(),
        ngk::geometry::Surface::Sphere(_)
    ));
}

#[test]
fn a_foreign_sphere_is_a_valid_oriented_solid() {
    // The one thing a boundaryless face cannot state for itself: with no
    // winding to read, which way it points comes from the file's `same_sense`
    // and lands on the shell's root. Getting that wrong yields a sphere of
    // negative volume that every other check accepts.
    let import = read(OCCT_SPHERE);
    let map = import.shapes[0].model();

    validate_gmap(map.topology()).expect("the sewn map should satisfy the gmap axioms");
    validate_all_solid_manifolds(map).expect("the shell should be closed");
    validate_all_solid_orientations(map).expect("the sphere should face outward");
}

/// A rectangle lofted to a circle, from OpenCascade.
///
/// Regenerate with `uv run python tests/exchange/foreign/generate/generate_nurbs.py`.
const OCCT_LOFTED: &str = include_str!("foreign/files/lofted.step");

/// A circle swept along a spline, from OpenCascade.
const OCCT_SWEPT_CIRCLE: &str = include_str!("foreign/files/swept_circle.step");

#[test]
fn foreign_splines_import_as_valid_oriented_solids() {
    for text in [OCCT_LOFTED, OCCT_SWEPT_CIRCLE] {
        let import = read(text);
        let map = import.shapes[0].model();
        validate_gmap(map.topology()).expect("the sewn map should satisfy the gmap axioms");
        validate_all_solid_manifolds(map).expect("the shell should be closed");
        validate_all_solid_orientations(map).expect("every face should point outward");
    }
}

#[test]
fn a_foreign_loft_arrives_on_spline_supports() {
    // The simple spelling: a polynomial B-spline is a leaf type, so each of
    // these is one record carrying its inherited attributes as well as its
    // own — a different attribute layout from the rational form under the
    // very same keyword.
    let import = read(OCCT_LOFTED);
    let solid = import.shapes[0].solid();

    assert!(
        import.report.is_clean(),
        "nothing should have been given up: {:?}",
        import.report.skipped
    );
    let splines = solid
        .faces()
        .iter()
        .filter(|face| matches!(face.surface(), ngk::geometry::Surface::Nurbs(_)))
        .count();
    assert!(splines > 0, "a loft is walled with spline patches");

    let spline_edges = solid
        .edges()
        .iter()
        .filter(|edge| matches!(edge.curve(), Some(ngk::geometry::Curve::Nurbs(_))))
        .count();
    assert!(spline_edges > 0, "and bounded by spline edges");
}

#[test]
fn a_foreign_swept_surface_arrives_as_one_clamped_spline_wall() {
    // The complex spelling, and the awkward one: the surface's two degrees
    // differ, so a transposed control net is visible rather than merely wrong,
    // and its u knots are unclamped, so it evaluates over its own domain only
    // after being clamped. The wall is periodic too, so it arrives cut open
    // and has to heal back to one ring face.
    let import = read(OCCT_SWEPT_CIRCLE);
    let solid = import.shapes[0].solid();

    assert!(
        import.report.is_clean(),
        "nothing should have been given up: {:?}",
        import.report.skipped
    );
    assert_eq!(solid.faces().len(), 3, "a wall between two caps");

    let wall = solid
        .faces()
        .iter()
        .find_map(|face| match face.surface() {
            ngk::geometry::Surface::Nurbs(surface) => Some(surface.clone()),
            _ => None,
        })
        .expect("the swept wall should arrive as a NURBS surface");
    assert_ne!(
        wall.degree_u().get(),
        wall.degree_v().get(),
        "the two degrees differ, which is what makes a transposed net visible",
    );
    assert!(
        wall.knots_u().is_clamped(wall.degree_u()) && wall.knots_v().is_clamped(wall.degree_v()),
        "an unclamped patch should be clamped on the way in",
    );
}
