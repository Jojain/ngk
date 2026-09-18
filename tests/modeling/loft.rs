use ngk::builders::loft::{LoftError, LoftOptions};
use ngk::geometry::{LINEAR_TOLERANCE, Point3};
use ngk::modeling::{faces, loft, profiles};
use ngk::topology::closed::Closeable;
use ngk::topology::validation::{validate_all_solid_manifolds, validate_all_solid_orientations};

/// A square of side 2 centred on the z axis at height `z`, as an owned shape.
fn square(z: f64) -> ngk::topology::shape::Shape<ngk::topology::shape::ProfileTag> {
    profiles::polygon(&[
        Point3::new(-1.0, -1.0, z),
        Point3::new(1.0, -1.0, z),
        Point3::new(1.0, 1.0, z),
        Point3::new(-1.0, 1.0, z),
    ])
    .expect("a square is a valid polygon")
}

#[test]
fn lofting_owned_profiles_copies_them_into_one_sheet() {
    let bottom = square(0.0);
    let top = square(2.0);

    let shape = loft::loft(&[&bottom, &top], LoftOptions::default())
        .expect("two squares should loft");
    let sheet = shape.sheet();

    // A tube: closed round its ring, but open at the two ends, so it is not
    // a shell. Only a capped loft closes in that direction.
    assert!(!sheet.is_closed());
    assert_eq!(sheet.faces().len(), 4);
    for vertex in sheet.vertices() {
        let z = vertex.point().z;
        assert!(
            z.abs() <= LINEAR_TOLERANCE || (z - 2.0).abs() <= LINEAR_TOLERANCE,
            "a lofted vertex should sit on one of the two sections, not at z = {z}"
        );
    }
}

#[test]
fn lofting_owned_faces_gives_a_capped_solid() {
    let bottom = faces::from_profile(&square(0.0)).expect("a square bounds a face");
    let top = faces::from_profile(&square(3.0)).expect("a square bounds a face");

    let shape =
        loft::loft(&[&bottom, &top], LoftOptions::ruled()).expect("two faces should loft");
    let model = shape.model();

    assert!(model.solid(shape.handle()).is_some());
    validate_all_solid_manifolds(model).expect("a lofted solid should be manifold");
    validate_all_solid_orientations(model).expect("a lofted solid should face outward");
}

#[test]
fn a_run_of_profiles_that_is_neither_all_open_nor_all_closed_is_refused() {
    let ring = square(0.0);
    let wire = profiles::polyline(&[Point3::new(0.0, 0.0, 2.0), Point3::new(1.0, 0.0, 2.0)])
        .expect("two points make a polyline");

    // `add_loft` cannot be handed a mixed run at all: its section types see to
    // that. Here the kind is inferred from the profiles, so this is the one
    // place the mix has to be reported.
    let error = loft::loft(&[&ring, &wire], LoftOptions::default())
        .err()
        .expect("a mixed run should be refused");
    assert_eq!(error, LoftError::MixedProfiles { index: 1 });
}
