use nalgebra::Vector3;
use ngk::builders::loft::{LoftError, LoftOptions};
use ngk::geometry::{LINEAR_TOLERANCE, Plane, Point3};
use ngk::modeling::solids::fuse;
use ngk::modeling::{faces, loft, profiles};
use ngk::topology::closed::Closeable;
use ngk::topology::validation::{
    validate_all_solid_manifolds, validate_all_solid_orientations, validate_solid_manifold,
    validate_solid_orientation,
};

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

    let shape =
        loft::loft(&[&bottom, &top], LoftOptions::default()).expect("two squares should loft");
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

    let shape = loft::loft(&[&bottom, &top], LoftOptions::ruled()).expect("two faces should loft");
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

/// A square-to-circle loft running along `normal`, its square centred on the
/// run and its circle at `tip`.
fn tapered(
    centre: Point3,
    x_dir: Vector3<f64>,
    normal: Vector3<f64>,
    side: f64,
    tip: Point3,
    radius: f64,
) -> ngk::topology::shape::Shape<ngk::topology::shape::SolidTag> {
    let y_dir = normal.cross(&x_dir).normalize();
    let corner = centre - (side / 2.0) * x_dir.normalize() - (side / 2.0) * y_dir;
    let square = faces::rectangle(Plane::new(corner, x_dir, normal), side, side)
        .expect("a positive side bounds a rectangle");
    let circle = faces::circle(Plane::new(tip, x_dir, normal), radius)
        .expect("a positive radius bounds a circle");
    loft::loft(&[&square, &circle], LoftOptions::default()).expect("two faces should loft")
}

#[test]
fn two_crossing_lofts_fuse_into_one_closed_solid() {
    // Every lateral face of a square-to-circle loft carries the arc join its
    // circular section was written with: an interior knot of full multiplicity,
    // across which the patch is smooth but its parameterization is not. The
    // contact curves here run over those joins on both operands, which is what
    // a Boolean between two lofts has to be able to fit.
    let upright = tapered(
        Point3::origin(),
        Vector3::x(),
        Vector3::z(),
        4.0,
        Point3::new(0.0, 0.0, 5.0),
        2.0,
    );
    let sideways = tapered(
        Point3::new(-3.0, 0.0, 2.0),
        Vector3::y(),
        Vector3::x(),
        2.5,
        Point3::new(6.0, 0.0, 2.0),
        1.0,
    );

    let fused = fuse(upright, sideways).expect("two crossing lofts should fuse");

    assert_eq!(fused.solid().faces().len(), 16);
    validate_solid_manifold(fused.model(), fused.key()).expect("the fusion should be manifold");
    validate_solid_orientation(fused.model(), fused.key()).expect("the fusion should face outward");
}
