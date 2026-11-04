use nalgebra::Vector3;

use ngk::builders::profiles::add_polyline;
use ngk::builders::sweep::{SweepOptions, SweepTransition};
use ngk::geometry::{Plane, Point3};
use ngk::model::Model;
use ngk::modeling::{faces, sweep};
use ngk::topology::payload::StandardPayload;
use ngk::topology::validation::validate_solid_orientation;

#[test]
fn sweep_face_returns_an_owned_transitioned_solid() {
    let section = faces::rectangle(
        Plane::from_xy(Point3::new(1.0, 0.0, 0.0), Vector3::x(), Vector3::y()),
        1.0,
        1.0,
    )
    .expect("section should build");
    let mut spine_model = Model::<StandardPayload>::new();
    let spine = add_polyline(
        &mut spine_model,
        &[
            Point3::origin(),
            Point3::new(0.0, 0.0, 4.0),
            Point3::new(4.0, 0.0, 4.0),
        ],
    )
    .expect("spine should build");
    let spine = spine_model.profile_unchecked(spine);

    let solid = sweep::sweep_face(
        section,
        &spine,
        SweepOptions {
            transition: SweepTransition::Straight,
            ..SweepOptions::default()
        },
    )
    .expect("the owned face should sweep");

    validate_solid_orientation(solid.model(), solid.key())
        .expect("the owned sweep should be outward");
}

fn straight_spine(points: &[Point3]) -> Model<StandardPayload> {
    let mut model = Model::<StandardPayload>::new();
    add_polyline(&mut model, points).expect("spine should build");
    model
}

#[test]
fn a_disc_swept_along_an_edge_is_a_three_face_cylinder() {
    let disc = faces::circle(Plane::xy(), 1.0).expect("disc should build");
    let spine_model = straight_spine(&[Point3::origin(), Point3::new(0.0, 0.0, 4.0)]);
    let (spine_key, _) = spine_model.iter_profiles().next().expect("one profile");
    let spine = spine_model.profile_unchecked(spine_key);
    let spine_edge = spine.edges()[0];

    let solid = sweep::sweep_face(disc, &spine_edge, SweepOptions::default())
        .expect("a disc should sweep along an edge");

    validate_solid_orientation(solid.model(), solid.key()).expect("the cylinder should be outward");
    assert_eq!(solid.solid().faces().len(), 3);
    let volume = solid.solid().volume().expect("the cylinder has a volume");
    assert!(
        (volume - 4.0 * std::f64::consts::PI).abs() < 1e-6,
        "volume {volume}"
    );
}

#[test]
fn a_disc_swept_along_a_bent_profile_is_a_mitred_tube() {
    let disc = faces::circle(Plane::xy(), 1.0).expect("disc should build");
    let spine_model = straight_spine(&[
        Point3::origin(),
        Point3::new(0.0, 0.0, 4.0),
        Point3::new(4.0, 0.0, 4.0),
    ]);
    let (spine_key, _) = spine_model.iter_profiles().next().expect("one profile");
    let spine = spine_model.profile_unchecked(spine_key);

    let solid = sweep::sweep_face(
        disc,
        &spine,
        SweepOptions {
            transition: SweepTransition::Straight,
            ..SweepOptions::default()
        },
    )
    .expect("a disc should sweep along a bent profile");

    validate_solid_orientation(solid.model(), solid.key()).expect("the tube should be outward");
    // The disc's centroid runs on the spine, and a mitre keeps its whole
    // 8-long path.
    let volume = solid.solid().volume().expect("the tube has a volume");
    assert!(
        (volume - 8.0 * std::f64::consts::PI).abs() < 1e-6,
        "volume {volume}"
    );
}

#[test]
fn an_annulus_swept_along_an_edge_is_a_tube() {
    let annulus = faces::annulus(Plane::xy(), 2.0, 1.0).expect("annulus should build");
    let spine_model = straight_spine(&[Point3::origin(), Point3::new(0.0, 0.0, 4.0)]);
    let (spine_key, _) = spine_model.iter_profiles().next().expect("one profile");
    let spine = spine_model.profile_unchecked(spine_key);
    let spine_edge = spine.edges()[0];

    let solid = sweep::sweep_face(annulus, &spine_edge, SweepOptions::default())
        .expect("an annulus should sweep along an edge");

    validate_solid_orientation(solid.model(), solid.key()).expect("the tube should be outward");
    let volume = solid.solid().volume().expect("the tube has a volume");
    let expected = std::f64::consts::PI * (4.0 - 1.0) * 4.0;
    assert!((volume - expected).abs() < 1e-6, "volume {volume}");
}
