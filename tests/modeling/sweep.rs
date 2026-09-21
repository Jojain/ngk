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
