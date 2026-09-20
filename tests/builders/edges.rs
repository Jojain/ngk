use std::f64::consts::TAU;

use ngk::builders::edges::add_helix;
use ngk::geometry::{Axis3, Curve, Interval, NativeParam};
use ngk::model::Model;
use ngk::topology::payload::StandardPayload;
use radians::Rad64;

#[test]
fn add_helix_builds_a_bounded_edge_over_the_requested_span() {
    let mut model = Model::<StandardPayload>::new();
    let axis = Axis3::z();

    let edge = add_helix(
        &mut model,
        axis,
        2.0,
        3.0,
        Rad64::ZERO,
        Rad64::new(2.0 * TAU),
    )
    .expect("helix edge should build");
    let view = model.edge(edge).expect("edge should exist");

    assert_eq!(view.parameter_interval(), Interval::new(0.0, 2.0 * TAU));
    assert!(matches!(view.curve(), Curve::Helix(helix) if
        helix.radius() == 2.0 && helix.pitch() == 3.0));
    assert_eq!(
        *view.bounded_unchecked().start().point(),
        view.curve().point_at(NativeParam::new(0.0))
    );
    let end = *view.bounded_unchecked().end().point();
    let expected = view.curve().point_at(NativeParam::new(2.0 * TAU));
    assert!((end - expected).norm() <= 1.0e-12);
}
