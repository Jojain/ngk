//! Whether every entity occupies exactly one raw cell of its own dimension.
//!
//! The rule is one-directional: a logical entity has exactly one raw cell of
//! its own dimension, while a raw cell need not carry an entity at all. A cell
//! that carries none is embedded in the entity of higher dimension whose
//! interior contains it — a bridge to a hole, a periodic seam, the 0-cell where
//! a circle closes — and stays invisible above the map, which is what lets a
//! purely topological seam be hidden.
//!
//! Only faces and solids can break it. A vertex and an edge name one anchor
//! dart each, so the cell they occupy is that dart's; profiles and sheets are
//! aggregates rather than entities with a cell of their own.
//!
//! These tests are the inventory: they say which shapes hold to the rule and
//! which do not, so that the ones that do cannot quietly stop and the ones that
//! do not are named rather than assumed.

use ngk::geometry::Plane;
use ngk::model::Model;
use ngk::modeling::{faces, solids};
use ngk::topology::payload::StandardPayload;
use ngk::topology::validation::cell_occupancy_violations;

/// Every shape whose entities are each expected to occupy exactly one cell.
fn compliant_shapes() -> Vec<(&'static str, Model<StandardPayload>)> {
    vec![
        (
            "rectangle",
            faces::rectangle(Plane::xy(), 2.0, 3.0)
                .expect("a rectangle builds")
                .into_model()
                .0,
        ),
        (
            "disc",
            faces::circle(Plane::xy(), 1.0)
                .expect("a disc builds")
                .into_model()
                .0,
        ),
        (
            "annulus",
            faces::annulus(Plane::xy(), 2.0, 1.0)
                .expect("an annulus builds")
                .into_model()
                .0,
        ),
        (
            "block",
            solids::block(1.0, 2.0, 3.0)
                .expect("a block builds")
                .into_model()
                .0,
        ),
        (
            "cylinder",
            solids::cylinder(1.0, 2.0)
                .expect("a cylinder builds")
                .into_model()
                .0,
        ),
        (
            "sphere",
            solids::sphere(1.0).expect("a sphere builds").into_model().0,
        ),
        (
            "torus",
            solids::torus(3.0, 1.0)
                .expect("a torus builds")
                .into_model()
                .0,
        ),
        (
            "hollow sphere",
            crate::hollow::hollow_sphere(2.0, 1.0).into_model().0,
        ),
    ]
}

#[test]
fn every_entity_of_an_ordinary_shape_occupies_exactly_one_raw_cell() {
    for (name, model) in compliant_shapes() {
        let violations = cell_occupancy_violations(&model);
        assert!(
            violations.is_empty(),
            "{name} should hold one raw cell per entity, but reports {violations:?}",
        );
    }
}

#[test]
fn a_solid_with_a_cavity_occupies_one_raw_cell() {
    let (model, _) = crate::hollow::hollow_sphere(2.0, 1.0).into_model();
    let violations = cell_occupancy_violations(&model);

    assert!(
        violations.is_empty(),
        "a hollow sphere should hold one raw cell per entity, but reports {violations:?}",
    );
}
