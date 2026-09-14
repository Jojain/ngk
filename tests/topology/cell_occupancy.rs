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
//! which do not, so that the ones that do cannot quietly stop.

use ngk::geometry::Plane;
use ngk::model::Model;
use ngk::modeling::{faces, solids};
use ngk::topology::payload::StandardPayload;
use ngk::topology::validation::{CellOccupancyError, cell_occupancy_violations};

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

/// A face with no raw cell at all, which the rule forbids and this tree builds.
///
/// `solids::sphere` and `solids::torus` cover a closed support with one face
/// carrying no loop, so the face names no dart and the map holds none. That is
/// the "not zero" half of the rule going unmet, and it is why a shell anchors
/// at a face key rather than at a dart, and why a face's dart is optional.
#[test]
fn a_face_covering_a_closed_support_occupies_no_raw_cell() {
    for (name, model) in [
        (
            "sphere",
            solids::sphere(1.0)
                .expect("a sphere builds")
                .into_model()
                .0,
        ),
        (
            "torus",
            solids::torus(3.0, 1.0)
                .expect("a torus builds")
                .into_model()
                .0,
        ),
    ] {
        let violations = cell_occupancy_violations(&model);
        assert!(
            violations
                .iter()
                .any(|violation| matches!(violation, CellOccupancyError::OccupiesNoCell { .. })),
            "{name} covers a closed support with a face that names no dart, \
             so it should report an entity occupying no cell; it reports {violations:?}",
        );
    }
}
