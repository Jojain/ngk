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

/// A solid with a cavity spans two raw 3-cells, which the rule forbids.
///
/// Its outer shell and its void are two closed surfaces with nothing between
/// them, so the `alpha0`/`alpha1`/`alpha2` walk from either never reaches the
/// other. What would join them is a scaffold face the solid owns — the cut a
/// boundary walk turns across rather than crosses, which is what keeps the two
/// shells two shells while the material is one cell. Nothing in this tree
/// synthesises one yet, so a cavity is the one shape the inventory still
/// reports, and it is why the check is not part of commit.
#[test]
fn a_solid_with_a_cavity_spans_two_raw_cells() {
    let (model, _) = crate::hollow::hollow_sphere(2.0, 1.0).into_model();
    let violations = cell_occupancy_violations(&model);

    assert!(
        violations
            .iter()
            .any(|violation| matches!(violation, CellOccupancyError::SpansSeveralCells { .. })),
        "a hollow sphere should report a solid spanning two cells; it reports {violations:?}",
    );
}
