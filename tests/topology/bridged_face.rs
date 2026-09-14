//! What a bridged face is, asked of a shape an ordinary builder produced.
//!
//! An annulus is the smallest face whose boundary is not one cycle, so it is
//! the smallest shape that needs a bridge: one raw edge the boundary walk uses
//! twice, owned by the face. These tests ask that the connectivity really lives
//! in the map — one raw 2-cell, two derived loops — rather than in a list the
//! face attribute keeps.

use ngk::geometry::Plane;
use ngk::model::Model;
use ngk::modeling::faces;
use ngk::topology::edge::Edge;
use ngk::topology::gmap::Dim;
use ngk::topology::payload::StandardPayload;
use ngk::topology::subdivision::{EntityOwner, boundary_cycles, recover_region};

fn annulus() -> Model<StandardPayload> {
    faces::annulus(Plane::xy(), 2.0, 1.0)
        .expect("an annulus builds")
        .into_model()
        .0
}

#[test]
fn a_bridged_annulus_is_one_raw_face_with_every_cell_classified() {
    let model = annulus();
    let ownership = model.ownership();

    assert_eq!(
        model.cells(Dim::Two).count(),
        1,
        "the bridge joins both rims into a single raw 2-cell, which is what \
         makes the annulus one face without a list saying so",
    );
    for dimension in [Dim::Two, Dim::One, Dim::Zero] {
        assert!(
            model
                .cells(dimension)
                .all(|dart| ownership.owner(dimension, dart).is_some()),
            "every raw {dimension:?}-cell names the entity it belongs to",
        );
    }
}

#[test]
fn the_bridge_belongs_to_the_face_and_is_never_a_logical_edge() {
    let model = annulus();
    let ownership = model.ownership();
    let (face, _) = model.iter_faces().next().expect("the annulus has one face");

    let owners: Vec<_> = model
        .cells(Dim::One)
        .map(|dart| ownership.owner(Dim::One, dart))
        .collect();
    assert_eq!(
        owners
            .iter()
            .filter(|owner| **owner == Some(EntityOwner::Face(face)))
            .count(),
        1,
        "exactly one raw edge is interior to the face: the bridge",
    );
    assert_eq!(
        model.iter_edges().count(),
        2,
        "and it is scaffold, so the annulus still holds only its two rims",
    );
}

#[test]
fn both_rims_of_an_annulus_stay_unmarked() {
    let model = annulus();
    let ownership = model.ownership();

    assert_eq!(
        model.iter_vertices().count(),
        0,
        "a bridge foot is where a rim closes, not a corner anything meets at",
    );
    for dart in model.cells(Dim::Zero) {
        assert!(
            matches!(ownership.owner(Dim::Zero, dart), Some(EntityOwner::Edge(_))),
            "so each 0-cell is classified inside the rim it sits on",
        );
    }
    for (key, _) in model.iter_edges() {
        assert!(
            matches!(model.edge_unchecked(key), Edge::Unmarked(_)),
            "which is what keeps both circles unmarked",
        );
    }
}

#[test]
fn the_boundary_walk_finds_both_rims_and_hides_the_bridge() {
    let model = annulus();
    let ownership = model.ownership();
    let (face, _) = model.iter_faces().next().expect("the annulus has one face");
    let anchor = model
        .cells(Dim::Two)
        .next()
        .expect("the annulus has a 2-cell");

    let region = recover_region(model.topology(), ownership, EntityOwner::Face(face), anchor)
        .expect("a bridged face's region is recoverable");
    let cycles = boundary_cycles(model.topology(), ownership, &region)
        .expect("a bridged face has extractable boundary cycles");

    assert_eq!(cycles.len(), 2, "an annulus has two boundaries");
    for cycle in &cycles {
        let uses = cycle
            .logical_uses(ownership)
            .expect("every boundary piece is a logical edge");
        assert_eq!(uses.len(), 1, "each rim is one whole circle");
    }
    let walked: Vec<_> = cycles
        .iter()
        .flat_map(|cycle| cycle.logical_uses(ownership).expect("uses"))
        .map(|use_| use_.edge)
        .collect();
    let mut distinct = walked.clone();
    distinct.sort();
    distinct.dedup();
    assert_eq!(distinct.len(), 2, "and the two rims are different edges");
}

#[test]
fn a_face_reports_the_loops_its_own_boundary_walk_finds() {
    let model = annulus();
    let (face, _) = model.iter_faces().next().expect("the annulus has one face");
    let view = model.face_unchecked(face);

    assert_eq!(view.loops().len(), 2, "an outer rim and a hole");
    assert_eq!(
        view.outer_loop()
            .expect("an annulus has an outer loop")
            .len(),
        1,
        "the outer loop runs along one whole circle, not across the bridge",
    );
    assert_eq!(view.inner_loops().len(), 1, "and one of them is the hole");
    assert_eq!(
        view.inner_loops()[0].len(),
        1,
        "which is also one whole circle",
    );
}
