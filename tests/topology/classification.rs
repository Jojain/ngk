//! What a shape built by an ordinary builder says about its own raw cells.
//!
//! The subdivision layer was proved in `subdivision.rs` against fixtures whose
//! labels were written by hand. These tests ask the same questions of a model
//! an ordinary builder produced, with nothing labelled deliberately: a shape
//! with no scaffold should still classify completely, because every entity
//! contains the cell its own anchor sits in and the classification reads that
//! off the entity stores rather than waiting to be told.

use ngk::geometry::Plane;
use ngk::model::{Cell0, Cell1, Cell2, Model};
use ngk::modeling::faces;
use ngk::topology::gmap::Dim;
use ngk::topology::payload::StandardPayload;
use ngk::topology::subdivision::{EntityOwner, boundary_cycles, recover_region};

/// A planar rectangle: four edges, four vertices, one face, and no scaffold.
fn rectangle() -> Model<StandardPayload> {
    faces::rectangle(Plane::xy(), 2.0, 3.0)
        .expect("a rectangle builds")
        .into_model()
        .0
}

#[test]
fn every_raw_cell_of_a_built_rectangle_names_the_entity_it_belongs_to() {
    let model = rectangle();
    let ownership = model.ownership();

    for dart in model.cells(Dim::Two) {
        let key = model
            .cell_key::<Cell2>(dart)
            .expect("a built face's 2-cell resolves to its key");
        assert_eq!(
            ownership.owner(Dim::Two, dart),
            Some(EntityOwner::Face(key)),
            "a raw 2-cell belongs to the face it backs",
        );
    }

    for dart in model.cells(Dim::One) {
        let key = model
            .cell_key::<Cell1>(dart)
            .expect("a built edge's 1-cell resolves to its key");
        assert_eq!(
            ownership.owner(Dim::One, dart),
            Some(EntityOwner::Edge(key)),
            "a raw 1-cell belongs to the edge it backs",
        );
    }

    for dart in model.cells(Dim::Zero) {
        let key = model
            .cell_key::<Cell0>(dart)
            .expect("a built vertex's 0-cell resolves to its key");
        assert_eq!(
            ownership.owner(Dim::Zero, dart),
            Some(EntityOwner::Vertex(key)),
            "a raw 0-cell belongs to the vertex it backs",
        );
    }
}

#[test]
fn a_shape_with_no_scaffold_stores_no_labels_at_all() {
    let model = rectangle();

    assert!(
        model.subdivision().is_empty(),
        "a rectangle has no cell inside anything but itself, so there is \
         nothing to write down -- the classification is entirely derived",
    );
    assert!(
        model
            .cells(Dim::Two)
            .all(|dart| model.ownership().owner(Dim::Two, dart).is_some()),
        "and yet every cell still has an owner, which is the point",
    );
}

#[test]
fn a_built_rectangle_walks_back_to_one_face_and_four_logical_edges() {
    let model = rectangle();
    let ownership = model.ownership();
    let (face_key, _) = model
        .iter_faces()
        .next()
        .expect("the rectangle has exactly one face");
    // Any dart of the face's own 2-cell anchors the walk; a rectangle is one
    // raw face, so there is only the one cell to start from.
    let anchor = model
        .cells(Dim::Two)
        .next()
        .expect("the rectangle has a 2-cell");

    let region = recover_region(
        model.topology(),
        ownership,
        EntityOwner::Face(face_key),
        anchor,
    )
    .expect("a built face's region is recoverable from its labels alone");

    let cycles = boundary_cycles(model.topology(), ownership, &region)
        .expect("a built face has extractable boundary cycles");
    assert_eq!(cycles.len(), 1, "a rectangle has one boundary loop");

    let uses = cycles[0]
        .logical_uses(ownership)
        .expect("every boundary piece of a rectangle is a logical edge");
    assert_eq!(
        uses.len(),
        4,
        "walked back to the four edges it was built from"
    );

    let walked: Vec<_> = uses.iter().map(|use_| use_.edge).collect();
    let mut distinct = walked.clone();
    distinct.sort();
    distinct.dedup();
    assert_eq!(
        distinct.len(),
        4,
        "and they are four different edges, not one edge four times",
    );
    for edge in walked {
        assert!(
            model.edge_attr(edge).is_some(),
            "a walked edge key names an edge this model actually holds",
        );
    }
}
