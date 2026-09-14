//! The logical topology recovered from a pure Model embedding.
//!
//! Each fixture is a raw map with no domain attribute on it at all, plus a
//! labelling saying which logical entity's interior contains each raw cell.
//! What these tests check is that the entities, their extents and their
//! boundaries all fall out of walking that pair — no stored loop seeds, no
//! stored shell roots, no stored membership lists.

use ngk::topology::embedding::{
    Embedding, EmbeddingError, EntityOwner, boundary_cycles, boundary_shells, boundary_vertices,
    recover_region,
};
use ngk::topology::gmap::{Dart, Dim};
use ngk::topology::validation::validate_gmap;

use crate::scaffold::{
    Scaffold, cavity_solid, circle, cylinder, handle_solid, holed_face, segment, sphere, torus,
};

/// Counts the logical entities of one dimension.
fn count(scaffold: &Scaffold, dimension: Dim) -> usize {
    scaffold.owners(dimension).len()
}

/// Returns the one entity of a dimension, failing when there is not exactly one.
fn only(scaffold: &Scaffold, dimension: Dim) -> EntityOwner {
    let owners = scaffold.owners(dimension);
    assert_eq!(owners.len(), 1, "expected exactly one {dimension:?} entity");
    owners[0]
}

#[test]
fn a_segment_is_one_edge_between_two_vertices() {
    let scaffold = segment();
    validate_gmap(scaffold.map()).expect("a segment should be a valid gmap");

    assert_eq!(count(&scaffold, Dim::One), 1);
    assert_eq!(count(&scaffold, Dim::Zero), 2);

    let edge = only(&scaffold, Dim::One);
    let region = scaffold.region(edge);
    assert_eq!(region.cells(scaffold.map()).len(), 1);

    let ends = boundary_vertices(scaffold.map(), &scaffold.index(), &region)
        .expect("a segment's ends should be recoverable");
    assert_eq!(ends.len(), 2);
}

#[test]
fn a_circle_is_one_edge_with_no_vertex() {
    let scaffold = circle();
    validate_gmap(scaffold.map()).expect("a circle should be a valid gmap");

    assert_eq!(count(&scaffold, Dim::One), 1);
    assert_eq!(
        count(&scaffold, Dim::Zero),
        0,
        "the closure point is inside the edge, not a vertex of its own"
    );

    let edge = only(&scaffold, Dim::One);
    let region = scaffold.region(edge);
    assert_eq!(region.cells(scaffold.map()).len(), 1);
    assert!(
        boundary_vertices(scaffold.map(), &scaffold.index(), &region)
            .expect("a circle's boundary should be recoverable")
            .is_empty()
    );
}

#[test]
fn a_circle_reads_the_same_way_round_from_either_dart() {
    let scaffold = circle();
    let index = scaffold.index();
    let edge = only(&scaffold, Dim::One);

    let forward = recover_region(scaffold.map(), &index, edge, Dart::new(0))
        .expect("the circle should be walkable from its first dart");
    let backward = recover_region(scaffold.map(), &index, edge, Dart::new(1))
        .expect("the circle should be walkable from its second dart");

    assert_eq!(forward.len(), backward.len());
    for dart in forward.darts() {
        assert!(backward.contains(dart));
        assert_ne!(
            forward.is_aligned(dart),
            backward.is_aligned(dart),
            "reversing the anchor should reverse every dart's sense"
        );
    }
}

#[test]
fn a_capped_cylinder_has_three_faces_two_edges_and_no_vertex() {
    let scaffold = cylinder();
    validate_gmap(scaffold.map()).expect("a capped cylinder should be a valid gmap");

    assert_eq!(count(&scaffold, Dim::Two), 3);
    assert_eq!(count(&scaffold, Dim::One), 2);
    assert_eq!(count(&scaffold, Dim::Zero), 0);
    scaffold.regions();
}

#[test]
fn a_cylinder_wall_has_two_loops_and_never_emits_its_seam() {
    let scaffold = cylinder();
    let index = scaffold.index();
    let wall = scaffold.owners(Dim::Two)[0];
    let region = scaffold.region(wall);

    let cycles = boundary_cycles(scaffold.map(), &index, &region)
        .expect("a wall's rims should be recoverable");
    assert_eq!(cycles.len(), 2, "a tube is bounded by its two rims");

    let seam = index.owner(Dim::One, Dart::new(2));
    assert_eq!(seam, Some(wall), "the seam belongs to the wall");
    for cycle in &cycles {
        let edges = cycle
            .edge_keys(&index)
            .expect("every rim dart should belong to a logical edge");
        assert_eq!(edges.len(), 1, "a rim is one closed edge walked once");
        for &dart in cycle.darts() {
            assert_ne!(
                index.owner(Dim::One, dart),
                Some(wall),
                "a loop must not emit the cut it turns across"
            );
        }
    }

    let rims: Vec<_> = cycles
        .iter()
        .map(|cycle| cycle.edge_keys(&index).unwrap()[0])
        .collect();
    assert_ne!(rims[0], rims[1], "the two rims are different edges");
}

#[test]
fn a_cap_and_its_wall_rim_are_opposite_uses_of_one_edge() {
    let scaffold = cylinder();
    let index = scaffold.index();
    let map = scaffold.map();

    let faces = scaffold.owners(Dim::Two);
    let (wall, caps) = (faces[0], &faces[1..]);
    let wall_cycles = boundary_cycles(map, &index, &scaffold.region(wall)).unwrap();

    for &cap in caps {
        let cap_cycles = boundary_cycles(map, &index, &scaffold.region(cap)).unwrap();
        assert_eq!(cap_cycles.len(), 1, "a cap is bounded by one rim");
        let cap_dart = cap_cycles[0].darts()[0];

        let matching = wall_cycles
            .iter()
            .find(|cycle| {
                cycle
                    .darts()
                    .contains(&map.alpha(Dim::Zero, map.alpha(Dim::Two, cap_dart)))
            })
            .expect("every cap rim should also be a wall rim");

        assert_eq!(
            index.owner(Dim::One, cap_dart),
            index.owner(Dim::One, matching.darts()[0]),
            "both uses run along the same logical edge"
        );
        assert_eq!(
            map.alpha(Dim::Zero, map.alpha(Dim::Two, cap_dart)),
            matching.darts()[0],
            "the two uses of a shared rim run opposite ways round it"
        );
    }
}

#[test]
fn a_bridged_hole_yields_two_cycles_and_hides_the_bridge() {
    let scaffold = holed_face(1);
    validate_gmap(scaffold.map()).expect("a bridged annulus should be a valid gmap");

    let index = scaffold.index();
    let face = only(&scaffold, Dim::Two);
    let cycles = boundary_cycles(scaffold.map(), &index, &scaffold.region(face))
        .expect("an annulus should have recoverable loops");

    assert_eq!(cycles.len(), 2, "an outer boundary and one hole");
    for cycle in &cycles {
        assert_eq!(cycle.len(), 4, "each square loop walks four edges");
        for &dart in cycle.darts() {
            assert_ne!(
                index.owner(Dim::One, dart),
                Some(face),
                "the bridge is interior to the face and is never emitted"
            );
        }
    }
}

#[test]
fn two_bridged_holes_yield_three_cycles() {
    let scaffold = holed_face(2);
    validate_gmap(scaffold.map()).expect("a twice-bridged face should be a valid gmap");

    let index = scaffold.index();
    let face = only(&scaffold, Dim::Two);
    let cycles = boundary_cycles(scaffold.map(), &index, &scaffold.region(face))
        .expect("a face with two holes should have recoverable loops");

    assert_eq!(cycles.len(), 3, "an outer boundary and two holes");
    assert_eq!(scaffold.raw_cell_count(Dim::Two), 1, "still one raw face");
}

#[test]
fn every_loop_of_a_face_is_wound_the_same_way() {
    let scaffold = holed_face(2);
    let index = scaffold.index();
    let face = only(&scaffold, Dim::Two);
    let region = scaffold.region(face);

    for cycle in boundary_cycles(scaffold.map(), &index, &region).unwrap() {
        for &dart in cycle.darts() {
            assert!(
                region.is_aligned(dart),
                "a loop seeded against the face's sense would wind the wrong way"
            );
        }
    }
}

#[test]
fn a_sphere_is_one_face_with_no_boundary() {
    let scaffold = sphere();
    validate_gmap(scaffold.map()).expect("a bigon with identified sides should be a valid gmap");

    assert_eq!(count(&scaffold, Dim::Two), 1);
    assert_eq!(count(&scaffold, Dim::One), 0);
    assert_eq!(count(&scaffold, Dim::Zero), 0);

    let face = only(&scaffold, Dim::Two);
    let region = scaffold.region(face);
    assert_eq!(
        region.cells(scaffold.map()).len(),
        1,
        "the face occupies the one raw 2-cell its bigon is"
    );
    assert!(
        boundary_cycles(scaffold.map(), &scaffold.index(), &region)
            .unwrap()
            .is_empty(),
        "a closed surface has no loop at all"
    );
}

#[test]
fn a_torus_is_one_face_with_no_boundary() {
    let scaffold = torus();
    validate_gmap(scaffold.map()).expect("a periodic square should be a valid gmap");

    assert_eq!(count(&scaffold, Dim::Two), 1);
    assert_eq!(count(&scaffold, Dim::One), 0);
    assert_eq!(count(&scaffold, Dim::Zero), 0);
    assert_eq!(scaffold.raw_cell_count(Dim::Zero), 1);
    assert_eq!(scaffold.raw_cell_count(Dim::One), 2);
    assert_eq!(scaffold.raw_cell_count(Dim::Two), 1);

    let face = only(&scaffold, Dim::Two);
    assert!(
        boundary_cycles(scaffold.map(), &scaffold.index(), &scaffold.region(face))
            .unwrap()
            .is_empty()
    );
}

#[test]
fn a_cavity_solid_is_one_raw_cell_holding_its_cut_face() {
    let scaffold = cavity_solid();
    validate_gmap(scaffold.map()).expect("a pillow with an identified side should be a valid gmap");

    assert_eq!(count(&scaffold, Dim::Three), 1);
    assert_eq!(
        scaffold.raw_cell_count(Dim::Three),
        1,
        "the material between two spheres is one raw 3-cell"
    );

    let solid = only(&scaffold, Dim::Three);
    assert_eq!(
        scaffold.region(solid).cells(scaffold.map()).len(),
        1,
        "a solid's region is the one cell it occupies"
    );
    assert_eq!(
        count(&scaffold, Dim::Two),
        2,
        "an outer face and an inner one; the cut between them is the solid's own"
    );
}

#[test]
fn a_cavity_solid_has_an_outer_shell_and_a_void_shell() {
    let scaffold = cavity_solid();
    let index = scaffold.index();
    let solid = only(&scaffold, Dim::Three);

    let shells = boundary_shells(scaffold.map(), &index, &scaffold.region(solid))
        .expect("a cavity solid's shells should be recoverable");
    assert_eq!(
        shells.len(),
        2,
        "an outer boundary and one void: the walk turns across the cut rather          than crossing it, so the two never join"
    );
    for shell in &shells {
        assert_eq!(
            shell.face_count(scaffold.map()),
            1,
            "each shell is one face"
        );
        assert_eq!(
            shell.euler_characteristic(scaffold.map(), &index),
            2,
            "both shells are spheres"
        );
    }
}

#[test]
fn a_handle_solid_has_one_shell_of_genus_one() {
    let scaffold = handle_solid();
    validate_gmap(scaffold.map()).expect("a cube with an identified side should be a valid gmap");

    let index = scaffold.index();
    let solid = only(&scaffold, Dim::Three);
    let region = scaffold.region(solid);
    assert_eq!(region.cells(scaffold.map()).len(), 1);

    let shells = boundary_shells(scaffold.map(), &index, &region)
        .expect("a handle's shell should be recoverable");
    assert_eq!(shells.len(), 1, "a shaft through a ring leaves one surface");
    assert_eq!(shells[0].face_count(scaffold.map()), 4);
    assert_eq!(
        shells[0].euler_characteristic(scaffold.map(), &index),
        0,
        "one handle, so the boundary is a torus"
    );
}

#[test]
fn an_interior_label_on_an_exterior_face_is_rejected() {
    let scaffold = cavity_solid();
    let solid = only(&scaffold, Dim::Three);
    let region = scaffold.region(solid);

    // A face on the outside of the material, recorded as if the solid ran
    // through it. A 2-cell embedded in a solid is a shape the rule allows --
    // that is what a cut face inside a cavity is -- but this one already carries
    // a face of its own, and a cell an entity occupies is not inside anything.
    let exterior = *region
        .frontier(&scaffold.index())
        .first()
        .expect("the solid has a boundary");
    let broken = scaffold.relabelled(Dim::Two, exterior, solid);

    let failure = scaffold
        .index_of(&broken)
        .expect_err("an exterior face cannot be interior to the solid");
    assert!(matches!(
        failure,
        EmbeddingError::ConflictingOwnership { held, claimed, .. }
            if held.dimension() == Dim::Two && claimed == solid
    ));
}

#[test]
fn a_label_crossing_a_public_boundary_is_rejected() {
    let scaffold = cylinder();
    let faces = scaffold.owners(Dim::Two);
    let wall = faces[0];

    // The rim between the wall and a cap, relabelled as interior to the wall.
    // Walking the wall then runs into the cap, which is a different face.
    let rim = boundary_cycles(scaffold.map(), &scaffold.index(), &scaffold.region(wall)).unwrap()
        [0]
    .darts()[0];
    let broken = scaffold.relabelled(Dim::One, rim, wall);

    let failure = scaffold
        .index_of(&broken)
        .expect_err("a face cannot own the boundary it shares with another face");
    assert!(matches!(
        failure,
        EmbeddingError::ConflictingOwnership { held, claimed, .. }
            if held.dimension() == Dim::One && claimed == wall
    ));
}

#[test]
fn a_face_recorded_on_a_2_cell_is_rejected() {
    let scaffold = sphere();
    let face = only(&scaffold, Dim::Two);
    // A face occupies exactly one 2-cell and that cell is read off its anchor,
    // so no record may name a 2-cell for a face at all. A record says a cell is
    // embedded in something larger, and a cell an entity occupies is not
    // embedded in anything.
    let mut same_dimension = Embedding::new();
    same_dimension.own(Dim::Two, Dart::new(0), face);

    let failure = scaffold
        .index_of(&same_dimension)
        .expect_err("a face's own dimension cannot be recorded");
    assert!(matches!(failure, EmbeddingError::OwnerNotAboveCell { .. }));
}

/// Re-recording one anchor is a correction to it, not a second opinion about it.
///
/// A seam edge that moves from one face's interior to another's is the
/// motivating case: the cell does not move and no second entry appears, the one
/// entry anchored there simply says something else afterwards.
#[test]
fn re_recording_an_anchor_replaces_what_it_said() {
    let mut scaffold = sphere();
    let face = only(&scaffold, Dim::Two);
    let other = scaffold.face();
    let seam = Dart::new(0);

    let mut embedding = Embedding::new();
    embedding.own(Dim::One, seam, face);
    embedding.own(Dim::One, seam, other);

    assert_eq!(
        embedding.owner_at(Dim::One, seam),
        Some(other),
        "the later record is the one that stands",
    );
    assert_eq!(
        embedding.records().count(),
        1,
        "and it replaced the earlier one rather than joining it",
    );
    scaffold
        .index_of(&embedding)
        .expect("a replaced label leaves nothing to contradict");
}

#[test]
fn two_entities_cannot_own_one_raw_cell() {
    let mut scaffold = sphere();
    let face = scaffold.face();
    let intruder = scaffold.face();
    let seam = Dart::new(0);
    let mut contested = Embedding::new();
    contested.own(Dim::One, seam, face);
    // A second record on the same 1-cell. Re-recording `seam` itself would
    // replace the first rather than contest it: one anchor holds one answer,
    // and it is two anchors meeting on one orbit that the index has to reject.
    let same_edge = scaffold.map().alpha(Dim::Zero, seam);
    contested.own(Dim::One, same_edge, intruder);

    let failure = scaffold
        .index_of(&contested)
        .expect_err("one cell cannot have two owners");
    assert!(matches!(
        failure,
        EmbeddingError::ConflictingOwnership { held, claimed, .. }
            if held == face && claimed == intruder
    ));
}

#[test]
fn an_owner_not_above_the_cell_it_records_is_rejected() {
    let mut scaffold = sphere();
    let edge = scaffold.edge();
    let mut inverted = Embedding::new();
    inverted.own(Dim::Two, Dart::new(0), edge);

    let failure = scaffold
        .index_of(&inverted)
        .expect_err("an edge cannot contain a face");
    assert!(matches!(failure, EmbeddingError::OwnerNotAboveCell { .. }));
}

#[test]
fn a_record_anchored_outside_the_map_is_rejected() {
    let scaffold = sphere();
    let mut dangling = Embedding::new();
    dangling.own(
        Dim::Two,
        Dart::new(scaffold.map().dart_count()),
        EntityOwner::Solid(Default::default()),
    );

    let failure = scaffold
        .index_of(&dangling)
        .expect_err("a record must name a dart that exists");
    assert!(matches!(failure, EmbeddingError::DanglingRecord { .. }));
}

#[test]
fn rebuilding_the_index_answers_exactly_as_the_first_one_did() {
    let scaffold = handle_solid();
    let first = scaffold.index();
    let second = scaffold.index();

    for dart in scaffold.map().darts() {
        for dimension in [Dim::Zero, Dim::One, Dim::Two, Dim::Three] {
            assert_eq!(first.owner(dimension, dart), second.owner(dimension, dart));
        }
    }
}
