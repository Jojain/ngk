//! The seam pass: a seamed model in, a seamless model out.

use ngk::geometry::{Axis2, DomainSide};
use ngk::healing::{HealingOptions, remove_redundant_cells};
use ngk::topology::attributes::LoopKind;
use ngk::topology::validation::validate_solid_manifold;

use crate::seamed::{seamed_cylinder_wall, seamed_revolved_sphere, seamed_spherical_cap};

/// The three shapes a seam can be hiding, canonicalized by one pass.
///
/// What is left once the cut is gone is decided by what closes the direction the
/// seam ran across: two rims make a ring, one rim and a pole make a cap, and a
/// sphere has neither.
#[test]
fn the_seam_pass_canonicalizes_every_periodic_face() {
    let (mut wall_map, wall) = seamed_cylinder_wall(1.0, 2.0);
    let report = remove_redundant_cells(&mut wall_map, HealingOptions::default())
        .expect("healing a wall should commit");
    assert_eq!(report.removed_seams.len(), 1, "the wall's cut is a seam");
    assert!(
        report.removed_edges.is_empty(),
        "a wall carries no redundant edge, only a seam"
    );
    let kinds = wall_map
        .face_unchecked(wall)
        .loops()
        .iter()
        .map(|loop_| loop_.kind())
        .collect::<Vec<_>>();
    assert_eq!(
        kinds,
        vec![
            LoopKind::Wrapping { axis: Axis2::U },
            LoopKind::Wrapping { axis: Axis2::U }
        ]
    );

    let (mut cap_map, cap) = seamed_spherical_cap(1.0, 0.5);
    let report = remove_redundant_cells(&mut cap_map, HealingOptions::default())
        .expect("healing a cap should commit");
    assert_eq!(report.removed_seams.len(), 1);
    assert_eq!(
        cap_map
            .face_unchecked(cap)
            .loops()
            .iter()
            .map(|loop_| loop_.kind())
            .collect::<Vec<_>>(),
        vec![LoopKind::Capping {
            axis: Axis2::U,
            side: DomainSide::High
        }]
    );

    let (mut sphere_map, sphere, solid) = seamed_revolved_sphere(1.0);
    let report = remove_redundant_cells(&mut sphere_map, HealingOptions::default())
        .expect("healing a sphere should commit");
    assert_eq!(report.removed_seams.len(), 1);
    assert!(sphere_map.face_unchecked(sphere).loops().is_empty());
    assert_eq!(sphere_map.dart_count(), 0);
    validate_solid_manifold(&sphere_map, solid).expect("the healed sphere is still well formed");
}

/// `seams_only` is what an importer runs: the cut comes off and nothing else
/// does.
///
/// The two halves are checked on the redundancies each option is about. A
/// seamed wall carries a cut and nothing else, and the seams-only run takes it;
/// a split block carries an ordinary redundant vertex and no cut, and the same
/// run leaves it exactly where it is — which a full run then removes, so the
/// subject really was healable and the option really did decline it.
#[test]
fn seams_only_removes_the_cut_and_declines_everything_else() {
    use ngk::builders::faces::split_face_edge;
    use ngk::modeling::solids;

    let (mut wall_map, wall) = seamed_cylinder_wall(1.0, 2.0);
    let edges_before = wall_map.iter_edges().count();
    let report = remove_redundant_cells(&mut wall_map, HealingOptions::seams_only())
        .expect("a seams-only run should commit");
    assert_eq!(report.removed_seams.len(), 1, "the seam comes off");
    assert_eq!(wall_map.iter_edges().count(), edges_before - 1);
    assert_eq!(wall_map.face_unchecked(wall).loops().len(), 2);

    let (mut block_map, _) = solids::block(2.0, 2.0, 2.0).expect("block").into_map();
    let face = block_map.iter_faces().next().expect("a block has faces").0;
    let edge = block_map
        .face_unchecked(face)
        .edges()
        .first()
        .expect("a block face has edges")
        .key();
    split_face_edge(&mut block_map, face, edge, 0.5).expect("splitting a block edge should work");
    let vertices_before = block_map.iter_vertices().count();

    let report = remove_redundant_cells(&mut block_map, HealingOptions::seams_only())
        .expect("a seams-only run should commit");
    assert!(
        report.is_empty(),
        "a split block carries no cut, so a seams-only run has nothing to do"
    );
    assert_eq!(block_map.iter_vertices().count(), vertices_before);

    let report = remove_redundant_cells(&mut block_map, HealingOptions::default())
        .expect("a full run should commit");
    assert_eq!(
        report.removed_vertices.len(),
        1,
        "the split vertex was healable all along"
    );
}

/// A seamless model has no seam to remove.
#[test]
fn the_seam_pass_leaves_a_seamless_cylinder_alone() {
    use ngk::modeling::solids;

    let (mut g, _) = solids::cylinder(1.0, 2.0).expect("cylinder").into_map();
    let report = remove_redundant_cells(&mut g, HealingOptions::seams_only())
        .expect("a seams-only run should commit");

    assert!(
        report.removed_seams.is_empty(),
        "no builder in this tree makes a seam"
    );
    assert!(report.is_empty());
}
