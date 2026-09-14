use nalgebra::Vector2;
use ngk::builders::boolean::{BooleanOperation, BooleanOptions, boolean};
use ngk::builders::faces::{FaceImprint, add_rectangle, split_face_by_imprints, split_face_edge};
use ngk::builders::removal::{
    CellRemovalError, MergedCell, is_removable, remove_cell, remove_cell_staged,
};
use ngk::geometry::{
    Axis2, Curve, Curve2, DomainSide, Frame, Plane, Point2, Point3, Surface, TrimmedCurve2,
};
use ngk::healing::{HealingOptions, HealingScope, remove_redundant_cells};
use ngk::model::Model;
use ngk::modeling::{faces, solids};
use ngk::topology::gmap::Dim;
use ngk::topology::shape_keys::{EdgeKey, FaceKey};
use ngk::topology::validation::validate_solid_manifold;
use ngk::topology::{ModelEditError, StandardPayload};

use super::seamed::{seam_of, seamed_cylinder_wall, seamed_revolved_sphere, seamed_spherical_cap};

/// Returns the first face of the map together with one of its boundary edges.
fn any_boundary_edge(g: &Model<StandardPayload>) -> (FaceKey, EdgeKey) {
    let face = g.iter_faces().next().expect("map should have a face").0;
    let edge = g
        .face(face)
        .expect("face should be registered")
        .edges()
        .first()
        .expect("face should have edges")
        .key();
    (face, edge)
}

/// Returns a rectangle cut in half, and the edge the two halves share.
fn halved_rectangle() -> (Model<StandardPayload>, EdgeKey) {
    let (mut map, _) = faces::rectangle(Plane::xy(), 2.0, 2.0)
        .expect("rectangle")
        .into_model();
    let face = map.iter_faces().next().expect("map should have a face").0;
    let imprint = FaceImprint::new(
        Curve::line(Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 2.0, 0.0)),
        TrimmedCurve2::segment(Point2::new(1.0, 0.0), Point2::new(1.0, 2.0)),
    );
    split_face_by_imprints(&mut map, face, &[imprint]).expect("a straight imprint should split");
    let shared = map
        .iter_edges()
        .map(|(key, _)| key)
        .find(|&key| map.edge_unchecked(key).faces().len() == 2)
        .expect("the imprint edge is shared by both halves");
    (map, shared)
}

#[test]
fn a_block_corner_is_not_removable() {
    let (map, _) = solids::block(1.0, 1.0, 1.0).expect("block").into_model();
    for (_, attr) in map.iter_vertices() {
        assert!(
            !is_removable(&map, attr.dart, Dim::Zero),
            "a corner joins three edges, so at most two faces cannot bound it"
        );
    }
}

#[test]
fn a_vertex_inserted_by_a_split_is_removable() {
    let (mut map, _) = solids::block(2.0, 2.0, 2.0).expect("block").into_model();
    let (face, edge) = any_boundary_edge(&map);
    let split = split_face_edge(&mut map, face, edge, 0.5).expect("split");

    let dart = map.vertex_attr_unchecked(split.vertex()).dart;
    assert!(is_removable(&map, dart, Dim::Zero));
}

#[test]
fn removing_a_split_vertex_restores_the_original_dart_count() {
    let (mut map, _) = solids::block(2.0, 2.0, 2.0).expect("block").into_model();
    let darts = map.dart_count();
    let (face, edge) = any_boundary_edge(&map);
    let split = split_face_edge(&mut map, face, edge, 0.5).expect("split");
    assert!(map.dart_count() > darts);

    let dart = map.vertex_attr_unchecked(split.vertex()).dart;
    map.transaction(|edit| remove_cell_staged(edit, dart, Dim::Zero))
        .expect("removing the inserted vertex should commit");

    assert_eq!(map.dart_count(), darts);
    assert!(map.vertex_attr(split.vertex()).is_none());
    assert_eq!(map.iter_edges().count(), 12);
    assert_eq!(map.iter_vertices().count(), 8);
    assert_eq!(map.iter_faces().count(), 6);
}

#[test]
fn a_vertex_removal_names_the_two_edges_it_fuses() {
    let (mut map, _) = solids::block(2.0, 2.0, 2.0).expect("block").into_model();
    let (face, edge) = any_boundary_edge(&map);
    let split = split_face_edge(&mut map, face, edge, 0.5).expect("split");
    let dart = map.vertex_attr_unchecked(split.vertex()).dart;

    let removal = map
        .transaction(|edit| remove_cell_staged(edit, dart, Dim::Zero))
        .expect("removing the inserted vertex should commit");

    let MergedCell::Edges { survivor, consumed } = removal.merged else {
        panic!("a 0-removal fuses edges");
    };
    assert!(survivor < consumed, "the lower key must survive");
    let halves: Vec<_> = split.edges().collect();
    assert_eq!(
        [survivor, consumed].map(|key| halves.contains(&key)),
        [true, true],
        "the fused pair must be the two halves of the split edge"
    );
    assert!(map.edge_attr(survivor).is_some());
    assert!(map.edge_attr(consumed).is_none());
}

#[test]
fn removal_translates_every_dart_it_did_not_delete() {
    let (mut map, _) = solids::block(2.0, 2.0, 2.0).expect("block").into_model();
    let (face, edge) = any_boundary_edge(&map);
    let split = split_face_edge(&mut map, face, edge, 0.5).expect("split");
    let dart = map.vertex_attr_unchecked(split.vertex()).dart;
    let before = map.dart_count();

    let removal = map
        .transaction(|edit| remove_cell_staged(edit, dart, Dim::Zero))
        .expect("removing the inserted vertex should commit");

    for removed in &removal.removed {
        assert!(
            removal.remap(*removed).is_none(),
            "a deleted dart has no image"
        );
    }
    let survivors = (0..before)
        .map(ngk::topology::Dart::new)
        .filter(|d| !removal.removed.contains(d))
        .filter(|d| removal.remap(*d).is_some())
        .count();
    assert_eq!(
        survivors,
        before - removal.removed.len(),
        "every surviving dart keeps an image"
    );
}

#[test]
fn removing_a_non_removable_cell_is_rejected_and_rolls_back() {
    let (mut map, _) = solids::block(1.0, 1.0, 1.0).expect("block").into_model();
    let dart = map
        .iter_vertices()
        .next()
        .expect("block should have vertices")
        .1
        .dart;
    let darts = map.dart_count();

    assert!(
        map.transaction(|edit| remove_cell_staged(edit, dart, Dim::Zero))
            .is_err(),
        "a three-edge corner is not removable"
    );
    assert_eq!(map.iter_vertices().count(), 8);
    assert_eq!(map.dart_count(), darts);
}

#[test]
fn removing_a_dimension_three_cell_is_rejected() {
    let (mut map, _) = solids::block(1.0, 1.0, 1.0).expect("block").into_model();
    let dart = map
        .iter_edges()
        .next()
        .expect("block should have edges")
        .1
        .dart;

    assert!(!is_removable(&map, dart, Dim::Three));
    assert!(
        map.transaction(|edit| remove_cell_staged(edit, dart, Dim::Three))
            .is_err()
    );
}

#[test]
fn removing_a_shared_edge_fuses_its_two_faces_and_their_loops() {
    let (mut map, shared) = halved_rectangle();
    assert_eq!(map.iter_faces().count(), 2);
    assert_eq!(map.iter_profiles().count(), 2);
    let dart = map.edge_attr_unchecked(shared).dart;

    let removal = map
        .transaction(|edit| remove_cell_staged(edit, dart, Dim::One))
        .expect("removing the shared edge should commit");

    let MergedCell::Faces {
        survivor, consumed, ..
    } = removal.merged
    else {
        panic!("a 1-removal fuses faces");
    };
    assert!(survivor < consumed, "the lower key must survive");
    assert_eq!(map.iter_faces().count(), 1);
    assert!(map.face_attr(survivor).is_some());
    assert!(map.face_attr(consumed).is_none());
    assert!(map.edge_attr(shared).is_none());
    assert_eq!(
        map.iter_profiles().count(),
        1,
        "the two loops fuse into one"
    );
    assert_eq!(
        map.face(survivor)
            .expect("survivor")
            .outer_loop()
            .expect("face should have an outer loop")
            .edges()
            .len(),
        6,
        "the fused boundary keeps every edge of both halves"
    );
}

#[test]
fn a_shared_edge_is_removable_between_two_free_faces() {
    let (map, shared) = halved_rectangle();
    let dart = map.edge_attr_unchecked(shared).dart;
    assert!(is_removable(&map, dart, Dim::One));
}

/// The tolerance the Boolean's fitted sections actually meet.
///
/// The kernel default is far tighter than anything an intersection engine
/// produces, so healing a Boolean result has to be told the real budget.
const BOOLEAN_TOLERANCE: f64 = 1.0e-7;

/// The union of a block with a cylinder tangent to two of its faces.
///
/// The cylinder's radius equals the block's size, so its circle runs exactly
/// through the block corners `(2, 0, 0)` and `(0, 2, 0)`. At `z = 0` the two
/// operands are coplanar, and splitting tiles that plane with three fragments:
/// the quarter disc both operands cover, the rest of the disc, and the block
/// corner that pokes out. All three describe one plane, so the union has a
/// single bottom face once the redundant topology is gone.
fn tangent_union() -> (Model<StandardPayload>, ngk::topology::shape_keys::SolidKey) {
    let size = 2.0;
    let (mut map, block_key) = solids::block_at(Frame::xyz(), size, size, size)
        .expect("block")
        .into_model();
    let (tool, tool_cylinder) = solids::cylinder_at(Frame::xyz(), size, 2.0 * size)
        .expect("cylinder")
        .into_model();
    let cylinder = map
        .transaction(|edit| {
            let handle = edit.merge(tool.solid_unchecked(tool_cylinder));
            Ok::<_, ModelEditError>(edit.solid_key_at(handle).unwrap())
        })
        .expect("import cylinder");

    let result = boolean(
        &mut map,
        block_key,
        cylinder,
        BooleanOperation::Union,
        BooleanOptions {
            heal: false,
            ..BooleanOptions::default()
        },
    )
    .expect("the tangent union should succeed");
    (map, result.solid)
}

/// Returns the solid's faces lying in the `z = 0` plane.
///
/// Selected by the support rather than by the corners: a face bounded by a
/// whole circle has no corners at all, and "every corner sits at z = 0" is
/// vacuously true of such a face wherever it actually is. The cylinder's far
/// cap is exactly that face, and answering by its own plane keeps it out.
fn bottom_faces(
    g: &Model<StandardPayload>,
    solid: ngk::topology::shape_keys::SolidKey,
) -> Vec<FaceKey> {
    g.solid_unchecked(solid)
        .faces()
        .iter()
        .filter(|face| match face.surface() {
            Surface::Plane(plane) => {
                plane.origin().z.abs() <= 1.0e-9 && plane.normal().z.abs() >= 1.0 - 1.0e-9
            }
            _ => false,
        })
        .map(|face| face.key())
        .collect()
}

#[test]
fn redundant_faces_of_boolean_fuse_are_deleted() {
    let (mut map, solid) = tangent_union();
    assert_eq!(
        bottom_faces(&map, solid).len(),
        3,
        "splitting leaves the bottom plane tiled by three fragments"
    );

    let report = remove_redundant_cells(
        &mut map,
        HealingOptions {
            scope: HealingScope::Solid(solid),
            linear_tolerance: BOOLEAN_TOLERANCE,
            angular_tolerance: BOOLEAN_TOLERANCE,
            ..HealingOptions::default()
        },
    )
    .expect("healing the union should succeed");

    let bottom = bottom_faces(&map, solid);
    assert_eq!(
        bottom.len(),
        1,
        "the three coplanar fragments describe one bottom face; skips were {:?}",
        report.skipped
    );

    // That single face is the disc plus the block corner: the major arc from
    // (0,2,0) round to (2,0,0), then the two square edges past (2,2,0).
    let face = map.face(bottom[0]).expect("the fused bottom face");
    assert!(
        face.inner_loops().is_empty(),
        "the fused bottom face has no holes"
    );
    assert_eq!(
        face.outer_loop()
            .expect("face should have an outer loop")
            .edges()
            .len(),
        3,
        "the fused boundary is the major arc and the two block edges"
    );

    // The two square edges through the origin were interior to the fused face,
    // so the corner they met at has nothing left to bound.
    assert!(
        !map.iter_vertices()
            .any(|(_, attr)| (attr.point - Point3::origin()).norm() <= BOOLEAN_TOLERANCE),
        "the vertex at the origin becomes isolated and must go with its edges"
    );
}

fn planar_imprint(pcurve: TrimmedCurve2) -> FaceImprint {
    let points = pcurve
        .sample(32)
        .into_iter()
        .map(|point| Point3::new(point.x, point.y, 0.0))
        .collect::<Vec<_>>();
    let curve = match pcurve.curve() {
        Curve2::Line(_) => Curve::line(points[0], *points.last().unwrap()),
        Curve2::Circle(_) | Curve2::Ellipse(_) | Curve2::Nurbs(_) => Curve::Nurbs(
            ngk::geometry::NurbsCurve::interpolate(&points)
                .expect("sampled planar pcurve should interpolate in 3D"),
        ),
    };
    FaceImprint::new(curve, pcurve)
}
/// Returns a rectangle partitioned by a closed square imprint.
fn rectangle_with_filled_inner_loop() -> (Model<StandardPayload>, FaceKey) {
    let mut g = Model::<StandardPayload>::new();
    let face = add_rectangle(&mut g, Plane::xy(), 4.0, 4.0).unwrap();
    let points = [
        Point2::new(1.0, 1.0),
        Point2::new(3.0, 1.0),
        Point2::new(3.0, 3.0),
        Point2::new(1.0, 3.0),
        Point2::new(1.0, 1.0),
    ];
    let imprints = points
        .windows(2)
        .map(|pair| planar_imprint(TrimmedCurve2::segment(pair[0], pair[1])))
        .collect::<Vec<_>>();
    let splits = split_face_by_imprints(&mut g, face, &imprints).unwrap();
    assert_eq!(splits.len(), 1, "the closed imprint creates one island");
    assert_eq!(g.iter_faces().count(), 2);
    assert_eq!(g.face_unchecked(face).inner_loops().len(), 1);
    (g, face)
}

#[test]
fn imprinted_face_inner_loop_gets_removed() {
    let (mut g, _) = rectangle_with_filled_inner_loop();

    let result = remove_redundant_cells(&mut g, HealingOptions::default()).unwrap();
    assert_eq!(
        g.iter_faces().count(),
        1,
        "the island must fuse into its surrounding face; skips were {:?}",
        result.skipped
    );
    let healed = g.iter_faces().next().unwrap().0;
    assert!(
        g.face_unchecked(healed).inner_loops().is_empty(),
        "the filled inner loop must disappear"
    );
    assert_eq!(g.iter_edges().count(), 4, "only the rectangle remains");
    assert_eq!(result.fused_faces.len(), 1);
}

#[test]
fn filled_inner_loop_removal_can_be_disabled() {
    let (mut g, face) = rectangle_with_filled_inner_loop();

    let result = remove_redundant_cells(
        &mut g,
        HealingOptions {
            remove_filled_inner_loops: false,
            ..HealingOptions::default()
        },
    )
    .unwrap();

    assert_eq!(g.iter_faces().count(), 2);
    assert_eq!(g.face_unchecked(face).inner_loops().len(), 1);
    assert_eq!(g.iter_edges().count(), 8);
    assert!(result.fused_faces.is_empty());
}

#[test]
fn single_edge_filled_inner_loop_gets_removed() {
    let mut g = Model::<StandardPayload>::new();
    let face = add_rectangle(&mut g, Plane::xy(), 4.0, 4.0).unwrap();
    let circle = TrimmedCurve2::arc(
        Point2::new(2.0, 2.0),
        Vector2::x(),
        1.0,
        std::f64::consts::TAU,
    );
    let splits = split_face_by_imprints(&mut g, face, &[planar_imprint(circle)]).unwrap();
    assert_eq!(splits.len(), 1);
    assert_eq!(g.iter_edges().count(), 5);

    let result = remove_redundant_cells(&mut g, HealingOptions::default()).unwrap();
    assert_eq!(g.iter_faces().count(), 1);
    let healed = g.iter_faces().next().unwrap().0;
    assert!(g.face_unchecked(healed).inner_loops().is_empty());
    assert_eq!(g.iter_edges().count(), 4);
    assert_eq!(result.fused_faces.len(), 1);
}

#[test]
fn removing_a_seam_leaves_the_face_a_ring() {
    let (mut g, wall) = seamed_cylinder_wall(1.0, 2.0);
    let (seam, dart) = seam_of(&g, wall).expect("the wall was built with a seam");
    assert_eq!(g.iter_edges().count(), 3, "two circles and the seam");

    let removal = remove_cell(&mut g, dart, Dim::One).expect("a seam should be removable");

    let MergedCell::Ring {
        face,
        survivor_loop,
        added_loop,
    } = removal.merged
    else {
        panic!("removing a seam leaves a ring, got {:?}", removal.merged);
    };
    assert_eq!(face, wall);
    assert_ne!(survivor_loop, added_loop);
    assert!(g.edge_attr(seam).is_none(), "the seam itself is gone");

    let face = g.face_unchecked(wall);
    assert_eq!(
        face.loops().len(),
        2,
        "the seam was hiding two loops, not one"
    );
    assert!(
        face.outer_loop().is_none(),
        "neither loop bounds a ring from outside"
    );
    assert_eq!(
        face.loops()
            .into_iter()
            .filter_map(|loop_| loop_.wrapping_axis())
            .collect::<Vec<_>>(),
        vec![Axis2::U, Axis2::U],
        "both loops span the closed direction"
    );
}

/// Healing is the canonicalizer: a seamed model in, a seamless model out.
#[test]
fn healing_removes_the_seam_of_an_imported_wall() {
    let (mut g, wall) = seamed_cylinder_wall(1.0, 2.0);

    remove_redundant_cells(&mut g, HealingOptions::default()).expect("healing should commit");

    assert_eq!(
        g.iter_edges().count(),
        2,
        "the seam is gone and both circles remain"
    );
    let face = g.face_unchecked(wall);
    assert_eq!(face.loops().len(), 2);
    assert!(face.outer_loop().is_none());
}

/// A seam whose removal leaves no loop at all leaves a boundaryless face.
///
/// A meridian revolved a whole turn is a sphere written the way an import
/// carries one: its whole boundary is the seam, walked up one side and down the
/// other between two pole vertices. Take the seam away and there is nothing
/// left for a boundary to be — which is exactly what `solids::sphere` builds
/// directly. The shell has no dart to be rooted at either, so it re-roots at
/// the face, the dart-preferred invariant read the other way round.
#[test]
fn removing_a_seam_can_leave_the_face_boundaryless() {
    let (mut g, sphere, solid) = seamed_revolved_sphere(1.0);
    let (seam, dart) = seam_of(&g, sphere).expect("a revolved meridian carries a seam");
    assert_eq!(g.iter_edges().count(), 1, "the meridian is the only edge");

    let removal = remove_cell(&mut g, dart, Dim::One).expect("a seam should be removable");

    let MergedCell::Unbounded { face, .. } = removal.merged else {
        panic!(
            "removing a sphere's whole boundary leaves it unbounded, got {:?}",
            removal.merged
        );
    };
    assert_eq!(face, sphere);
    assert!(g.edge_attr(seam).is_none(), "the seam itself is gone");

    assert_eq!(
        (
            g.dart_count(),
            g.iter_vertices().count(),
            g.iter_edges().count(),
            g.iter_faces().count()
        ),
        (0, 0, 0, 1),
        "what is left is one face covering its whole support"
    );
    let face = g.face_unchecked(sphere);
    assert!(face.loops().is_empty(), "a boundaryless face has no loops");
    assert!(face.dart().is_none());
    assert_eq!(
        g.solid_attr_unchecked(solid).outer_shell.face(),
        Some(sphere),
        "the shell re-roots at the face, there being no dart left to root at"
    );
    validate_solid_manifold(&g, solid).expect("the healed sphere should still be well formed");
}

/// The same seam, taken apart by healing rather than by hand.
#[test]
fn healing_removes_the_seam_of_an_imported_sphere() {
    let (mut g, sphere, solid) = seamed_revolved_sphere(1.0);

    remove_redundant_cells(&mut g, HealingOptions::default()).expect("healing should commit");

    assert_eq!(
        g.dart_count(),
        0,
        "a seamed sphere heals into a seamless one"
    );
    assert!(g.face_unchecked(sphere).loops().is_empty());
    assert_eq!(
        g.solid_attr_unchecked(solid).outer_shell.face(),
        Some(sphere)
    );
    validate_solid_manifold(&g, solid).expect("the healed sphere should still be well formed");
}

/// Only a closed support can carry a face with no boundary.
///
/// A disk's rim is its whole boundary, so removing it would leave the face
/// covering its whole domain — the entire plane. The support is what decides,
/// and a plane closes in neither direction, so the removal declines instead of
/// producing a face nothing bounds.
#[test]
fn removing_the_only_boundary_of_an_open_support_is_refused() {
    let shape = faces::circle(Plane::xy(), 1.0).expect("a circular face should build");
    let (mut g, disk) = shape.into_model();
    let rim = g
        .face(disk)
        .expect("the disk is registered")
        .edges()
        .first()
        .expect("a disk has a rim")
        .dart();

    assert!(
        matches!(
            remove_cell(&mut g, rim, Dim::One),
            Err(CellRemovalError::WouldUnboundFace { .. })
        ),
        "a plane cannot bound a face with no loops"
    );
}

/// A seam whose removal leaves one loop rather than two leaves a cap.
///
/// The far side of a spherical cap is closed by the pole, not by a boundary, so
/// there is no second loop for the removal to fall into. Which pole closes it
/// follows from the loop's own travel: the face lies to the left of its
/// boundary, and this one runs forward along `u`, so the face is the part above
/// the latitude and the degeneracy that closes it is the domain's high end.
#[test]
fn removing_a_seam_leaves_the_face_a_cap() {
    let (mut g, cap) = seamed_spherical_cap(1.0, 0.5);
    let (seam, dart) = seam_of(&g, cap).expect("the cap was built with a seam");
    assert_eq!(g.iter_edges().count(), 2, "one latitude and the seam");

    let removal = remove_cell(&mut g, dart, Dim::One).expect("a seam should be removable");

    let MergedCell::Cap {
        face,
        side,
        survivor_loop: _,
    } = removal.merged
    else {
        panic!("removing this seam leaves a cap, got {:?}", removal.merged);
    };
    assert_eq!(face, cap);
    assert_eq!(side, DomainSide::High, "the north pole closes it");
    assert!(g.edge_attr(seam).is_none(), "the seam itself is gone");

    let face = g.face_unchecked(cap);
    assert_eq!(face.loops().len(), 1, "one latitude circle bounds it");
    assert!(
        face.outer_loop().is_none(),
        "a period-spanning loop bounds no inside"
    );
    let kind = face.loops()[0].kind();
    assert_eq!(kind.wrapped_axis(), Some(Axis2::U));
    assert_eq!(kind.capped_side(), Some(DomainSide::High));
    assert_eq!(
        g.iter_vertices().count(),
        1,
        "the pole the seam ran up to is gone; the latitude keeps its own vertex"
    );
}

/// Healing takes an imported cap apart the same way it takes a wall apart.
#[test]
fn healing_removes_the_seam_of_an_imported_cap() {
    let (mut g, cap) = seamed_spherical_cap(1.0, 0.5);

    remove_redundant_cells(&mut g, HealingOptions::default()).expect("healing should commit");

    assert_eq!(g.iter_edges().count(), 1, "only the latitude remains");
    let face = g.face_unchecked(cap);
    assert_eq!(face.loops().len(), 1);
    assert_eq!(face.loops()[0].kind().capped_side(), Some(DomainSide::High));
}
