use nalgebra::Vector2;
use ngk::builders::boolean::{BooleanOperation, BooleanOptions, boolean};
use ngk::builders::faces::{FaceImprint, add_rectangle, split_face_by_imprints, split_face_edge};
use ngk::builders::removal::{MergedCell, is_removable, remove_cell_staged};
use ngk::geometry::{Circle2, Curve, Curve2, Frame, Line2, Plane, Point2, Point3, Surface};
use ngk::healing::{HealingOptions, HealingScope, remove_redundant_cells};
use ngk::modeling::{faces, solids};
use ngk::topology::gmap::{Dim, GMap};
use ngk::topology::shape_keys::{EdgeKey, FaceKey};
use ngk::topology::{StandardPayload, TopologyEditError};

/// Returns the first face of the map together with one of its boundary edges.
fn any_boundary_edge(g: &GMap<StandardPayload>) -> (FaceKey, EdgeKey) {
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
fn halved_rectangle() -> (GMap<StandardPayload>, EdgeKey) {
    let (mut map, _) = faces::rectangle(Plane::xy(), 2.0, 2.0)
        .expect("rectangle")
        .into_map();
    let face = map.iter_faces().next().expect("map should have a face").0;
    let imprint = FaceImprint::new(
        Curve::line(Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 2.0, 0.0)),
        Curve2::Line(Line2::new(Point2::new(1.0, 0.0), Point2::new(1.0, 2.0))),
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
    let (map, _) = solids::block(1.0, 1.0, 1.0).expect("block").into_map();
    for (_, attr) in map.iter_vertices() {
        assert!(
            !is_removable(&map, attr.dart, Dim::Zero),
            "a corner joins three edges, so at most two faces cannot bound it"
        );
    }
}

#[test]
fn a_vertex_inserted_by_a_split_is_removable() {
    let (mut map, _) = solids::block(2.0, 2.0, 2.0).expect("block").into_map();
    let (face, edge) = any_boundary_edge(&map);
    let split = split_face_edge(&mut map, face, edge, 0.5).expect("split");

    let dart = map.vertex_attr_unchecked(split.vertex).dart;
    assert!(is_removable(&map, dart, Dim::Zero));
}

#[test]
fn removing_a_split_vertex_restores_the_original_dart_count() {
    let (mut map, _) = solids::block(2.0, 2.0, 2.0).expect("block").into_map();
    let darts = map.dart_count();
    let (face, edge) = any_boundary_edge(&map);
    let split = split_face_edge(&mut map, face, edge, 0.5).expect("split");
    assert!(map.dart_count() > darts);

    let dart = map.vertex_attr_unchecked(split.vertex).dart;
    map.transaction(|edit| remove_cell_staged(edit, dart, Dim::Zero))
        .expect("removing the inserted vertex should commit");

    assert_eq!(map.dart_count(), darts);
    assert!(map.vertex_attr(split.vertex).is_none());
    assert_eq!(map.iter_edges().count(), 12);
    assert_eq!(map.iter_vertices().count(), 8);
    assert_eq!(map.iter_faces().count(), 6);
}

#[test]
fn a_vertex_removal_names_the_two_edges_it_fuses() {
    let (mut map, _) = solids::block(2.0, 2.0, 2.0).expect("block").into_map();
    let (face, edge) = any_boundary_edge(&map);
    let split = split_face_edge(&mut map, face, edge, 0.5).expect("split");
    let dart = map.vertex_attr_unchecked(split.vertex).dart;

    let removal = map
        .transaction(|edit| remove_cell_staged(edit, dart, Dim::Zero))
        .expect("removing the inserted vertex should commit");

    let MergedCell::Edges { survivor, consumed } = removal.merged else {
        panic!("a 0-removal fuses edges");
    };
    assert!(survivor < consumed, "the lower key must survive");
    assert_eq!(
        [survivor, consumed].map(|key| [split.first, split.second].contains(&key)),
        [true, true],
        "the fused pair must be the two halves of the split edge"
    );
    assert!(map.edge_attr(survivor).is_some());
    assert!(map.edge_attr(consumed).is_none());
}

#[test]
fn removal_translates_every_dart_it_did_not_delete() {
    let (mut map, _) = solids::block(2.0, 2.0, 2.0).expect("block").into_map();
    let (face, edge) = any_boundary_edge(&map);
    let split = split_face_edge(&mut map, face, edge, 0.5).expect("split");
    let dart = map.vertex_attr_unchecked(split.vertex).dart;
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
    let (mut map, _) = solids::block(1.0, 1.0, 1.0).expect("block").into_map();
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
    let (mut map, _) = solids::block(1.0, 1.0, 1.0).expect("block").into_map();
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
fn tangent_union() -> (GMap<StandardPayload>, ngk::topology::shape_keys::SolidKey) {
    let size = 2.0;
    let (mut map, block_key) = solids::block_at(Frame::xyz(), size, size, size)
        .expect("block")
        .into_map();
    let (tool, tool_cylinder) = solids::cylinder_at(Frame::xyz(), size, 2.0 * size)
        .expect("cylinder")
        .into_map();
    let cylinder = map
        .transaction(|edit| {
            let dart = edit.merge(tool.solid_unchecked(tool_cylinder));
            Ok::<_, TopologyEditError>(edit.solid_key(dart).unwrap())
        })
        .expect("import cylinder");

    let result = boolean(
        &mut map,
        block_key,
        cylinder,
        BooleanOperation::Union,
        BooleanOptions::default(),
    )
    .expect("the tangent union should succeed");
    (map, result.solid)
}

/// Returns the solid's planar faces whose vertices all sit at `z = 0`.
fn bottom_faces(
    g: &GMap<StandardPayload>,
    solid: ngk::topology::shape_keys::SolidKey,
) -> Vec<FaceKey> {
    g.solid_unchecked(solid)
        .faces()
        .iter()
        .filter(|face| matches!(face.surface(), Surface::Plane(_)))
        .filter(|face| {
            face.vertices()
                .iter()
                .all(|vertex| vertex.point().is_some_and(|point| point.z.abs() <= 1.0e-9))
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
        face.outer_loop().edges().len(),
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

fn planar_imprint(pcurve: Curve2) -> FaceImprint {
    let points = pcurve
        .sample(32)
        .into_iter()
        .map(|point| Point3::new(point.x, point.y, 0.0))
        .collect::<Vec<_>>();
    let curve = match &pcurve {
        Curve2::Line(_) => Curve::line(points[0], *points.last().unwrap()),
        Curve2::Circle(_) | Curve2::Nurbs(_) => Curve::Nurbs(
            ngk::geometry::NurbsCurve::interpolate(&points)
                .expect("sampled planar pcurve should interpolate in 3D"),
        ),
    };
    FaceImprint::new(curve, pcurve)
}
/// Returns a rectangle partitioned by a closed square imprint.
fn rectangle_with_filled_inner_loop() -> (GMap<StandardPayload>, FaceKey) {
    let mut g = GMap::<StandardPayload>::new();
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
        .map(|pair| planar_imprint(Curve2::Line(Line2::new(pair[0], pair[1]))))
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
    let mut g = GMap::<StandardPayload>::new();
    let face = add_rectangle(&mut g, Plane::xy(), 4.0, 4.0).unwrap();
    let circle = Curve2::Circle(Circle2::new(
        Point2::new(2.0, 2.0),
        Vector2::x(),
        1.0,
        std::f64::consts::TAU,
    ));
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
