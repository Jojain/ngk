use ngk::builders::faces::{FaceImprint, split_face_by_imprints};
use ngk::builders::profiles::plane_uv;
use ngk::geometry::{Curve, Curve2, Line2, Plane, Point3, Surface};
use ngk::healing::{HealingOptions, remove_redundant_cells};
use ngk::modeling::{faces, solids};
use ngk::tessellate::TessellateOpts;
use ngk::tessellate::face::tessellate_face_key;
use ngk::topology::StandardPayload;
use ngk::topology::gmap::GMap;
use ngk::topology::shape_keys::FaceKey;

/// Returns the face whose vertices all sit at `height`.
fn face_at_height(g: &GMap<StandardPayload>, height: f64) -> FaceKey {
    g.iter_faces()
        .map(|(key, _)| key)
        .find(|&key| {
            g.face(key)
                .expect("face should be registered")
                .vertices()
                .iter()
                .all(|vertex| {
                    vertex
                        .point()
                        .is_some_and(|point| (point.z - height).abs() <= 1.0e-9)
                })
        })
        .expect("a face at the requested height should exist")
}

/// Returns the plane a face sits on.
fn face_plane(g: &GMap<StandardPayload>, face: FaceKey) -> Plane {
    match &g.face_attr_unchecked(face).surface {
        Surface::Plane(plane) => plane.clone(),
        other => panic!(
            "expected a planar face, got {:?}",
            std::mem::discriminant(other)
        ),
    }
}

/// Cuts `face` in two along the segment from `start` to `end`.
fn imprint_segment(
    g: &mut GMap<StandardPayload>,
    face: FaceKey,
    start: Point3,
    end: Point3,
) -> usize {
    let plane = face_plane(g, face);
    let imprint = FaceImprint::new(
        Curve::line(start, end),
        Curve2::Line(Line2::new(plane_uv(&plane, start), plane_uv(&plane, end))),
    );
    split_face_by_imprints(g, face, &[imprint])
        .expect("a straight imprint across a face should split it")
        .len()
}

#[test]
fn coplanar_faces_sharing_an_edge_fuse_into_one_face() {
    let (mut map, _) = faces::rectangle(Plane::xy(), 2.0, 2.0)
        .expect("rectangle")
        .into_map();
    let face = map.iter_faces().next().expect("map should have a face").0;
    assert_eq!(
        imprint_segment(
            &mut map,
            face,
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 2.0, 0.0),
        ),
        1
    );
    assert_eq!(map.iter_faces().count(), 2);

    let report = remove_redundant_cells(&mut map, HealingOptions::default())
        .expect("healing should succeed");

    assert_eq!(
        map.iter_faces().count(),
        1,
        "the two halves sit on one plane, skips were {:?}",
        report.skipped
    );
    assert_eq!(report.fused_faces.len(), 1);
    assert_eq!(map.iter_edges().count(), 4);
    assert_eq!(map.iter_vertices().count(), 4);
}

#[test]
fn imprinting_and_healing_a_block_face_restores_the_block() {
    let (mut map, _) = solids::block(2.0, 2.0, 2.0).expect("block").into_map();
    let base = face_at_height(&map, 0.0);
    imprint_segment(
        &mut map,
        base,
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(1.0, 2.0, 0.0),
    );
    assert_eq!(map.iter_faces().count(), 7);

    let report = remove_redundant_cells(&mut map, HealingOptions::default())
        .expect("healing should succeed");

    assert_eq!(
        (
            map.iter_faces().count(),
            map.iter_edges().count(),
            map.iter_vertices().count(),
        ),
        (6, 12, 8),
        "healing should restore the block's cell counts, skips were {:?}",
        report.skipped
    );
    for (key, _) in map.iter_faces() {
        let mesh = tessellate_face_key(&map, key, TessellateOpts::default())
            .expect("every healed face should tessellate");
        assert!(
            !mesh.positions.is_empty(),
            "face {key:?} should emit vertices"
        );
    }
}

#[test]
fn healing_an_imprinted_block_preserves_its_shell_euler_characteristic() {
    let (mut map, solid) = solids::block(2.0, 2.0, 2.0).expect("block").into_map();
    let euler = |g: &GMap<StandardPayload>| {
        let shell = &g.solid_unchecked(solid).shells()[0];
        shell.vertices().len() as isize - shell.edges().len() as isize
            + shell.faces().len() as isize
    };
    let before = euler(&map);

    let base = face_at_height(&map, 0.0);
    imprint_segment(
        &mut map,
        base,
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(1.0, 2.0, 0.0),
    );
    remove_redundant_cells(&mut map, HealingOptions::default()).expect("healing should succeed");

    assert_eq!(euler(&map), before);
}

#[test]
fn perpendicular_faces_of_a_block_are_not_fused() {
    let (mut map, _) = solids::block(1.0, 2.0, 3.0).expect("block").into_map();
    let report = remove_redundant_cells(&mut map, HealingOptions::default())
        .expect("healing should succeed");

    assert!(report.fused_faces.is_empty());
    assert_eq!(map.iter_faces().count(), 6);
}

#[test]
fn a_cylinder_seam_edge_is_preserved() {
    let (mut map, _) = solids::cylinder(1.0, 2.0).expect("cylinder").into_map();
    let faces = map.iter_faces().count();
    let edges = map.iter_edges().count();

    let report = remove_redundant_cells(&mut map, HealingOptions::default())
        .expect("healing should succeed");

    assert!(
        report.fused_faces.is_empty(),
        "the lateral face bounds its seam on both sides and must keep it"
    );
    assert_eq!(map.iter_faces().count(), faces);
    assert_eq!(map.iter_edges().count(), edges);
}

#[test]
fn healing_is_idempotent() {
    let (mut map, _) = solids::block(2.0, 2.0, 2.0).expect("block").into_map();
    let base = face_at_height(&map, 0.0);
    imprint_segment(
        &mut map,
        base,
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(1.0, 2.0, 0.0),
    );

    remove_redundant_cells(&mut map, HealingOptions::default()).expect("first run should succeed");
    let second = remove_redundant_cells(&mut map, HealingOptions::default())
        .expect("second run should succeed");

    assert!(
        second.is_empty(),
        "a healed map has nothing left to remove, but got {second:?}"
    );
}
