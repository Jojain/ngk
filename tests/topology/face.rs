use ngk::builders::faces::{add_annulus, add_circle};
use ngk::geometry::{LINEAR_TOLERANCE, Plane, Point3, PointCoincidence};
use ngk::modeling::{faces, solids};
use ngk::topology::face::Face;
use ngk::topology::gmap::{Dim, GMap};
use ngk::topology::payload::StandardPayload;

#[test]
fn face_point_at_evaluates_its_support_surface() {
    let shape = faces::rectangle(Plane::xy(), 2.0, 3.0).expect("face should build");
    let point = shape.face().point_at(0.5, 1.25);

    assert!(point.coincides(&Point3::new(0.5, 1.25, 0.0), LINEAR_TOLERANCE));
}

#[test]
fn face_point_at_is_defined_inside_a_trimmed_hole() {
    let shape = faces::annulus(Plane::xy(), 2.0, 1.0).expect("face should build");
    let point = shape.face().point_at(0.0, 0.0);

    assert!(point.coincides(&Point3::origin(), LINEAR_TOLERANCE));
}

#[test]
fn face_views_from_opposite_darts_reverse_boundary_and_normal() {
    let mut g = GMap::<StandardPayload>::new();
    let face_key = add_circle(&mut g, Plane::xy(), 1.0).expect("circle face should build");
    let default_dart = g.face_attr_unchecked(face_key).outer_loop;
    let reversed_dart = g.alpha(Dim::Zero, default_dart);
    let default_face = Face::from_dart(&g, default_dart).expect("face should resolve");
    let reversed_face = Face::from_dart(&g, reversed_dart).expect("face should resolve");

    assert_eq!(default_face.key(), face_key);
    assert_eq!(reversed_face.key(), face_key);
    assert_eq!(default_face.outer_loop().dart, default_dart);
    assert_eq!(reversed_face.outer_loop().dart, reversed_dart);
    assert!(
        default_face
            .normal_at(0.0, 0.0)
            .dot(&reversed_face.normal_at(0.0, 0.0))
            < -1.0 + LINEAR_TOLERANCE,
        "face views built from opposite darts should have opposite normals"
    );
}

#[test]
fn face_views_from_stored_loop_seeds_share_the_same_normal() {
    let mut g = GMap::<StandardPayload>::new();
    let face_key = add_annulus(&mut g, Plane::xy(), 2.0, 1.0).expect("annulus face should build");
    let attr = g.face_attr_unchecked(face_key);
    let outer = Face::from_dart(&g, attr.outer_loop).expect("outer loop should resolve its face");
    let inner =
        Face::from_dart(&g, attr.inner_loops[0]).expect("inner loop should resolve its face");

    assert!(
        outer.normal_at(0.0, 0.0).dot(&inner.normal_at(0.0, 0.0)) > 1.0 - LINEAR_TOLERANCE,
        "all stored loop seeds should produce the same geometric face orientation"
    );
}

#[test]
fn face_boundary_edges_preserve_their_exact_loop_darts() {
    let shape = solids::block(1.0, 2.0, 3.0).expect("block should build");

    for face in shape.solid().faces() {
        let loop_darts = face.outer_loop().darts().step_by(2).collect::<Vec<_>>();
        let edges = face.outer_loop().edges();

        assert_eq!(edges.len(), loop_darts.len());
        for (edge, loop_dart) in edges.iter().zip(loop_darts) {
            assert_eq!(
                edge.dart(),
                loop_dart,
                "face {:?} edge {:?} should retain the exact dart discovered by its boundary traversal",
                face.key(),
                edge.key()
            );
        }
    }
}

#[test]
fn block_face_pcurves_follow_oriented_boundary_edges() {
    let shape = solids::block(1.0, 2.0, 3.0).expect("block should build");

    for face in shape.solid().faces() {
        for edge in face.outer_loop().edges() {
            let pcurve = face
                .pcurve(edge.dart())
                .expect("each block boundary edge should have a pcurve");
            let start_uv = pcurve.point_at(0.0);
            let end_uv = pcurve.point_at(1.0);
            let pcurve_start = face.point_at(start_uv.x, start_uv.y);
            let pcurve_end = face.point_at(end_uv.x, end_uv.y);
            let edge_start = *edge
                .start()
                .point()
                .expect("block edge start should have geometry");
            let edge_end = *edge
                .end()
                .point()
                .expect("block edge end should have geometry");

            assert!(
                pcurve_start.coincides(edge_start, LINEAR_TOLERANCE),
                "face {:?} edge {:?} pcurve should start at its oriented edge start: {pcurve_start:?} != {edge_start:?}",
                face.key(),
                edge.key()
            );
            assert!(
                pcurve_end.coincides(edge_end, LINEAR_TOLERANCE),
                "face {:?} edge {:?} pcurve should end at its oriented edge end: {pcurve_end:?} != {edge_end:?}",
                face.key(),
                edge.key()
            );
        }
    }
}
