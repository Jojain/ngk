use ngk::builders::faces::{add_annulus, add_circle};
use ngk::geometry::{Axis2, LINEAR_TOLERANCE, Plane, Point3, PointCoincidence};
use ngk::modeling::{faces, solids};
use ngk::topology::face::Face;
use ngk::topology::gmap::{Dart, Dim, GMap};
use ngk::topology::payload::StandardPayload;
use ngk::topology::{LoopDefinition, LoopKind, Orientation};

#[test]
fn loop_definitions_make_their_kind_explicit() {
    let seed = Dart::new(42);

    assert_eq!(LoopDefinition::outer(seed).kind(), LoopKind::Outer);
    assert_eq!(LoopDefinition::inner(seed).kind(), LoopKind::Inner);
    assert_eq!(
        LoopDefinition::wrapping(seed, Axis2::U).kind(),
        LoopKind::Wrapping { axis: Axis2::U },
    );
}

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
    let default_dart = g.face_unchecked(face_key).dart_unchecked();
    let reversed_dart = g.alpha(Dim::Zero, default_dart);
    let default_face = Face::from_dart(&g, default_dart).expect("face should resolve");
    let reversed_face = Face::from_dart(&g, reversed_dart).expect("face should resolve");

    assert_eq!(default_face.key(), face_key);
    assert_eq!(reversed_face.key(), face_key);
    assert_eq!(
        default_face
            .outer_loop()
            .expect("face should have an outer loop")
            .dart,
        default_dart
    );
    assert_eq!(
        reversed_face
            .outer_loop()
            .expect("face should have an outer loop")
            .dart,
        reversed_dart
    );
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
    let face = g.face_unchecked(face_key);
    let outer = Face::from_dart(
        &g,
        face.outer_loop()
            .expect("face should have an outer loop")
            .dart,
    )
    .expect("outer loop should resolve its face");
    let inner = Face::from_dart(&g, face.inner_loops()[0].dart)
        .expect("inner loop should resolve its face");

    assert!(
        outer.normal_at(0.0, 0.0).dot(&inner.normal_at(0.0, 0.0)) > 1.0 - LINEAR_TOLERANCE,
        "all stored loop seeds should produce the same geometric face orientation"
    );
}

#[test]
fn face_boundary_edges_preserve_their_exact_loop_darts() {
    let shape = solids::block(1.0, 2.0, 3.0).expect("block should build");

    for face in shape.solid().faces() {
        let loop_darts = face
            .outer_loop()
            .expect("face should have an outer loop")
            .darts()
            .step_by(2)
            .collect::<Vec<_>>();
        let edges = face
            .outer_loop()
            .expect("face should have an outer loop")
            .edges();

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
fn face_loops_carry_their_domain_kind_and_contextual_edges() {
    let shape = solids::block(1.0, 2.0, 3.0).expect("block should build");

    for face in shape.solid().faces() {
        let loops = face.loops();
        assert_eq!(loops.len(), 1);
        assert_eq!(loops[0].kind(), LoopKind::Outer);
        assert_eq!(loops[0].edges().len(), 4);
    }
}

#[test]
fn block_face_pcurves_follow_oriented_boundary_edges() {
    let shape = solids::block(1.0, 2.0, 3.0).expect("block should build");

    for face in shape.solid().faces() {
        for edge in face
            .outer_loop()
            .expect("face should have an outer loop")
            .edges()
        {
            let pcurve = face
                .pcurve(edge.dart())
                .expect("each block boundary edge should have a pcurve");
            let start_uv = pcurve.point_at(0.0);
            let end_uv = pcurve.point_at(1.0);
            let pcurve_start = face.point_at(start_uv.x, start_uv.y);
            let pcurve_end = face.point_at(end_uv.x, end_uv.y);
            let edge_start = *edge
                .bounded_unchecked()
                .start()
                .point()
                .expect("block edge start should have geometry");
            let edge_end = *edge
                .bounded_unchecked()
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

/// The view carries a sense, not a locator: every stored loop seed of a face
/// names the same view, and the dart it hands back re-resolves to it.
#[test]
fn a_face_view_is_named_by_its_sense_not_by_the_dart_it_was_reached_from() {
    let mut g = GMap::<StandardPayload>::new();
    let face_key = add_annulus(&mut g, Plane::xy(), 2.0, 1.0).expect("annulus face should build");
    let face = g.face_unchecked(face_key);
    let inner_seed = g.face_unchecked(face_key).inner_loops()[0].dart;
    let from_inner = Face::from_dart(&g, inner_seed).expect("inner seed should resolve its face");

    assert_eq!(face.sense(), Orientation::Same);
    assert_eq!(from_inner.sense(), Orientation::Same);
    assert_eq!(from_inner.dart(), face.dart());

    let reversed = face.reversed();
    let round_tripped =
        Face::from_dart(&g, reversed.dart_unchecked()).expect("dart should resolve its face");

    assert_eq!(reversed.sense(), Orientation::Reversed);
    assert_eq!(round_tripped.sense(), Orientation::Reversed);
    assert_eq!(
        round_tripped
            .outer_loop()
            .expect("face should have an outer loop")
            .dart,
        reversed
            .outer_loop()
            .expect("face should have an outer loop")
            .dart
    );
    assert_eq!(reversed.reversed().sense(), Orientation::Same);
}
