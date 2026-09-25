use std::collections::HashSet;
use std::f64::consts::PI;

use nalgebra::Vector3;
use ngk::builders::blend::BlendError;
use ngk::builders::chamfer::chamfer;
use ngk::builders::edges::add_edge;
use ngk::builders::faces::add_face;
use ngk::builders::profiles::{add_polyline, append_edge};
use ngk::builders::solids::add_extruded_face;
use ngk::geometry::{Curve, NurbsCurve, Point3, Surface};
use ngk::model::Model;
use ngk::modeling::solids::block;
use ngk::topology::StandardPayload;
use ngk::topology::shape_keys::SolidKey;
use ngk::topology::validation::{validate_solid_manifold, validate_solid_orientation};

use super::blend_shapes::{
    L_SHAPE, boss_on_block, edge_between, face_at_height, lens, oblique_boss_on_block,
    plate_with_bore, prism, prism_with_hole, slot_prism,
};

const TOLERANCE: f64 = 1.0e-9;

/// How far a skinned blend may stray from the sections it was solved from:
/// the share of the fitting tolerance the skin is held to, with room for
/// evaluation.
const SKIN_TOLERANCE: f64 = 1.0e-5;

fn assert_valid(g: &Model<StandardPayload>, solid: SolidKey) {
    validate_solid_manifold(g, solid).expect("the bevelled solid should stay manifold");
    validate_solid_orientation(g, solid).expect("the bevelled solid's faces should stay outward");
}

#[test]
fn failed_chamfer_builder_preserves_the_source_profile() {
    let mut g = Model::<StandardPayload>::new();
    let profile = add_polyline(
        &mut g,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ],
    )
    .expect("profile should build");
    let corner = g.profile_unchecked(profile).edges()[0]
        .bounded_unchecked()
        .end()
        .key();
    let before_darts = g.dart_count();
    let before_edges = g.iter_edges().count();
    let before_vertices = g.iter_vertices().count();

    let result = chamfer(&mut g, corner, -1.0);

    assert!(
        matches!(result, Err(BlendError::InvalidDistance { .. })),
        "unexpected result: {result:?}"
    );
    assert_eq!(g.dart_count(), before_darts);
    assert_eq!(g.iter_edges().count(), before_edges);
    assert_eq!(g.iter_vertices().count(), before_vertices);
}

#[test]
fn profile_chamfer_reports_created_topology() {
    let mut g = Model::<StandardPayload>::new();
    let profile = add_polyline(
        &mut g,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ],
    )
    .expect("profile should build");
    let corner = g.profile_unchecked(profile).edges()[0]
        .bounded_unchecked()
        .end()
        .key();

    let result = chamfer(&mut g, corner, 0.25);

    result.expect("profile corner should chamfer");
    assert_eq!(g.iter_edges().count(), 3);
    assert_eq!(g.iter_vertices().count(), 4);
}

#[test]
fn solid_edge_chamfer_replaces_a_block_edge_with_a_planar_face() {
    let mut shape = block(2.0, 3.0, 4.0).expect("block should build");
    let solid = shape.key();
    let edge = shape
        .solid()
        .edges()
        .into_iter()
        .find(|edge| {
            let start = *edge.bounded_unchecked().start().point();
            let end = *edge.bounded_unchecked().end().point();
            (start.x - end.x).abs() < 1.0e-9
                && (start.y - end.y).abs() < 1.0e-9
                && (start.z - end.z).abs() > 3.9
        })
        .expect("block should have a vertical edge")
        .key();

    let result = chamfer(shape.model_mut(), edge, 0.25);

    result.expect("straight block edge should chamfer");
    assert_eq!(shape.solid().faces().len(), 7);
    assert_eq!(shape.solid().edges().len(), 15);
    assert_eq!(shape.solid().vertices().len(), 10);
    validate_solid_manifold(shape.model(), solid).expect("chamfered block should remain manifold");
    validate_solid_orientation(shape.model(), solid)
        .expect("chamfered block faces should remain outward");
}

#[test]
fn solid_vertex_chamfer_replaces_a_block_corner_with_a_planar_face() {
    let mut shape = block(2.0, 3.0, 4.0).expect("block should build");
    let solid = shape.key();
    let vertex = shape
        .solid()
        .vertices()
        .into_iter()
        .find(|vertex| {
            let point = *vertex.point();
            point.coords.norm() < 1.0e-9
        })
        .expect("block should have an origin vertex")
        .key();

    let result = chamfer(shape.model_mut(), vertex, 0.25);

    result.expect("trihedral block vertex should chamfer");
    assert_eq!(shape.solid().faces().len(), 7);
    assert_eq!(shape.solid().edges().len(), 15);
    assert_eq!(shape.solid().vertices().len(), 10);
    validate_solid_manifold(shape.model(), solid).expect("chamfered block should remain manifold");
    validate_solid_orientation(shape.model(), solid)
        .expect("chamfered block faces should remain outward");
}

#[test]
fn profile_target_chamfers_every_corner_in_place() {
    let mut g = Model::<StandardPayload>::new();
    let profile =
        ngk::builders::profiles::add_rectangle(&mut g, ngk::geometry::Plane::xy(), 2.0, 1.0)
            .expect("rectangle should build");

    chamfer(&mut g, profile, 0.1).expect("closed line profile should chamfer");

    assert_eq!(g.iter_edges().count(), 8);
    assert_eq!(g.iter_vertices().count(), 8);
}

#[test]
fn several_disjoint_solid_edges_can_be_chamfered_in_one_transaction() {
    let mut shape = block(2.0, 3.0, 4.0).expect("block should build");
    let solid = shape.key();
    let edges = shape
        .solid()
        .edges()
        .into_iter()
        .filter(|edge| {
            let start = *edge.bounded_unchecked().start().point();
            let end = *edge.bounded_unchecked().end().point();
            (start.x - end.x).abs() < 1.0e-9
                && (start.y - end.y).abs() < 1.0e-9
                && (start.z - end.z).abs() > 3.9
                && ((start.x < 1.0e-9 && start.y < 1.0e-9) || (start.x > 1.9 && start.y > 2.9))
        })
        .map(|edge| edge.key())
        .collect::<Vec<_>>();
    assert_eq!(edges.len(), 2);

    chamfer(shape.model_mut(), edges, 0.2).expect("disjoint block edges should chamfer");

    assert_eq!(shape.solid().faces().len(), 8);
    assert_eq!(shape.solid().edges().len(), 18);
    assert_eq!(shape.solid().vertices().len(), 12);
    validate_solid_manifold(shape.model(), solid).expect("multi-chamfer should remain manifold");
    validate_solid_orientation(shape.model(), solid)
        .expect("multi-chamfer faces should remain outward");
}

#[test]
fn solid_face_profile_chamfer_replaces_the_complete_rim_with_a_bevel_ring() {
    let mut shape = block(2.0, 3.0, 4.0).expect("block should build");
    let solid = shape.key();
    let top_profile = shape
        .solid()
        .faces()
        .into_iter()
        .find(|face| {
            face.vertices()
                .iter()
                .all(|vertex| (vertex.point().z - 4.0).abs() < 1.0e-9)
        })
        .expect("block should have a top face")
        .outer_loop()
        .expect("face should have an outer loop")
        .profile_key()
        .expect("the outer loop should run along a registered profile");

    chamfer(shape.model_mut(), top_profile, 0.25)
        .expect("complete top profile should chamfer as one solid operation");

    assert_eq!(shape.solid().faces().len(), 10);
    assert_eq!(shape.solid().edges().len(), 20);
    assert_eq!(shape.solid().vertices().len(), 12);
    validate_solid_manifold(shape.model(), solid)
        .expect("profile-chamfered block should remain manifold");
    validate_solid_orientation(shape.model(), solid)
        .expect("profile-chamfered block faces should remain outward");
}

#[test]
fn solid_edge_chamfer_supports_an_extruded_nurbs_profile_edge() {
    let mut g = Model::<StandardPayload>::new();
    let profile = add_polyline(
        &mut g,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 0.0),
            Point3::new(4.0, 3.0, 0.0),
        ],
    )
    .expect("open profile should build");
    let wavy_edge = add_edge(
        &mut g,
        Point3::new(4.0, 3.0, 0.0),
        Point3::new(0.0, 3.0, 0.0),
        Curve::Nurbs(
            NurbsCurve::interpolate(&[
                Point3::new(4.0, 3.0, 0.0),
                Point3::new(3.0, 2.65, 0.0),
                Point3::new(2.0, 3.35, 0.0),
                Point3::new(1.0, 2.65, 0.0),
                Point3::new(0.0, 3.0, 0.0),
            ])
            .expect("wave samples should interpolate"),
        ),
    )
    .expect("wavy edge should build");
    append_edge(&mut g, profile, wavy_edge).expect("wavy edge should append");
    let closing_edge = add_edge(
        &mut g,
        Point3::new(0.0, 3.0, 0.0),
        Point3::new(0.0, 0.0, 0.0),
        Curve::line(Point3::new(0.0, 3.0, 0.0), Point3::new(0.0, 0.0, 0.0)),
    )
    .expect("closing edge should build");
    append_edge(&mut g, profile, closing_edge).expect("profile should close");
    let face = add_face(&mut g, profile).expect("wavy planar face should build");
    let solid = add_extruded_face(&mut g, face, Vector3::new(0.0, 0.0, 2.0))
        .expect("wavy face should extrude")
        .solid;
    let top_wavy_edge = g
        .solid_unchecked(solid)
        .edges()
        .into_iter()
        .find(|edge| {
            matches!(edge.curve(), Curve::Nurbs(_))
                && (edge.bounded_unchecked().start().point().z - 2.0).abs() < 1.0e-9
        })
        .expect("extrusion should contain a translated wavy edge")
        .key();
    let original_faces = g.iter_faces().map(|(key, _)| key).collect::<HashSet<_>>();

    chamfer(&mut g, top_wavy_edge, 0.2).expect("wavy solid edge should chamfer");

    assert_eq!(g.solid_unchecked(solid).faces().len(), 7);
    let chamfer_face = g
        .solid_unchecked(solid)
        .faces()
        .into_iter()
        .find(|face| {
            !original_faces.contains(&face.key())
                && matches!(
                    face.surface(),
                    Surface::Ruled(surface) if surface.direction().y.abs() > 1.0e-9
                )
        })
        .expect("chamfer should insert a new face");
    assert!(
        matches!(chamfer_face.surface(), Surface::Ruled(_)),
        "the curved chamfer should have a ruled support surface"
    );
    assert_eq!(
        chamfer_face
            .edges()
            .iter()
            .filter(|edge| matches!(edge.curve(), Curve::Nurbs(_)))
            .count(),
        2,
        "the chamfer should retain both curved NURBS boundaries"
    );
    validate_solid_manifold(&g, solid).expect("curved chamfer should remain manifold");
    validate_solid_orientation(&g, solid).expect("curved chamfer faces should remain outward");
}

#[test]
fn chamfer_bevels_edges_sharing_a_vertex_together() {
    let (a, b, c, distance) = (2.0, 3.0, 4.0, 0.25);
    let mut shape = block(a, b, c).expect("block should build");
    let solid = shape.key();
    let origin = Point3::origin();
    let edges = [
        edge_between(shape.model(), solid, origin, Point3::new(a, 0.0, 0.0)),
        edge_between(shape.model(), solid, origin, Point3::new(0.0, 0.0, c)),
    ];

    chamfer(shape.model_mut(), edges, distance).expect("edges sharing a vertex should chamfer");

    validate_solid_manifold(shape.model(), solid).expect("mitred chamfer should remain manifold");
    validate_solid_orientation(shape.model(), solid)
        .expect("mitred chamfer faces should remain outward");
    // Two bevels meeting at a square corner overlap in d^3 / 3 of what they cut.
    let expected = a * b * c - distance * distance / 2.0 * (a + c) + distance.powi(3) / 3.0;
    let volume = shape
        .solid()
        .volume()
        .expect("chamfered block should measure");
    assert!(
        (volume - expected).abs() < 1.0e-9,
        "volume {volume} should be {expected}"
    );
}

#[test]
fn chamfer_bevels_every_block_edge_to_common_points() {
    let (a, b, c, distance) = (2.0, 3.0, 4.0, 0.25);
    let mut shape = block(a, b, c).expect("block should build");
    let solid = shape.key();
    let edges = shape
        .solid()
        .edges()
        .into_iter()
        .map(|edge| edge.key())
        .collect::<Vec<_>>();

    chamfer(shape.model_mut(), edges, distance).expect("every block edge should chamfer");

    assert_eq!(shape.solid().faces().len(), 18);
    assert_eq!(shape.solid().edges().len(), 48);
    assert_eq!(shape.solid().vertices().len(), 32);
    validate_solid_manifold(shape.model(), solid).expect("chamfered block should remain manifold");
    validate_solid_orientation(shape.model(), solid)
        .expect("chamfered block faces should remain outward");
    let expected = a * b * c - 2.0 * distance * distance * (a + b + c) + 6.0 * distance.powi(3);
    let volume = shape
        .solid()
        .volume()
        .expect("chamfered block should measure");
    assert!(
        (volume - expected).abs() < 1.0e-9,
        "volume {volume} should be {expected}"
    );
}

#[test]
fn chamfer_fills_a_concave_edge() {
    let distance = 0.2;
    let (mut g, solid) = prism(&L_SHAPE, 1.0);
    let edge = edge_between(
        &g,
        solid,
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(1.0, 1.0, 1.0),
    );

    chamfer(&mut g, edge, distance).expect("concave edge should chamfer");

    validate_solid_manifold(&g, solid).expect("filled edge should remain manifold");
    validate_solid_orientation(&g, solid).expect("filled edge faces should remain outward");
    let expected = 3.0 + distance * distance / 2.0;
    let volume = g
        .solid_unchecked(solid)
        .volume()
        .expect("filled solid should measure");
    assert!(
        (volume - expected).abs() < 1.0e-9,
        "volume {volume} should be {expected}"
    );
}

#[test]
fn chamfer_cuts_the_corners_between_two_arcs_along_them() {
    let mut g = Model::<StandardPayload>::new();
    let (profile, centres) = lens(&mut g);
    let distance = 0.1;

    chamfer(&mut g, profile, distance).expect("the lens's two corners should chamfer");

    assert_eq!(g.iter_edges().count(), 4);
    // Each trim lies on its arc, an arc length `distance` from its corner.
    for (_, attr) in g.iter_vertices() {
        let point = attr.point;
        let on_arc = centres
            .iter()
            .any(|centre| ((point - centre).norm() - 2.0_f64.sqrt()).abs() < 1.0e-9);
        assert!(on_arc, "trim {point:?} should lie on one of the arcs");
        let corner = Point3::new(point.x.signum(), 0.0, 0.0);
        let chord = (point - corner).norm();
        let arc = 2.0 * 2.0_f64.sqrt() * (chord / (2.0 * 2.0_f64.sqrt())).asin();
        assert!(
            (arc - distance).abs() < 1.0e-9,
            "trim should be {distance} along its arc"
        );
    }
}

#[test]
fn chamfer_bevels_the_rim_of_a_hole_through_a_solid() {
    let distance = 0.2;
    let (mut g, solid) = prism_with_hole(4.0, 2.0, 1.0);
    let top = face_at_height(&g, solid, 1.0);

    chamfer(&mut g, top, distance).expect("both rims of the holed top should chamfer");

    validate_solid_manifold(&g, solid).expect("bevelled rims should remain manifold");
    validate_solid_orientation(&g, solid).expect("bevelled rims should remain outward");
    // Four convex corners give back d^3 / 3 each, and the hole's four reflex
    // corners reach past their vertices by as much.
    let expected = 12.0 - distance * distance / 2.0 * 24.0;
    let volume = g
        .solid_unchecked(solid)
        .volume()
        .expect("holed solid should measure");
    assert!(
        (volume - expected).abs() < 1.0e-9,
        "volume {volume} should be {expected}"
    );
}

/// A boss fused onto a block meets it in a concave circle; its chamfer is
/// the cone between a circle on the block's top and one on the boss's wall,
/// each `distance` from the joint.
#[test]
fn chamfer_bevels_the_circle_where_a_boss_meets_a_block() {
    let (mut g, solid, joint) = boss_on_block();
    let before = g
        .solid_unchecked(solid)
        .volume()
        .expect("the boss should measure");
    let distance = 0.25;

    let result = chamfer(&mut g, joint, distance).expect("the joint should bevel");

    assert_valid(&g, solid);
    assert_eq!(result.faces.len(), 1);
    assert_eq!(result.consumed_edges, vec![joint]);
    assert!(matches!(
        g.face_attr_unchecked(result.faces[0]).surface,
        Surface::Cone(_)
    ));
    let mut rails = g
        .face_unchecked(result.faces[0])
        .edges()
        .iter()
        .map(|edge| match edge.curve() {
            Curve::Circle(circle) => (circle.radius(), circle.plane().origin().z),
            other => panic!("a rail should be a circle, got {other:?}"),
        })
        .collect::<Vec<_>>();
    rails.sort_by(|a, b| a.0.total_cmp(&b.0));
    assert_eq!(rails.len(), 2);
    assert!(
        (rails[0].0 - 1.0).abs() < TOLERANCE && (rails[0].1 - (2.0 + distance)).abs() < TOLERANCE
    );
    assert!(
        (rails[1].0 - (1.0 + distance)).abs() < TOLERANCE && (rails[1].1 - 2.0).abs() < TOLERANCE
    );
    // The triangle the bevel fills, turned about the axis at its centroid.
    let added = 2.0 * PI * (1.0 + distance / 3.0) * distance * distance / 2.0;
    let after = g
        .solid_unchecked(solid)
        .volume()
        .expect("the bevelled solid should measure");
    assert!(
        (after - before - added).abs() <= 0.05 * added,
        "the volume changed by {}, should by {added}",
        after - before
    );
}

/// Both rims of a bore bevel in one call, each its own cone.
#[test]
fn chamfer_bevels_both_rims_of_a_bore() {
    let (mut g, solid, rims) = plate_with_bore(4.0, 2.0);

    let result = chamfer(&mut g, rims.to_vec(), 0.25).expect("both rims should bevel");

    assert_valid(&g, solid);
    assert_eq!(result.faces.len(), 2);
    for &face in &result.faces {
        assert!(matches!(
            g.face_attr_unchecked(face).surface,
            Surface::Cone(_)
        ));
    }
}

/// A boss leaning into the block meets it in an ellipse, which no closed
/// form covers: the bevel is marched along it and skinned, its rails a true
/// `distance` from the edge on each face.
#[test]
fn chamfer_bevels_the_ellipse_where_an_oblique_boss_meets_a_block() {
    let (mut g, solid, joint) = oblique_boss_on_block();
    let edge = g.edge_unchecked(joint).trimmed_curve();
    let distance = 0.25;

    let result = chamfer(&mut g, joint, distance).expect("the joint should bevel");

    assert_valid(&g, solid);
    assert_eq!(result.faces.len(), 1);
    let Surface::Nurbs(skin) = &g.face_attr_unchecked(result.faces[0]).surface else {
        panic!("an ellipse's bevel should be skinned");
    };
    let tilt = 15.0_f64.to_radians();
    let axis = Vector3::new(tilt.sin(), 0.0, tilt.cos());
    let from_axis = |point: Point3| {
        let offset = point - Point3::new(1.7, 2.1, 0.5);
        (offset - axis * offset.dot(&axis)).norm()
    };
    for index in 0..=64 {
        let u = f64::from(index) / 64.0;
        let on_block = skin.point_at(u, 0.0);
        let on_wall = skin.point_at(u, 1.0);
        assert!((on_block.z - 2.0).abs() < SKIN_TOLERANCE, "{on_block:?}");
        assert!(
            (from_axis(on_wall) - 0.9).abs() < SKIN_TOLERANCE,
            "{on_wall:?}"
        );
        for rail in [on_block, on_wall] {
            let setback = (rail - edge.curve().project(rail)).norm();
            assert!(
                (setback - distance).abs() < SKIN_TOLERANCE,
                "a rail point at {u} lies {setback} from the edge"
            );
        }
    }
}

/// A slot's rim runs straight into arcs with no corner anywhere: each side is
/// bevelled by a strip, each end by a cone, and each pair joins along the
/// segment their bevels share where the rim turns from straight to curved.
#[test]
fn chamfer_bevels_the_smooth_rim_of_a_slot() {
    let (length, slot_radius, height) = (2.0, 1.0, 1.0);
    let (mut g, solid) = slot_prism(length, slot_radius, height);
    let top = face_at_height(&g, solid, height);
    let before = g
        .solid_unchecked(solid)
        .volume()
        .expect("the slot should measure");
    let distance = 0.25;

    let result = chamfer(&mut g, top, distance).expect("the slot's rim should bevel");

    assert_valid(&g, solid);
    assert_eq!(result.faces.len(), 4);
    let cones = result
        .faces
        .iter()
        .filter(|&&face| matches!(g.face_attr_unchecked(face).surface, Surface::Cone(_)))
        .count();
    assert_eq!(cones, 2);
    // The triangle the bevel cuts away, swept along both sides and turned
    // about both ends' axes at its centroid.
    let path = 2.0 * length + 2.0 * PI * (slot_radius - distance / 3.0);
    let removed = distance * distance / 2.0 * path;
    let after = g
        .solid_unchecked(solid)
        .volume()
        .expect("the bevelled slot should measure");
    assert!(
        (before - after - removed).abs() <= 0.05 * removed,
        "the volume changed by {}, should by {}",
        after - before,
        -removed
    );
}
