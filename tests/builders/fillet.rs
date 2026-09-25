use std::f64::consts::{FRAC_PI_2, PI};

use nalgebra::Vector3;
use ngk::builders::blend::{BlendError, BlendSelection, BlendTarget};
use ngk::builders::edges::add_arc;
use ngk::builders::faces::{add_polygon_with_holes, add_rectangle as add_rectangle_face};
use ngk::builders::fillet::fillet;
use ngk::builders::profiles::{add_polyline, add_rectangle, append_edge};
use ngk::geometry::{Curve, Plane, Point3, PointCoincidence, Surface};
use ngk::model::Model;
use ngk::modeling::solids::{block, cylinder};
use ngk::topology::StandardPayload;
use ngk::topology::shape_keys::{FaceKey, SolidKey};
use ngk::topology::validation::{validate_solid_manifold, validate_solid_orientation};
use radians::Rad64;

use super::blend_shapes::{
    L_SHAPE, TRAPEZOID, boss_on_block, closed_edge_at_height, edge_between, face_at_height, lens,
    oblique_boss_on_block, plate_with_bore, prism, prism_with_hole, slot_prism,
};

const TOLERANCE: f64 = 1.0e-9;

fn assert_valid(g: &Model<StandardPayload>, solid: SolidKey) {
    validate_solid_manifold(g, solid).expect("the rounded solid should stay manifold");
    validate_solid_orientation(g, solid).expect("the rounded solid's faces should stay outward");
}

/// Compares a tessellated volume with its closed form.
///
/// Curved faces are measured on their tessellation, so the tolerance is a
/// small share of the volume the blend changed: tight enough that a wrong
/// radius or a misplaced rail fails it.
fn assert_volume(g: &Model<StandardPayload>, solid: SolidKey, expected: f64, changed: f64) {
    let volume = g
        .solid_unchecked(solid)
        .volume()
        .expect("the rounded solid should measure");
    assert!(
        (volume - expected).abs() <= 0.01 * changed.abs(),
        "volume {volume} should be {expected}"
    );
}

/// How far a skinned blend may stray from the sections it was solved from:
/// the share of the fitting tolerance the skin is held to, with room for
/// evaluation.
const SKIN_TOLERANCE: f64 = 1.0e-5;

/// Where the centroid of a round's spandrel sits, in radii from the crease
/// along either face: the square of side `r` less its quarter disc.
const SPANDREL_CENTROID: f64 = (10.0 - 3.0 * PI) / (3.0 * (4.0 - PI));

/// The area a round of `radius` fills in, or cuts out of, a right-angled
/// crease.
fn spandrel(radius: f64) -> f64 {
    radius * radius * (1.0 - PI / 4.0)
}

/// Compares a tessellated change of volume with its closed form.
///
/// Both volumes are measured on one tessellation, so much of its error
/// cancels; but a curved face changes shape under a blend and the rest does
/// not, and five percent of the change is what that leaves — well short of
/// what a wrong radius or a misplaced rail would move it.
fn assert_volume_change(g: &Model<StandardPayload>, solid: SolidKey, before: f64, expected: f64) {
    let after = g
        .solid_unchecked(solid)
        .volume()
        .expect("the blended solid should measure");
    assert!(
        (after - before - expected).abs() <= 0.05 * expected.abs(),
        "the volume changed by {}, should by {expected}",
        after - before
    );
}

/// Asserts that `face` is the torus of tube `radius` whose tube's centre
/// circle has radius `spine` about the vertical through `centre`, in the
/// plane at `centre`'s height.
fn assert_round_about_axis(
    g: &Model<StandardPayload>,
    face: FaceKey,
    centre: Point3,
    spine: f64,
    radius: f64,
) {
    let Surface::Torus(torus) = &g.face_attr_unchecked(face).surface else {
        panic!("a round about an axis should be a torus");
    };
    assert!(
        torus.frame().origin.coincides(centre, TOLERANCE),
        "the torus should be centred at {centre:?}, is at {:?}",
        torus.frame().origin
    );
    assert!(torus.frame().z_dir.cross(&Vector3::z()).norm() < TOLERANCE);
    assert!((torus.major_radius() - spine).abs() < TOLERANCE);
    assert!((torus.minor_radius() - radius).abs() < TOLERANCE);
}

/// Asserts that `face`'s edges are circles about the vertical through
/// `axis`, one per `(radius, height)`.
fn assert_rails(g: &Model<StandardPayload>, face: FaceKey, rails: &[(f64, f64)], axis: Point3) {
    let mut circles = g
        .face_unchecked(face)
        .edges()
        .iter()
        .map(|edge| match edge.curve() {
            Curve::Circle(circle) => {
                let centre = circle.plane().origin();
                assert!((centre.x - axis.x).abs() < TOLERANCE);
                assert!((centre.y - axis.y).abs() < TOLERANCE);
                (circle.radius(), centre.z)
            }
            other => panic!("a rail should be a circle, got {other:?}"),
        })
        .collect::<Vec<_>>();
    circles.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut expected = rails.to_vec();
    expected.sort_by(|a, b| a.0.total_cmp(&b.0));
    assert_eq!(circles.len(), expected.len());
    for (circle, rail) in circles.iter().zip(&expected) {
        assert!(
            (circle.0 - rail.0).abs() < TOLERANCE && (circle.1 - rail.1).abs() < TOLERANCE,
            "rail {circle:?} should be {rail:?}"
        );
    }
}

fn count_curves(g: &Model<StandardPayload>, solid: SolidKey, pick: fn(&Curve) -> bool) -> usize {
    g.solid_unchecked(solid)
        .edges()
        .iter()
        .filter(|edge| pick(edge.curve()))
        .count()
}

#[test]
fn fillet_rounds_every_corner_of_a_rectangle_wire() {
    let mut g = Model::<StandardPayload>::new();
    let profile = add_rectangle(&mut g, Plane::xy(), 2.0, 1.0).expect("rectangle should build");
    let radius = 0.2;

    let result = fillet(&mut g, profile, radius).expect("rectangle corners should round");

    assert!(result.faces.is_empty());
    assert_eq!(g.iter_edges().count(), 8);
    assert_eq!(g.iter_vertices().count(), 8);
    let arcs = g
        .iter_edges()
        .filter(|(_, attr)| matches!(attr.curve, Curve::Circle(_)))
        .count();
    assert_eq!(arcs, 4);
    let expected = 2.0 * (2.0 + 1.0) - 8.0 * radius + 2.0 * PI * radius;
    let length = g.profile_unchecked(profile).length();
    assert!(
        (length - expected).abs() < TOLERANCE,
        "rounded rectangle length {length} should be {expected}"
    );
}

#[test]
fn fillet_rounds_every_corner_of_a_free_face() {
    let mut g = Model::<StandardPayload>::new();
    let face = add_rectangle_face(&mut g, Plane::xy(), 2.0, 1.0).expect("face should build");
    let radius = 0.25;

    fillet(&mut g, face, radius).expect("free face corners should round");

    let view = g.face_unchecked(face);
    assert_eq!(view.edges().len(), 8);
    let expected = 2.0 - (4.0 - PI) * radius * radius;
    let area = view.area().expect("rounded face should measure");
    assert!(
        (area - expected).abs() < 1.0e-3,
        "rounded face area {area} should be {expected}"
    );
    for edge in view.edges() {
        assert!(
            view.pcurve(edge.dart()).is_some(),
            "every boundary edge keeps a pcurve"
        );
    }
}

#[test]
fn fillet_rounds_a_corner_between_a_line_and_an_arc() {
    let mut g = Model::<StandardPayload>::new();
    // A line rising to the origin, then a quarter circle about (0, 1) leaving
    // it horizontally: a right-angled corner between a line and an arc.
    let profile = add_polyline(
        &mut g,
        &[Point3::new(0.0, -1.0, 0.0), Point3::new(0.0, 0.0, 0.0)],
    )
    .expect("line should build");
    let arc = add_arc(
        &mut g,
        Plane::from_xy(
            Point3::new(0.0, 1.0, 0.0),
            nalgebra::Vector3::x(),
            nalgebra::Vector3::y(),
        ),
        1.0,
        Rad64::new(-FRAC_PI_2),
        Rad64::new(0.0),
    )
    .expect("arc should build");
    append_edge(&mut g, profile, arc).expect("arc should append");
    let corner = g
        .iter_vertices()
        .find(|(_, attr)| attr.point.coords.norm() < TOLERANCE)
        .map(|(key, _)| key)
        .expect("the corner vertex should exist");
    let radius = 0.2;

    fillet(&mut g, corner, radius).expect("line-arc corner should round");

    assert_eq!(g.iter_edges().count(), 3);
    assert_eq!(g.iter_vertices().count(), 4);
    let (center, round_radius) = g
        .iter_edges()
        .find_map(|(_, attr)| match &attr.curve {
            Curve::Circle(circle) if (circle.radius() - radius).abs() < TOLERANCE => {
                Some((circle.plane().origin(), circle.radius()))
            }
            _ => None,
        })
        .expect("the round should be an arc of the requested radius");
    // Tangent to the line x = 0 and to the unit circle about (0, 1). The path
    // turns right at the corner and the arc then bends back left, so the round
    // sits outside that circle and touches it externally.
    assert!((center.x - round_radius).abs() < TOLERANCE);
    let to_arc_center = (center - Point3::new(0.0, 1.0, 0.0)).norm();
    assert!((to_arc_center - (1.0 + round_radius)).abs() < TOLERANCE);
}

#[test]
fn oversized_fillet_leaves_the_wire_unchanged() {
    let mut g = Model::<StandardPayload>::new();
    let profile = add_rectangle(&mut g, Plane::xy(), 2.0, 1.0).expect("rectangle should build");
    let before = (
        g.iter_edges().count(),
        g.iter_vertices().count(),
        g.dart_count(),
    );

    let result = fillet(&mut g, profile, 0.6);

    assert!(
        matches!(result, Err(BlendError::EdgeDoesNotFit { .. })),
        "unexpected result: {result:?}"
    );
    assert_eq!(
        (
            g.iter_edges().count(),
            g.iter_vertices().count(),
            g.dart_count()
        ),
        before
    );
}

#[test]
fn fillet_rejects_a_radius_that_is_not_positive() {
    let mut g = Model::<StandardPayload>::new();
    let profile = add_rectangle(&mut g, Plane::xy(), 2.0, 1.0).expect("rectangle should build");

    let result = fillet(&mut g, profile, 0.0);

    assert!(
        matches!(result, Err(BlendError::InvalidRadius { .. })),
        "unexpected result: {result:?}"
    );
}

#[test]
fn fillet_rounds_one_block_edge_into_a_cylinder() {
    let (a, b, c, radius) = (2.0, 3.0, 4.0, 0.25);
    let mut shape = block(a, b, c).expect("block should build");
    let solid = shape.key();
    let edge = edge_between(
        shape.model(),
        solid,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(0.0, 0.0, c),
    );

    let result = fillet(shape.model_mut(), edge, radius).expect("block edge should round");

    assert_eq!(result.consumed_edges, vec![edge]);
    assert_eq!(result.faces.len(), 1);
    let round = shape.model().face_unchecked(result.faces[0]);
    let Surface::Cylinder(support) = round.surface() else {
        panic!("a straight edge between planes should round into a cylinder");
    };
    assert!((support.radius - radius).abs() < TOLERANCE);
    assert_eq!(shape.solid().faces().len(), 7);
    assert_eq!(shape.solid().edges().len(), 15);
    assert_eq!(shape.solid().vertices().len(), 10);
    assert_valid(shape.model(), solid);
    let removed = (1.0 - PI / 4.0) * radius * radius * c;
    assert_volume(shape.model(), solid, a * b * c - removed, removed);
}

#[test]
fn fillet_mitres_the_rounds_around_a_block_face() {
    let (a, b, c, radius) = (2.0, 3.0, 4.0, 0.25);
    let mut shape = block(a, b, c).expect("block should build");
    let solid = shape.key();
    let top = face_at_height(shape.model(), solid, c);

    let result = fillet(shape.model_mut(), top, radius).expect("top face rim should round");

    assert_eq!(result.faces.len(), 4);
    assert_eq!(shape.solid().faces().len(), 10);
    assert_eq!(shape.solid().edges().len(), 20);
    assert_eq!(shape.solid().vertices().len(), 12);
    assert_eq!(
        count_curves(shape.model(), solid, |curve| matches!(
            curve,
            Curve::Ellipse(_)
        )),
        4,
        "each corner should be mitred along an ellipse"
    );
    assert_valid(shape.model(), solid);
    // Each corner's two rounds overlap in (5/3 - pi/2) r^3 of what they remove.
    let removed = (1.0 - PI / 4.0) * radius * radius * 2.0 * (a + b)
        - 4.0 * (5.0 / 3.0 - PI / 2.0) * radius.powi(3);
    assert_volume(shape.model(), solid, a * b * c - removed, removed);
}

#[test]
fn fillet_closes_a_fully_rounded_block_with_balls() {
    let (a, b, c, radius) = (2.0, 3.0, 4.0, 0.25);
    let mut shape = block(a, b, c).expect("block should build");
    let solid = shape.key();
    let edges = shape
        .solid()
        .edges()
        .into_iter()
        .map(|edge| edge.key())
        .collect::<Vec<_>>();

    let result = fillet(shape.model_mut(), edges, radius).expect("every block edge should round");

    assert_eq!(result.faces.len(), 20);
    assert_eq!(shape.solid().faces().len(), 26);
    let balls = shape
        .solid()
        .faces()
        .iter()
        .filter(|face| match face.surface() {
            Surface::Sphere(sphere) => (sphere.radius() - radius).abs() < TOLERANCE,
            _ => false,
        })
        .count();
    assert_eq!(balls, 8);
    assert_valid(shape.model(), solid);
    let (x, y, z) = (a - 2.0 * radius, b - 2.0 * radius, c - 2.0 * radius);
    let rounded = x * y * z
        + 2.0 * radius * (x * y + y * z + x * z)
        + PI * radius * radius * (x + y + z)
        + 4.0 / 3.0 * PI * radius.powi(3);
    assert_volume(shape.model(), solid, rounded, a * b * c - rounded);
}

#[test]
fn fillet_grows_the_caps_of_a_concave_edge() {
    let radius = 0.2;
    let (mut g, solid) = prism(&L_SHAPE, 1.0);
    let edge = edge_between(
        &g,
        solid,
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(1.0, 1.0, 1.0),
    );

    fillet(&mut g, edge, radius).expect("concave edge should round");

    assert_valid(&g, solid);
    let added = (1.0 - PI / 4.0) * radius * radius;
    assert_volume(&g, solid, 3.0 + added, added);
}

#[test]
fn fillet_mitres_the_reflex_corner_of_a_face_rim() {
    let radius = 0.2;
    let (mut g, solid) = prism(&L_SHAPE, 1.0);
    let top = face_at_height(&g, solid, 1.0);

    fillet(&mut g, top, radius).expect("the L's top rim should round");

    assert_valid(&g, solid);
    // Five convex corners each give back (5/3 - pi/2) r^3 of overlap; the
    // reflex one reaches past its vertex by the same amount.
    let removed =
        (1.0 - PI / 4.0) * radius * radius * 8.0 - 4.0 * (5.0 / 3.0 - PI / 2.0) * radius.powi(3);
    assert_volume(&g, solid, 3.0 - removed, removed);
}

#[test]
fn fillet_runs_out_along_an_ellipse_on_an_oblique_face() {
    let radius = 0.2;
    let (mut g, solid) = prism(&TRAPEZOID, 1.0);
    let edge = edge_between(
        &g,
        solid,
        Point3::new(0.0, 0.0, 1.0),
        Point3::new(4.0, 0.0, 1.0),
    );

    fillet(&mut g, edge, radius).expect("edge ending on a leaning face should round");

    assert_valid(&g, solid);
    assert_eq!(
        count_curves(&g, solid, |curve| matches!(curve, Curve::Ellipse(_))),
        1,
        "the leaning end face should cut the round along an ellipse"
    );
    // The leaning face x = 4 - y/2 shortens the round by y/2 at distance y
    // from the edge, so the removed volume loses half the section's first moment.
    let removed =
        4.0 * (1.0 - PI / 4.0) * radius * radius - 0.5 * (5.0 / 6.0 - PI / 4.0) * radius.powi(3);
    assert_volume(&g, solid, 7.0 - removed, removed);
}

#[test]
fn fillet_result_does_not_depend_on_how_the_selection_is_spelled() {
    let radius = 0.25;
    let summary = |target: fn(&Model<StandardPayload>, SolidKey) -> BlendTarget| {
        let mut shape = block(2.0, 3.0, 4.0).expect("block should build");
        let solid = shape.key();
        let target = target(shape.model(), solid);
        fillet(shape.model_mut(), target, radius).expect("top rim should round");
        (
            shape.solid().faces().len(),
            shape.solid().edges().len(),
            shape.solid().vertices().len(),
            shape
                .solid()
                .volume()
                .expect("rounded block should measure"),
        )
    };

    let by_face = summary(|g, solid| face_at_height(g, solid, 4.0).into());
    let by_profile = summary(|g, solid| {
        g.face_unchecked(face_at_height(g, solid, 4.0))
            .outer_loop()
            .and_then(|loop_| loop_.profile_key())
            .expect("the top face's rim should be a profile")
            .into()
    });
    let by_edges = summary(|g, solid| {
        let mut edges = g
            .face_unchecked(face_at_height(g, solid, 4.0))
            .edges()
            .iter()
            .map(|edge| edge.key())
            .collect::<Vec<_>>();
        edges.reverse();
        edges.rotate_left(1);
        edges.into()
    });
    let mixed = summary(|g, solid| {
        let top = face_at_height(g, solid, 4.0);
        let edge = g.face_unchecked(top).edges()[2].key();
        BlendTarget::new()
            .with(BlendSelection::Edge(edge))
            .with(BlendSelection::Face(top))
    });

    for other in [by_profile, by_edges, mixed] {
        assert_eq!(by_face.0, other.0);
        assert_eq!(by_face.1, other.1);
        assert_eq!(by_face.2, other.2);
        assert!((by_face.3 - other.3).abs() < TOLERANCE);
    }
}

#[test]
fn fillet_refuses_rounds_that_overlap_on_a_narrow_face() {
    let mut shape = block(1.0, 3.0, 4.0).expect("block should build");
    let solid = shape.key();
    let edges = [
        edge_between(
            shape.model(),
            solid,
            Point3::new(0.0, 0.0, 4.0),
            Point3::new(0.0, 3.0, 4.0),
        ),
        edge_between(
            shape.model(),
            solid,
            Point3::new(1.0, 0.0, 4.0),
            Point3::new(1.0, 3.0, 4.0),
        ),
    ];
    let before = (shape.model().dart_count(), shape.solid().faces().len());

    let result = fillet(shape.model_mut(), edges, 0.6);

    assert!(
        matches!(result, Err(BlendError::EdgeDoesNotFit { .. })),
        "unexpected result: {result:?}"
    );
    assert_eq!(
        (shape.model().dart_count(), shape.solid().faces().len()),
        before
    );
}

#[test]
fn fillet_refuses_a_radius_reaching_past_the_edges_beside_it() {
    let mut shape = block(1.0, 3.0, 4.0).expect("block should build");
    let solid = shape.key();
    let edge = edge_between(
        shape.model(),
        solid,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(0.0, 0.0, 4.0),
    );

    let result = fillet(shape.model_mut(), edge, 1.5);

    assert!(
        matches!(result, Err(BlendError::VertexDoesNotFit { .. })),
        "unexpected result: {result:?}"
    );
    assert_eq!(shape.solid().faces().len(), 6);
}

#[test]
fn fillet_refuses_a_solid_vertex() {
    let mut shape = block(1.0, 1.0, 1.0).expect("block should build");
    let vertex = shape.solid().vertices()[0].key();

    let result = fillet(shape.model_mut(), vertex, 0.1);

    assert!(
        matches!(result, Err(BlendError::SolidVertexFillet { .. })),
        "unexpected result: {result:?}"
    );
}

/// A cylinder's rim is one closed edge with no corner: its round is the
/// torus about the cylinder's axis, and has no end to treat.
#[test]
fn fillet_rounds_the_rim_of_a_cylinder() {
    let mut shape = cylinder(1.0, 2.0).expect("cylinder should build");
    let solid = shape.key();
    let rim = closed_edge_at_height(shape.model(), solid, 2.0);
    let before = shape.solid().volume().expect("the cylinder should measure");
    let radius = 0.25;

    let result = fillet(shape.model_mut(), rim, radius).expect("the rim should round");

    let g = shape.model();
    assert_valid(g, solid);
    assert_eq!(result.faces.len(), 1);
    assert_eq!(result.consumed_edges, vec![rim]);
    assert_round_about_axis(
        g,
        result.faces[0],
        Point3::new(0.0, 0.0, 1.75),
        0.75,
        radius,
    );
    assert_rails(
        g,
        result.faces[0],
        &[(0.75, 2.0), (1.0, 1.75)],
        Point3::origin(),
    );
    // Pappus turns the spandrel about the axis at its centroid, which sits
    // in from the rim on a convex round.
    let removed = 2.0 * PI * (1.0 - SPANDREL_CENTROID * radius) * spandrel(radius);
    assert_volume_change(g, solid, before, -removed);
}

/// A boss fused onto a block meets it in a concave circle; its round is the
/// torus whose tube runs round the boss, touching the block's top and the
/// boss's wall.
#[test]
fn fillet_rounds_the_circle_where_a_boss_meets_a_block() {
    let (mut g, solid, joint) = boss_on_block();
    let before = g
        .solid_unchecked(solid)
        .volume()
        .expect("the boss should measure");
    let radius = 0.25;

    let result = fillet(&mut g, joint, radius).expect("the joint should round");

    assert_valid(&g, solid);
    assert_eq!(result.faces.len(), 1);
    assert_eq!(result.consumed_edges, vec![joint]);
    let centre = Point3::new(2.0, 2.0, 2.0 + radius);
    assert_round_about_axis(&g, result.faces[0], centre, 1.0 + radius, radius);
    assert_rails(
        &g,
        result.faces[0],
        &[(1.0 + radius, 2.0), (1.0, 2.0 + radius)],
        Point3::new(2.0, 2.0, 0.0),
    );
    let added = 2.0 * PI * (1.0 + SPANDREL_CENTROID * radius) * spandrel(radius);
    assert_volume_change(&g, solid, before, added);
}

/// Both rims of a bore are rounded in one call, each its own torus, and the
/// wall between them keeps the band the two rails leave it.
#[test]
fn fillet_rounds_both_rims_of_a_bore() {
    let (mut g, solid, rims) = plate_with_bore(4.0, 2.0);
    let before = g
        .solid_unchecked(solid)
        .volume()
        .expect("the plate should measure");
    let radius = 0.25;

    let result = fillet(&mut g, rims.to_vec(), radius).expect("both rims should round");

    assert_valid(&g, solid);
    assert_eq!(result.faces.len(), 2);
    let mut heights = result
        .faces
        .iter()
        .map(|&face| match &g.face_attr_unchecked(face).surface {
            Surface::Torus(torus) => torus.frame().origin.z,
            other => panic!("a rim's round should be a torus, got {other:?}"),
        })
        .collect::<Vec<_>>();
    heights.sort_by(f64::total_cmp);
    for (face, height) in result.faces.iter().zip(heights) {
        let centre = Point3::new(2.0, 2.0, height);
        assert_round_about_axis(&g, *face, centre, 1.0 + radius, radius);
    }
    let removed = 2.0 * 2.0 * PI * (1.0 + SPANDREL_CENTROID * radius) * spandrel(radius);
    assert_volume_change(&g, solid, before, -removed);
}

/// A boss's wall is bounded by two closed edges, one convex and one concave;
/// selecting the wall rounds both, each the right way.
#[test]
fn fillet_rounds_both_closed_edges_of_a_selected_face() {
    let (mut g, solid, _) = boss_on_block();
    let wall = g
        .solid_unchecked(solid)
        .faces()
        .into_iter()
        .find(|face| matches!(face.surface(), Surface::Cylinder(_)))
        .expect("the boss should have a wall")
        .key();
    let radius = 0.25;

    let result = fillet(&mut g, wall, radius).expect("the wall's rims should round");

    assert_valid(&g, solid);
    let mut spines = result
        .faces
        .iter()
        .map(|&face| match &g.face_attr_unchecked(face).surface {
            Surface::Torus(torus) => (torus.major_radius(), torus.frame().origin.z),
            other => panic!("a rim's round should be a torus, got {other:?}"),
        })
        .collect::<Vec<_>>();
    spines.sort_by(|a, b| a.0.total_cmp(&b.0));
    assert_eq!(spines.len(), 2);
    // The convex rim at the boss's top, then the concave one at its foot.
    assert!((spines[0].0 - (1.0 - radius)).abs() < TOLERANCE);
    assert!((spines[0].1 - (3.0 - radius)).abs() < TOLERANCE);
    assert!((spines[1].0 - (1.0 + radius)).abs() < TOLERANCE);
    assert!((spines[1].1 - (2.0 + radius)).abs() < TOLERANCE);
}

/// A round wider than the rim it runs round would have its tube pass through
/// the axis; there is no torus to build.
#[test]
fn fillet_refuses_a_round_wider_than_the_rim_it_rounds() {
    let mut shape = cylinder(0.3, 2.0).expect("cylinder should build");
    let rim = closed_edge_at_height(shape.model(), shape.key(), 2.0);

    let result = fillet(shape.model_mut(), rim, 0.5);

    assert!(
        matches!(result, Err(BlendError::EdgeDoesNotFit { edge, .. }) if edge == rim),
        "unexpected result: {result:?}"
    );
}

/// The rounds at the two ends of a short bore meet on its wall when their
/// rails pass each other, which would turn the wall inside out.
#[test]
fn fillet_refuses_rounds_that_overrun_the_wall_between_them() {
    let (mut g, solid, rims) = plate_with_bore(6.0, 1.0);
    let before = g.solid_unchecked(solid).faces().len();

    let result = fillet(&mut g, rims.to_vec(), 0.6);

    assert!(
        matches!(result, Err(BlendError::FaceDoesNotFit { .. })),
        "unexpected result: {result:?}"
    );
    assert_eq!(g.solid_unchecked(solid).faces().len(), before);
}

/// A boss leaning into the block meets it in an ellipse, which no closed
/// form covers: the round is marched along it and skinned. Every section is
/// still the arc of one ball touching both faces.
#[test]
fn fillet_rounds_the_ellipse_where_an_oblique_boss_meets_a_block() {
    let (mut g, solid, joint) = oblique_boss_on_block();
    let before = g
        .solid_unchecked(solid)
        .volume()
        .expect("the boss should measure");
    let radius = 0.25;

    let result = fillet(&mut g, joint, radius).expect("the joint should round");

    assert_valid(&g, solid);
    assert_eq!(result.faces.len(), 1);
    let Surface::Nurbs(skin) = &g.face_attr_unchecked(result.faces[0]).surface else {
        panic!("an ellipse's round should be skinned");
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
        assert!((on_block.z - 2.0).abs() < SKIN_TOLERANCE, "{on_block:?}");
        // The ball touches the block's top from above, so its centre is one
        // radius over the contact, and one radius out from the wall.
        let centre = on_block + Vector3::z() * radius;
        assert!((from_axis(centre) - (0.9 + radius)).abs() < SKIN_TOLERANCE);
        let on_wall = skin.point_at(u, 1.0);
        assert!((from_axis(on_wall) - 0.9).abs() < SKIN_TOLERANCE);
        for v in [0.0, 0.25, 0.5, 0.75, 1.0] {
            let distance = (skin.point_at(u, v) - centre).norm();
            assert!(
                (distance - radius).abs() < SKIN_TOLERANCE,
                "the skin at ({u}, {v}) lies {distance} from its ball's centre"
            );
        }
    }
    // A concave round adds material, close to what a right-angled round of
    // a circle the ellipse's size would: the crease opens by the tilt on one
    // side as much as it closes on the other.
    let added = g
        .solid_unchecked(solid)
        .volume()
        .expect("the round should measure")
        - before;
    assert!((0.08..0.1).contains(&added), "the round added {added}");
}

/// A round's rails run tangent into the faces beside them, so a round is
/// no crease: expanding its face finds nothing to blend, and naming a rail is
/// refused.
#[test]
fn fillet_finds_no_crease_along_a_round() {
    let (mut g, solid, joint) = boss_on_block();
    let round = fillet(&mut g, joint, 0.25)
        .expect("the joint should round")
        .faces[0];
    let rail = g.face_unchecked(round).edges()[0].key();

    let expanded = fillet(&mut g, round, 0.1);
    let named = fillet(&mut g, rail, 0.1);

    assert!(
        matches!(expanded, Err(BlendError::EmptyTarget)),
        "unexpected result: {expanded:?}"
    );
    assert!(
        matches!(named, Err(BlendError::FlatEdge { edge }) if edge == rail),
        "unexpected result: {named:?}"
    );
    assert_valid(&g, solid);
}

#[test]
fn fillet_refuses_rounds_that_land_apart_at_a_vertex() {
    // At (4, 0, 1) the front face meets the top square and the leaning side at
    // an angle, so a round along the top edge and one up the vertical edge
    // land on different points of the leaning side's top edge.
    let (mut g, solid) = prism(&TRAPEZOID, 1.0);
    let edges = [
        edge_between(
            &g,
            solid,
            Point3::new(0.0, 0.0, 1.0),
            Point3::new(4.0, 0.0, 1.0),
        ),
        edge_between(
            &g,
            solid,
            Point3::new(4.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 1.0),
        ),
    ];
    let before = g.dart_count();

    let result = fillet(&mut g, edges, 0.2);

    assert!(
        matches!(result, Err(BlendError::UnsupportedVertex { .. })),
        "unexpected result: {result:?}"
    );
    assert_eq!(g.dart_count(), before);
}

#[test]
fn fillet_refuses_a_convex_and_a_concave_round_meeting() {
    let (mut g, solid) = prism(&L_SHAPE, 1.0);
    let edges = [
        edge_between(
            &g,
            solid,
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
        ),
        edge_between(
            &g,
            solid,
            Point3::new(1.0, 1.0, 1.0),
            Point3::new(2.0, 1.0, 1.0),
        ),
    ];

    let result = fillet(&mut g, edges, 0.2);

    assert!(
        matches!(result, Err(BlendError::UnsupportedVertex { .. })),
        "unexpected result: {result:?}"
    );
}

#[test]
fn fillet_rounds_the_corners_between_two_arcs() {
    let mut g = Model::<StandardPayload>::new();
    let (profile, centres) = lens(&mut g);
    let radius = 0.1;

    fillet(&mut g, profile, radius).expect("the lens's two corners should round");

    assert_eq!(g.iter_edges().count(), 4);
    let rounds = g
        .iter_edges()
        .filter_map(|(_, attr)| match &attr.curve {
            Curve::Circle(circle) if (circle.radius() - radius).abs() < TOLERANCE => {
                Some(circle.plane().origin())
            }
            _ => None,
        })
        .collect::<Vec<_>>();
    assert_eq!(rounds.len(), 2);
    // Inside both circles, so each round touches each arc from inside it.
    for center in rounds {
        for arc_center in centres {
            let distance = (center - arc_center).norm();
            assert!((distance - (2.0_f64.sqrt() - radius)).abs() < TOLERANCE);
        }
    }
}

#[test]
fn fillet_rounds_the_hole_of_a_free_face_as_well_as_its_rim() {
    let mut g = Model::<StandardPayload>::new();
    let square = |size: f64, offset: f64| {
        vec![
            Point3::new(offset, offset, 0.0),
            Point3::new(offset + size, offset, 0.0),
            Point3::new(offset + size, offset + size, 0.0),
            Point3::new(offset, offset + size, 0.0),
        ]
    };
    let outer = square(4.0, 0.0);
    let hole = square(2.0, 1.0);
    let face = add_polygon_with_holes(&mut g, Plane::xy(), &outer, &[&hole])
        .expect("holed face should build");

    fillet(&mut g, face, 0.2).expect("every corner of the holed face should round");

    let view = g.face_unchecked(face);
    assert_eq!(view.loops().len(), 2);
    assert_eq!(view.edges().len(), 16);
    // The rim loses (4 - pi) r^2 and the hole shrinks by as much.
    let area = view.area().expect("holed face should measure");
    assert!((area - 12.0).abs() < 1.0e-3, "area {area} should be 12");
}

#[test]
fn fillet_rounds_both_rims_of_a_hole_through_a_solid() {
    let radius = 0.2;
    let (mut g, solid) = prism_with_hole(4.0, 2.0, 1.0);
    let top = face_at_height(&g, solid, 1.0);

    fillet(&mut g, top, radius).expect("both rims of the holed top should round");

    assert_valid(&g, solid);
    // Four convex corners give back (5/3 - pi/2) r^3 each, and the hole's four
    // reflex corners reach past their vertices by as much.
    let removed = (1.0 - PI / 4.0) * radius * radius * 24.0;
    assert_volume(&g, solid, 12.0 - removed, removed);
}

/// A slot's rim runs straight into arcs with no corner anywhere: each side is
/// rounded by a cylinder, each end by a torus, and each pair joins along the
/// arc their rounds share where the rim turns from straight to curved.
#[test]
fn fillet_rounds_the_smooth_rim_of_a_slot() {
    let (length, slot_radius, height) = (2.0, 1.0, 1.0);
    let (mut g, solid) = slot_prism(length, slot_radius, height);
    let top = face_at_height(&g, solid, height);
    let before = g
        .solid_unchecked(solid)
        .volume()
        .expect("the slot should measure");
    let radius = 0.25;

    let result = fillet(&mut g, top, radius).expect("the slot's rim should round");

    assert_valid(&g, solid);
    assert_eq!(result.faces.len(), 4);
    assert_eq!(result.consumed_edges.len(), 4);
    let kinds = result
        .faces
        .iter()
        .map(|&face| match &g.face_attr_unchecked(face).surface {
            Surface::Cylinder(_) => "cylinder",
            Surface::Torus(_) => "torus",
            other => panic!("a slot's rounds should be cylinders and tori, got {other:?}"),
        })
        .collect::<Vec<_>>();
    assert_eq!(kinds.iter().filter(|kind| **kind == "torus").count(), 2);
    // Pappus along the rim: the spandrel swept along both straight sides and
    // turned about both ends' axes at its centroid's radius.
    let path = 2.0 * length + 2.0 * PI * (slot_radius - SPANDREL_CENTROID * radius);
    assert_volume_change(&g, solid, before, -spandrel(radius) * path);
}

/// A round cannot stop part way along a smooth crease, so naming one arc of a
/// slot's rim rounds every edge the arc runs on into, the same as naming the
/// whole rim.
#[test]
fn fillet_carries_a_round_along_the_edges_it_runs_on_into() {
    let (mut g, solid) = slot_prism(2.0, 1.0, 1.0);
    let arc = g
        .face_unchecked(face_at_height(&g, solid, 1.0))
        .edges()
        .into_iter()
        .find(|edge| matches!(edge.curve(), Curve::Circle(_)))
        .expect("the slot's rim should have an arc")
        .key();
    let (mut whole, whole_solid) = slot_prism(2.0, 1.0, 1.0);
    let top = face_at_height(&whole, whole_solid, 1.0);

    let result = fillet(&mut g, arc, 0.25).expect("the arc's chain should round");
    let expected = fillet(&mut whole, top, 0.25).expect("the rim should round");

    assert_valid(&g, solid);
    assert_eq!(result.consumed_edges.len(), 4);
    assert_eq!(result.faces.len(), expected.faces.len());
    let volume = g.solid_unchecked(solid).volume().expect("should measure");
    let whole_volume = whole
        .solid_unchecked(whole_solid)
        .volume()
        .expect("should measure");
    assert!((volume - whole_volume).abs() < TOLERANCE);
}
