use std::f64::consts::{FRAC_PI_2, PI};

use nalgebra::Vector3;
use ngk::builders::blend::{BlendError, BlendSelection, BlendTarget};
use ngk::builders::edges::add_arc;
use ngk::builders::faces::{add_face, add_rectangle as add_rectangle_face};
use ngk::builders::fillet::fillet;
use ngk::builders::profiles::{add_polyline, add_rectangle, append_edge};
use ngk::builders::solids::add_extruded_face;
use ngk::geometry::{Curve, Plane, Point3, Surface};
use ngk::model::Model;
use ngk::modeling::solids::{block, cylinder};
use ngk::topology::StandardPayload;
use ngk::topology::shape_keys::{EdgeKey, FaceKey, SolidKey};
use ngk::topology::validation::{validate_solid_manifold, validate_solid_orientation};
use radians::Rad64;

const TOLERANCE: f64 = 1.0e-9;

/// An L seen from above: five convex corners and one reflex corner at (1, 1).
const L_SHAPE: [(f64, f64); 6] = [
    (0.0, 0.0),
    (2.0, 0.0),
    (2.0, 1.0),
    (1.0, 1.0),
    (1.0, 2.0),
    (0.0, 2.0),
];

/// A quadrilateral whose side from (4, 0) to (3, 2) leans away from square.
const TRAPEZOID: [(f64, f64); 4] = [(0.0, 0.0), (4.0, 0.0), (3.0, 2.0), (0.0, 2.0)];

/// Extrudes the closed polygon through `points` in the xy plane by `height`.
fn prism(points: &[(f64, f64)], height: f64) -> (Model<StandardPayload>, SolidKey) {
    let mut g = Model::<StandardPayload>::new();
    let mut corners = points
        .iter()
        .map(|&(x, y)| Point3::new(x, y, 0.0))
        .collect::<Vec<_>>();
    corners.push(corners[0]);
    let profile = add_polyline(&mut g, &corners).expect("polygon should build");
    let face = add_face(&mut g, profile).expect("polygon face should build");
    let solid = add_extruded_face(&mut g, face, Vector3::new(0.0, 0.0, height))
        .expect("polygon should extrude")
        .solid;
    (g, solid)
}

fn edge_between(g: &Model<StandardPayload>, solid: SolidKey, a: Point3, b: Point3) -> EdgeKey {
    g.solid_unchecked(solid)
        .edges()
        .into_iter()
        .find(|edge| {
            let bounded = edge.clone().bounded_unchecked();
            let (start, end) = (*bounded.start().point(), *bounded.end().point());
            ((start - a).norm() < TOLERANCE && (end - b).norm() < TOLERANCE)
                || ((start - b).norm() < TOLERANCE && (end - a).norm() < TOLERANCE)
        })
        .expect("the solid should have this edge")
        .key()
}

fn face_at_height(g: &Model<StandardPayload>, solid: SolidKey, z: f64) -> FaceKey {
    g.solid_unchecked(solid)
        .faces()
        .into_iter()
        .find(|face| {
            face.vertices()
                .iter()
                .all(|vertex| (vertex.point().z - z).abs() < TOLERANCE)
        })
        .expect("the solid should have a face at this height")
        .key()
}

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
    assert!(
        matches!(round.surface(), Surface::Cylinder(cylinder) if (cylinder.radius - radius).abs() < TOLERANCE)
    );
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
        .filter(|face| matches!(face.surface(), Surface::Sphere(sphere) if (sphere.radius() - radius).abs() < TOLERANCE))
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

#[test]
fn fillet_refuses_an_edge_it_has_no_section_for() {
    let mut shape = cylinder(1.0, 2.0).expect("cylinder should build");
    let rim = shape
        .solid()
        .edges()
        .into_iter()
        .find(|edge| matches!(edge.curve(), Curve::Circle(_)))
        .expect("the cylinder should have a circular rim")
        .key();

    let result = fillet(shape.model_mut(), rim, 0.1);

    assert!(
        matches!(result, Err(BlendError::UnsupportedEdge { .. })),
        "unexpected result: {result:?}"
    );
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
