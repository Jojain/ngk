use std::collections::HashSet;

use nalgebra::Vector3;
use radians::Rad64;

use ngk::builders::edges::add_edge;
use ngk::builders::faces::{add_face, add_polygon};
use ngk::builders::revolve::{RevolveError, add_revolved_edge, add_revolved_face};
use ngk::geometry::axis::Axis3;
use ngk::geometry::{Curve, LINEAR_TOLERANCE, Point3, PointCoincidence, Surface};
use ngk::tessellate::{TessellateOpts, tessellate_face_key};
use ngk::topology::gmap::GMap;
use ngk::topology::payload::StandardPayload;
use ngk::topology::validation::{validate_solid_manifold, validate_solid_orientation};

#[test]
fn revolve_edge_partial_turn_creates_four_edge_face() {
    let mut g = GMap::<StandardPayload>::new();
    let edge_key = add_edge(
        &mut g,
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
        Curve::line(Point3::new(1.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)),
    )
    .expect("edge should build");

    let face_key = add_revolved_edge(
        &mut g,
        edge_key,
        Axis3::new(Point3::origin(), Vector3::z()),
        Rad64::QUARTER_TURN,
    )
    .unwrap();
    let face = g.face_unchecked(face_key);
    let boundary_edges = face.outer_loop().edges();
    let boundary_vertices = face.outer_loop().vertices();
    let boundary_edge_keys = boundary_edges
        .iter()
        .map(|edge| edge.key())
        .collect::<HashSet<_>>();
    let boundary_vertex_keys = boundary_vertices
        .iter()
        .map(|vertex| vertex.key())
        .collect::<HashSet<_>>();
    assert_eq!(
        (
            g.iter_faces().count(),
            g.iter_edges().count(),
            g.iter_vertices().count()
        ),
        (1, 4, 4)
    );
    assert_eq!(
        (
            face.loops().len(),
            boundary_edges.len(),
            boundary_vertices.len()
        ),
        (1, 4, 4)
    );
    assert_eq!(
        (boundary_edge_keys.len(), boundary_vertex_keys.len()),
        (4, 4)
    );
    assert_eq!(
        boundary_edges
            .iter()
            .filter(|edge| edge.key() == edge_key)
            .count(),
        1,
        "the source edge should occur once in the boundary"
    );
}

#[test]
fn revolve_edge_partial_turn_uses_quarter_circle_sides() {
    let mut g = GMap::<StandardPayload>::new();
    let edge_key = add_edge(
        &mut g,
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
        Curve::line(Point3::new(1.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)),
    )
    .expect("edge should build");

    let face_key = add_revolved_edge(
        &mut g,
        edge_key,
        Axis3::new(Point3::origin(), Vector3::z()),
        Rad64::QUARTER_TURN,
    )
    .unwrap();

    let midpoint = side_arc_midpoint(&g, face_key, Point3::new(1.0, 0.0, 0.0));

    assert!(
        midpoint.coincides(
            Point3::new(
                std::f64::consts::FRAC_1_SQRT_2,
                std::f64::consts::FRAC_1_SQRT_2,
                0.0
            ),
            LINEAR_TOLERANCE
        ),
        "side arc midpoint should stay on the same quarter-turn as the revolved face, got {midpoint:?}"
    );
}

#[test]
fn revolve_edge_past_half_turn_sweeps_the_long_way() {
    let mut g = GMap::<StandardPayload>::new();
    let edge_key = add_edge(
        &mut g,
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
        Curve::line(Point3::new(1.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)),
    )
    .expect("edge should build");

    let face_key = add_revolved_edge(
        &mut g,
        edge_key,
        Axis3::new(Point3::origin(), Vector3::z()),
        Rad64::new(3.0 * std::f64::consts::FRAC_PI_2),
    )
    .unwrap();

    let midpoint = side_arc_midpoint(&g, face_key, Point3::new(1.0, 0.0, 0.0));
    let three_eighths = 3.0 * std::f64::consts::FRAC_PI_4;

    assert!(
        midpoint.coincides(
            Point3::new(three_eighths.cos(), three_eighths.sin(), 0.0),
            LINEAR_TOLERANCE
        ),
        "a three-quarter turn should sweep through 135 degrees, not back through -45, got {midpoint:?}"
    );
}

/// Returns the midpoint of the arc swept by `origin` on a revolved face.
fn side_arc_midpoint(
    g: &GMap<StandardPayload>,
    face_key: ngk::topology::shape_keys::FaceKey,
    origin: Point3,
) -> Point3 {
    let arc = g
        .face_unchecked(face_key)
        .edges()
        .into_iter()
        .find(|edge| {
            let start = *edge
                .start()
                .point()
                .expect("arc start should have geometry");
            let end = *edge.end().point().expect("arc end should have geometry");
            matches!(edge.curve().map(Curve::base), Some(Curve::Circle(_)))
                && (start.coincides(origin, LINEAR_TOLERANCE)
                    || end.coincides(origin, LINEAR_TOLERANCE))
        })
        .expect("revolve should create a circular side arc");
    let curve = arc.curve().expect("side arc should have geometry");
    let start = *arc.start().point().expect("arc start should have geometry");
    let end = *arc.end().point().expect("arc end should have geometry");
    let interval = curve.parameters_between(start, end);
    curve.point_at(0.5 * (interval.start + interval.end))
}

#[test]
fn revolve_edge_full_turn_creates_inner_loop() {
    let mut g = GMap::<StandardPayload>::new();
    let edge_key = add_edge(
        &mut g,
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
        Curve::line(Point3::new(1.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)),
    )
    .expect("edge should build");

    let face_key = add_revolved_edge(
        &mut g,
        edge_key,
        Axis3::new(Point3::origin(), Vector3::z()),
        Rad64::FULL_TURN,
    )
    .unwrap();
    let face = g.face_unchecked(face_key);
    let boundary_edges = face.edges();
    let boundary_vertices = face.vertices();
    let boundary_edge_keys = boundary_edges
        .iter()
        .map(|edge| edge.key())
        .collect::<HashSet<_>>();
    let boundary_vertex_keys = boundary_vertices
        .iter()
        .map(|vertex| vertex.key())
        .collect::<HashSet<_>>();
    assert_eq!(
        (
            g.iter_faces().count(),
            g.iter_edges().count(),
            g.iter_vertices().count()
        ),
        (1, 2, 2)
    );
    assert_eq!(
        (
            face.loops().len(),
            boundary_edges.len(),
            boundary_vertices.len()
        ),
        (2, 2, 2)
    );
    assert_eq!(
        (boundary_edge_keys.len(), boundary_vertex_keys.len()),
        (2, 2)
    );
}

#[test]
fn revolve_edge_full_turn_without_inner_loop() {
    let mut g = GMap::<StandardPayload>::new();
    let edge_key = add_edge(
        &mut g,
        Point3::origin(),
        Point3::new(2.0, 0.0, 0.0),
        Curve::line(Point3::origin(), Point3::new(2.0, 0.0, 0.0)),
    )
    .expect("edge should build");

    let face_key = add_revolved_edge(
        &mut g,
        edge_key,
        Axis3::new(Point3::origin(), Vector3::z()),
        Rad64::FULL_TURN,
    )
    .unwrap();
    let face = g.face_unchecked(face_key);
    let boundary_edges = face.edges();
    let boundary_vertices = face.vertices();
    let boundary_edge_keys = boundary_edges
        .iter()
        .map(|edge| edge.key())
        .collect::<HashSet<_>>();
    let boundary_vertex_keys = boundary_vertices
        .iter()
        .map(|vertex| vertex.key())
        .collect::<HashSet<_>>();

    assert_eq!(
        (
            g.iter_faces().count(),
            g.iter_edges().count(),
            g.iter_vertices().count()
        ),
        (1, 1, 1)
    );
    assert_eq!(
        (
            face.loops().len(),
            boundary_edges.len(),
            boundary_vertices.len()
        ),
        (1, 1, 1)
    );
    assert_eq!(
        (boundary_edge_keys.len(), boundary_vertex_keys.len()),
        (1, 1)
    );
}

#[test]
fn revolved_face_adds_surface_of_revolution_faces() {
    let mut g = GMap::<StandardPayload>::new();
    let profile_key = add_polygon(
        &mut g,
        &[
            Point3::new(0.75, 0.0, -0.85),
            Point3::new(1.85, 0.0, -0.05),
            Point3::new(0.85, 0.0, 0.9),
        ],
    );
    let source_face = add_face(&mut g, profile_key).unwrap();

    add_revolved_face(
        &mut g,
        source_face,
        Axis3::new(Point3::origin(), Vector3::z()),
        Rad64::QUARTER_TURN,
    )
    .unwrap();

    let revolved_faces = g
        .iter_faces()
        .filter(|(_, attr)| matches!(attr.surface, Surface::Revolution(_)))
        .collect::<Vec<_>>();

    assert_eq!(revolved_faces.len(), 3);
    for (face_key, attr) in revolved_faces {
        assert_eq!(attr.pcurves.len(), 4);
        let mesh = tessellate_face_key(&g, face_key, TessellateOpts::default())
            .expect("revolved face should tessellate from its pcurves");
        assert!(!mesh.is_empty());
    }
    assert_eq!(g.iter_solids().count(), 1);
}

#[test]
fn revolved_triangle_partial_turn_has_wedge_topology() {
    let mut g = GMap::<StandardPayload>::new();
    let profile_key = add_polygon(
        &mut g,
        &[
            Point3::new(0.75, 0.0, -0.85),
            Point3::new(1.85, 0.0, -0.05),
            Point3::new(0.85, 0.0, 0.9),
        ],
    );
    let source_face = add_face(&mut g, profile_key).unwrap();

    let solid = add_revolved_face(
        &mut g,
        source_face,
        Axis3::new(Point3::origin(), Vector3::z()),
        Rad64::QUARTER_TURN,
    )
    .unwrap();

    let vertices = g.iter_vertices().count();
    let edges = g.iter_edges().count();
    let faces = g.iter_faces().count();

    // A quarter-turn wedge from a triangle: two triangular caps joined by three
    // lateral surface-of-revolution faces. Nothing may be merged away.
    assert_eq!(
        (vertices, edges, faces),
        (6, 9, 5),
        "revolved triangle should keep 3 source and 3 rotated corners"
    );
    assert_eq!(
        vertices as i64 - edges as i64 + faces as i64,
        2,
        "the wedge shell should be a topological sphere"
    );

    // A mismatched alpha2 sew merges the rotated corners onto each other, so
    // check the surviving points are actually distinct in space too.
    let points = g
        .iter_vertices()
        .map(|(_, attr)| attr.point)
        .collect::<Vec<_>>();
    for (index, first) in points.iter().enumerate() {
        for second in points.iter().skip(index + 1) {
            assert!(
                !first.coincides(*second, LINEAR_TOLERANCE),
                "revolved wedge should not have coincident vertices: {first:?} and {second:?}"
            );
        }
    }

    validate_solid_manifold(&g, solid).expect("revolved wedge should be a closed manifold shell");
}

#[test]
fn revolved_wedge_faces_point_outward_for_either_profile_winding() {
    for winding in [[0, 1, 2], [2, 1, 0]] {
        let corners = [
            Point3::new(0.75, 0.0, -0.85),
            Point3::new(1.85, 0.0, -0.05),
            Point3::new(0.85, 0.0, 0.9),
        ];
        let mut g = GMap::<StandardPayload>::new();
        let profile_key = add_polygon(&mut g, &winding.map(|index| corners[index]));
        let source_face = add_face(&mut g, profile_key).unwrap();

        let solid = add_revolved_face(
            &mut g,
            source_face,
            Axis3::new(Point3::origin(), Vector3::z()),
            Rad64::QUARTER_TURN,
        )
        .unwrap();

        validate_solid_orientation(&g, solid).unwrap_or_else(|err| {
            panic!("wedge from a {winding:?} profile should face outward: {err}")
        });
    }
}

#[test]
fn revolved_annular_wedge_walls_face_away_from_the_material() {
    let mut g = GMap::<StandardPayload>::new();
    let profile_key = add_polygon(
        &mut g,
        &[
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 1.0),
            Point3::new(1.0, 0.0, 1.0),
        ],
    );
    let source_face = add_face(&mut g, profile_key).unwrap();
    add_revolved_face(
        &mut g,
        source_face,
        Axis3::new(Point3::origin(), Vector3::z()),
        Rad64::QUARTER_TURN,
    )
    .unwrap();

    // The two curved walls sit at fixed radii, so "outward" is unambiguous:
    // the outer wall must lean away from the axis and the inner wall towards
    // it, whatever the shell's centroid happens to be.
    let mut radial_signs = Vec::new();
    for (key, attr) in g.iter_faces() {
        if !matches!(attr.surface, Surface::Revolution(_)) {
            continue;
        }
        let face = ngk::topology::face::Face::new(&g, key);
        let (u, v) = (0.5, std::f64::consts::FRAC_PI_4);
        let point = attr.surface.point_at(u, v);
        let radius = (point.x * point.x + point.y * point.y).sqrt();
        if !(0.9..=2.1).contains(&radius) || (radius - 1.5).abs() < 0.4 {
            continue;
        }
        let normal = *face.normal_at(u, v);
        let outward_radial = normal.x * point.x + normal.y * point.y;
        radial_signs.push((radius, outward_radial));
    }

    assert_eq!(
        radial_signs.len(),
        2,
        "expected one inner and one outer wall"
    );
    for (radius, outward_radial) in radial_signs {
        if radius > 1.5 {
            assert!(
                outward_radial > LINEAR_TOLERANCE,
                "the outer wall at radius {radius} should face away from the axis, got {outward_radial}"
            );
        } else {
            assert!(
                outward_radial < -LINEAR_TOLERANCE,
                "the inner wall at radius {radius} should face the axis, got {outward_radial}"
            );
        }
    }
}

#[test]
fn revolved_face_full_turn_closes_its_seam() {
    let mut g = GMap::<StandardPayload>::new();
    let profile_key = add_polygon(
        &mut g,
        &[
            Point3::new(0.75, 0.0, -0.85),
            Point3::new(1.85, 0.0, -0.05),
            Point3::new(0.85, 0.0, 0.9),
        ],
    );
    let source_face = add_face(&mut g, profile_key).unwrap();

    let solid = add_revolved_face(
        &mut g,
        source_face,
        Axis3::new(Point3::origin(), Vector3::z()),
        Rad64::FULL_TURN,
    )
    .unwrap();

    // One lateral band per source edge, each closed onto itself at the seam.
    // The source face and its wire are interior to the solid and must be gone.
    assert_eq!(
        (
            g.iter_vertices().count(),
            g.iter_edges().count(),
            g.iter_faces().count()
        ),
        (3, 6, 3)
    );
    assert!(
        g.iter_faces()
            .all(|(_, attr)| matches!(attr.surface, Surface::Revolution(_))),
        "a full turn has no caps, so no planar source face may survive"
    );

    validate_solid_manifold(&g, solid).expect("a full turn should close its shell");
    validate_solid_orientation(&g, solid).expect("a full turn should face outward");
}

#[test]
fn revolving_an_edge_on_the_axis_is_rejected() {
    let mut g = GMap::<StandardPayload>::new();
    let edge_key = add_edge(
        &mut g,
        Point3::origin(),
        Point3::new(0.0, 0.0, 1.0),
        Curve::line(Point3::origin(), Point3::new(0.0, 0.0, 1.0)),
    )
    .expect("edge should build");

    let error = add_revolved_edge(
        &mut g,
        edge_key,
        Axis3::new(Point3::origin(), Vector3::z()),
        Rad64::QUARTER_TURN,
    )
    .expect_err("an edge on the axis sweeps no area");

    assert!(matches!(error, RevolveError::EdgeOnRevolutionAxis { .. }));
}

#[test]
fn partially_revolving_an_edge_touching_the_axis_is_rejected() {
    let mut g = GMap::<StandardPayload>::new();
    let edge_key = add_edge(
        &mut g,
        Point3::origin(),
        Point3::new(1.0, 0.0, 0.0),
        Curve::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0)),
    )
    .expect("edge should build");

    let error = add_revolved_edge(
        &mut g,
        edge_key,
        Axis3::new(Point3::origin(), Vector3::z()),
        Rad64::QUARTER_TURN,
    )
    .expect_err("a partial turn cannot build the apex yet");

    assert!(matches!(error, RevolveError::ApexRevolveUnsupported { .. }));
}
