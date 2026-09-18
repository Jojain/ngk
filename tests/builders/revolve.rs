use std::collections::HashSet;

use nalgebra::Vector3;
use radians::Rad64;

use ngk::builders::edges::{add_circle, add_edge, split_edge};
use ngk::builders::faces::{add_face, add_polygon};
use ngk::builders::revolve::{RevolveError, add_revolved_edge, add_revolved_face};
use ngk::geometry::axis::Axis3;
use ngk::geometry::{
    Axis2, Curve, Curve2, Fraction, LINEAR_TOLERANCE, Point3, PointCoincidence, Surface,
};
use ngk::model::Model;
use ngk::tessellate::{TessellateOpts, tessellate_face_key};
use ngk::topology::LoopKind;
use ngk::topology::payload::StandardPayload;
use ngk::topology::validation::{
    validate_all_solid_manifolds, validate_gmap, validate_solid_manifold,
    validate_solid_orientation,
};

#[test]
fn revolve_edge_partial_turn_creates_four_edge_face() {
    let mut g = Model::<StandardPayload>::new();
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
    let boundary_edges = face
        .outer_loop()
        .expect("face should have an outer loop")
        .edges();
    let boundary_vertices = face
        .outer_loop()
        .expect("face should have an outer loop")
        .vertices();
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
    let mut g = Model::<StandardPayload>::new();
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
    let mut g = Model::<StandardPayload>::new();
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
    g: &Model<StandardPayload>,
    face_key: ngk::topology::shape_keys::FaceKey,
    origin: Point3,
) -> Point3 {
    let arc = g
        .face_unchecked(face_key)
        .edges()
        .into_iter()
        .find(|edge| {
            let start = *edge.bounded_unchecked().start().point();
            let end = *edge.bounded_unchecked().end().point();
            matches!(edge.curve(), Curve::Circle(_))
                && (start.coincides(origin, LINEAR_TOLERANCE)
                    || end.coincides(origin, LINEAR_TOLERANCE))
        })
        .expect("revolve should create a circular side arc");
    let curve = arc.curve();
    let interval = arc.parameter_interval();
    curve.point_at(interval.at(Fraction::new(0.5)))
}

/// A full turn of a *marked* profile is a different shape, so it is refused.
///
/// The corner sweeps a circle, and that circle bounds the result — so the
/// boundaryless torus below is not the answer for this profile. Consuming the
/// source loop as if it were unmarked would delete a corner the caller put there
/// on purpose and hand back a shape nobody asked for, so the gap is named
/// instead.
#[test]
fn revolve_marked_closed_edge_full_turn_is_refused() {
    let mut g = Model::<StandardPayload>::new();
    let profile = ngk::geometry::Plane::new(Point3::new(3.0, 0.0, 0.0), Vector3::x(), Vector3::y());
    let circle = add_circle(&mut g, profile, 1.0).expect("profile circle should build");
    split_edge(&mut g, circle, Fraction::new(0.5)).expect("the profile takes a corner");
    assert_eq!(g.iter_vertices().count(), 1, "the profile is marked");

    let refused = add_revolved_edge(
        &mut g,
        circle,
        Axis3::new(Point3::origin(), Vector3::z()),
        Rad64::FULL_TURN,
    );

    assert!(
        matches!(&refused, Err(RevolveError::MarkedProfileRevolve { key }) if *key == circle),
        "a marked profile sweeps a bounded torus, got {refused:?}",
    );
    assert_eq!(
        g.iter_vertices().count(),
        1,
        "and the corner it refused over is still there",
    );
}

/// A full turn of a closed profile sweeps a torus: one face bounded by nothing.
///
/// The source circle is not reused as a boundary the way an open profile's ends
/// are — a torus has no boundary for it to become — so it is consumed outright.
/// What the face is left standing on is the polygon schema of its own support:
/// the eight-dart square with both pairs of opposite sides identified, every
/// cell of it embedded in the face and none of it a logical edge or vertex.
#[test]
fn revolve_closed_edge_full_turn_sweeps_a_boundaryless_torus() {
    let mut g = Model::<StandardPayload>::new();
    let profile = ngk::geometry::Plane::new(Point3::new(3.0, 0.0, 0.0), Vector3::x(), Vector3::y());
    let circle = add_circle(&mut g, profile, 1.0).expect("profile circle should build");

    let face_key = add_revolved_edge(
        &mut g,
        circle,
        Axis3::new(Point3::origin(), Vector3::z()),
        Rad64::FULL_TURN,
    )
    .expect("a circle off the axis should revolve into a torus");

    assert_eq!(
        (
            g.dart_count(),
            g.iter_vertices().count(),
            g.iter_edges().count(),
            g.iter_profiles().count(),
            g.iter_faces().count()
        ),
        (8, 0, 0, 0, 1),
        "the source loop is consumed, and the face stands on the square a whole          torus is"
    );
    let face = g.face_unchecked(face_key);
    assert!(face.loops().is_empty(), "a torus face has no boundary loop");
    assert!(
        matches!(face.surface(), Surface::Revolution(_)),
        "a revolved circle keeps the swept support"
    );
}

/// A closed profile crossing the axis pinches its sweep, and is refused.
///
/// The refusal is the support's own answer — it reports a row where its
/// parameterization collapses — rather than a separate intersection here.
#[test]
fn revolve_closed_edge_crossing_the_axis_is_refused() {
    let mut g = Model::<StandardPayload>::new();
    let profile = ngk::geometry::Plane::new(Point3::origin(), Vector3::x(), Vector3::y());
    let circle = add_circle(&mut g, profile, 1.0).expect("profile circle should build");

    assert!(
        matches!(
            add_revolved_edge(
                &mut g,
                circle,
                Axis3::new(Point3::origin(), Vector3::z()),
                Rad64::FULL_TURN,
            ),
            Err(RevolveError::EdgeOnRevolutionAxis { .. })
        ),
        "a profile straddling the axis sweeps no torus"
    );
}

/// A full turn of an off-axis edge sweeps two circles, and both bound the band.
///
/// Neither is a hole: in the support's parameters the band covers the whole turn,
/// so both loops wrap it. `revolve_edge_full_turn_bounds_its_band_with_wrapping_loops`
/// checks the kinds; this checks that the cells are shared rather than duplicated.
#[test]
fn revolve_edge_full_turn_sweeps_two_distinct_circles() {
    let mut g = Model::<StandardPayload>::new();
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

/// An edge with one end on the axis sweeps only one circle.
///
/// The other end sweeps nothing, so that side of the band is closed by the
/// degeneracy rather than by a loop, and the band has a single boundary.
#[test]
fn revolve_edge_full_turn_with_an_end_on_the_axis_has_one_loop() {
    let mut g = Model::<StandardPayload>::new();
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
    // A line meeting the axis at right angles keeps every point at its own
    // height, so it sweeps a *plane*: the rim closes in that plane's own
    // parameters, which makes it a genuine outer loop rather than a cap. What
    // decides the kind is always the support's parameterization — a spherical
    // cap is just as flat a disk in space, and is a cap because a sphere's
    // parameters collapse at its centre.
    assert!(matches!(face.surface(), Surface::Plane(_)));
    assert!(matches!(face.loops()[0].kind(), LoopKind::Outer));
}

/// A slanted edge from the axis sweeps a cone, and its rim is that cone's cap.
///
/// The support is periodic in the sweep, so the rim runs a whole period and
/// closes only on the quotient — it has no inside of its own. What bounds the
/// face on the far side is the apex, a parametric degeneracy rather than a loop.
/// The apex sits *inside* an unbounded parameter direction rather than at an end
/// of it, which is why the loop records only which side it is on and the support
/// is asked where: `Cone::apex_parameter`, through `Surface::degenerate_rows`.
///
/// Written rim-first deliberately: `revolved_support` needs a non-degenerate
/// start radius to build a cone's frame, so an apex-first line falls through to
/// a generic surface of revolution. Both are capped, but only this one is a
/// `Cone`.
#[test]
fn revolve_edge_full_turn_from_the_axis_caps_a_cone() {
    let mut g = Model::<StandardPayload>::new();
    let apex = Point3::origin();
    let rim = Point3::new(1.0, 0.0, 2.0);
    let edge_key = add_edge(&mut g, rim, apex, Curve::line(rim, apex)).expect("edge should build");

    let face_key = add_revolved_edge(
        &mut g,
        edge_key,
        Axis3::new(Point3::origin(), Vector3::z()),
        Rad64::FULL_TURN,
    )
    .unwrap();
    let face = g.face_unchecked(face_key);

    assert!(matches!(face.surface(), Surface::Cone(_)));
    assert_eq!(face.loops().len(), 1);
    assert!(
        matches!(face.loops()[0].kind(), LoopKind::Capping { .. }),
        "a cone's rim is a cap, not an outer loop: {:?}",
        face.loops()[0].kind()
    );
    assert!(face.outer_loop().is_none());

    // The support locates the apex: one generatrix length from the rim, which
    // sits at v = 0.
    let rows = face.surface().degenerate_rows(Axis2::V);
    assert_eq!(rows.len(), 1);
    assert!((rows[0].abs() - 5.0_f64.sqrt()).abs() < LINEAR_TOLERANCE);
}

/// The same band written apex-first is a surface of revolution, and is capped
/// too — there the row comes from intersecting the profile with the axis.
#[test]
fn revolve_edge_full_turn_from_the_axis_caps_a_surface_of_revolution() {
    let mut g = Model::<StandardPayload>::new();
    let apex = Point3::origin();
    let rim = Point3::new(1.0, 0.0, 2.0);
    let edge_key = add_edge(&mut g, apex, rim, Curve::line(apex, rim)).expect("edge should build");

    let face_key = add_revolved_edge(
        &mut g,
        edge_key,
        Axis3::new(Point3::origin(), Vector3::z()),
        Rad64::FULL_TURN,
    )
    .unwrap();
    let face = g.face_unchecked(face_key);

    assert!(matches!(face.surface(), Surface::Revolution(_)));
    assert!(
        matches!(face.loops()[0].kind(), LoopKind::Capping { .. }),
        "the apex closes this band: {:?}",
        face.loops()[0].kind()
    );
    // The profile meets the axis at its own parameter 0, which is this
    // surface's `u`.
    let rows = face.surface().degenerate_rows(Axis2::U);
    assert_eq!(rows.len(), 1);
    assert!(rows[0].abs() < LINEAR_TOLERANCE);
}

#[test]
fn revolved_face_adds_surface_of_revolution_faces() {
    let mut g = Model::<StandardPayload>::new();
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
        .filter(|(_, attr)| is_swept_support(&attr.surface))
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
    let mut g = Model::<StandardPayload>::new();
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
        let mut g = Model::<StandardPayload>::new();
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
    let mut g = Model::<StandardPayload>::new();
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
        if !is_swept_support(&attr.surface) {
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
fn revolved_face_full_turn_bands_are_rings() {
    let mut g = Model::<StandardPayload>::new();
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

    // One lateral band per source edge, each a ring bounded by the two circles
    // its endpoints swept — and by nothing else, since a whole turn brings the
    // swept copy of the source edge back onto the source edge. There is no seam
    // to sew, so the only edges are the three swept circles.
    //
    // The source face and its wire are interior to the solid and must be gone.
    assert_eq!(
        (
            g.iter_vertices().count(),
            g.iter_edges().count(),
            g.iter_faces().count()
        ),
        (3, 3, 3)
    );
    assert!(
        g.iter_faces()
            .all(|(_, attr)| is_swept_support(&attr.surface)),
        "a full turn has no caps, so no planar source face may survive"
    );
    assert!(
        g.iter_faces().all(|(_, attr)| {
            let face = attr.face(&g);
            face.outer_loop().is_none()
                && face
                    .loops()
                    .into_iter()
                    .filter_map(|loop_| loop_.wrapping_axis())
                    .count()
                    == 2
        }),
        "every band is bounded by two wrapping loops and no outer loop"
    );
    // A torus, read with each band counted for what it is: an annulus, not a
    // disk. Three vertices, three edges, three faces of Euler characteristic 0.
    assert_eq!(
        g.iter_vertices().count() as isize - g.iter_edges().count() as isize,
        0,
        "V - E + sum(chi) = 0, the characteristic of a torus"
    );

    validate_solid_manifold(&g, solid).expect("a full turn should close its shell");
    validate_solid_orientation(&g, solid).expect("a full turn should face outward");
}

#[test]
fn revolving_an_edge_on_the_axis_is_rejected() {
    let mut g = Model::<StandardPayload>::new();
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
    let mut g = Model::<StandardPayload>::new();
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

/// Whether a face's support is one a revolution sweeps out.
///
/// A straight profile edge parallel to the axis sweeps a cylinder and one
/// meeting the axis at an angle sweeps a cone, so the walls of a revolution
/// are not all `Surface::Revolution`.
fn is_swept_support(surface: &Surface) -> bool {
    matches!(
        surface,
        Surface::Revolution(_) | Surface::Cylinder(_) | Surface::Cone(_)
    )
}

/// A full turn of a band whose support is periodic in the sweep wraps; it does
/// not carve a hole.
///
/// In such a support's own parameters the band is a rectangle covering the whole
/// turn, so neither boundary circle closes there and neither bounds the other.
/// Which circle is the wider one in space is not a fact about the domain, and
/// calling the narrower one a hole would put a winding test on a loop that has
/// no inside. A planar washer is the exception and gets its own test: its
/// support is not periodic at all, and both circles close in its parameters.
#[test]
fn revolve_edge_full_turn_bounds_its_band_with_wrapping_loops() {
    for (start, end) in [
        // A segment parallel to the axis: a cylinder wall.
        (Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 2.0)),
        // A slanted segment: a cone frustum.
        (Point3::new(1.0, 0.0, 0.0), Point3::new(2.0, 0.0, 2.0)),
    ] {
        let mut g = Model::<StandardPayload>::new();
        let edge =
            add_edge(&mut g, start, end, Curve::line(start, end)).expect("edge should build");
        let face = add_revolved_edge(
            &mut g,
            edge,
            Axis3::new(Point3::origin(), Vector3::z()),
            Rad64::FULL_TURN,
        )
        .expect("a full turn should build");

        let face = g.face_unchecked(face);
        assert_eq!(face.loops().len(), 2, "{start:?} -> {end:?}");
        assert!(
            face.outer_loop().is_none(),
            "{start:?} -> {end:?} should have no outer loop"
        );
        assert_eq!(
            face.loops()
                .into_iter()
                .filter_map(|loop_| loop_.wrapping_axis())
                .count(),
            2,
            "{start:?} -> {end:?}: both swept circles wrap the turn"
        );
    }
}

/// A segment perpendicular to the axis sweeps a plane, not a surface of
/// revolution — so its two circles really do bound an annulus.
///
/// Every point of such a profile keeps its height, so the sweep never leaves the
/// plane at that height. That plane is the honest support, and in its own
/// Cartesian parameters the two swept circles are circles: closed, with the
/// wider one outside the narrower. This is the one revolved band where a hole is
/// the right answer, and it is the support's parameterization that says so.
#[test]
fn revolve_edge_full_turn_perpendicular_to_the_axis_sweeps_a_planar_annulus() {
    let mut g = Model::<StandardPayload>::new();
    let (start, end) = (Point3::new(1.0, 0.0, 3.0), Point3::new(2.0, 0.0, 3.0));
    let edge = add_edge(&mut g, start, end, Curve::line(start, end)).expect("edge should build");
    let face_key = add_revolved_edge(
        &mut g,
        edge,
        Axis3::new(Point3::origin(), Vector3::z()),
        Rad64::FULL_TURN,
    )
    .expect("a full turn should build");
    let face = g.face_unchecked(face_key);

    assert!(matches!(face.surface(), Surface::Plane(_)));
    assert_eq!(face.loops().len(), 2);
    assert!(face.outer_loop().is_some(), "the wider circle bounds it");
    assert_eq!(face.inner_loops().len(), 1, "the narrower circle is a hole");
    assert_eq!(
        face.loops()
            .into_iter()
            .filter_map(|loop_| loop_.wrapping_axis())
            .count(),
        0,
        "a plane has no period for a loop to wrap"
    );

    // Both pcurves are circles, not chords. This is the whole reason a plane
    // could not be recognized before: a segment between two mapped corners is
    // the exact image of a swept circle in a cylinder's or a cone's parameters,
    // and a chord across it in a plane's.
    for edge in face.edges() {
        let pcurve = face.pcurve(edge.dart()).expect("every edge carries one");
        assert!(
            matches!(pcurve.curve(), Curve2::Circle(_)),
            "a swept circle stays a circle in a plane's parameters"
        );
    }

    // Every pcurve tracks its own edge all the way round, the hole's included.
    // A chord would fail this at every fraction but the two ends, and so would a
    // hole whose pcurve ran against its dart: the hole winds against the outer
    // loop by sweeping its own 3D circle backwards, which is what keeps the two
    // in step here.
    for edge in face.edges() {
        let pcurve = face.pcurve(edge.dart()).expect("every edge carries one");
        let section = edge.trimmed_curve();
        for fraction in [0.0, 0.25, 0.5, 0.75] {
            let uv = pcurve.point_at(Fraction::new(fraction));
            assert!(
                face.point_at(uv.x, uv.y)
                    .coincides(section.point_at(Fraction::new(fraction)), LINEAR_TOLERANCE),
                "pcurve left its edge at {fraction}"
            );
        }
    }
}

/// A rectangle offset from the axis sweeps a tube, and its two radial edges
/// sweep planar annuli rather than slit disks.
///
/// A band whose swept support is periodic comes out a ring with no seam, but a
/// radial edge sweeps a *plane*, which has no period, and the same band used to
/// fall back to a quad: a copy of the source edge at each end of a whole turn,
/// two distinct edges lying on the same segment and alpha2-free, leaving the
/// shell open. The two swept circles bound the band on their own, exactly as
/// they do for a single revolved edge.
#[test]
fn revolve_face_full_turn_of_an_offset_rectangle_closes_its_shell() {
    let mut g = Model::<StandardPayload>::new();
    let profile_key = add_polygon(
        &mut g,
        &[
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(5.0, 0.0, 0.0),
            Point3::new(5.0, 0.0, 10.0),
            Point3::new(2.0, 0.0, 10.0),
        ],
    );
    let source_face = add_face(&mut g, profile_key).unwrap();

    let solid = add_revolved_face(
        &mut g,
        source_face,
        Axis3::new(Point3::origin(), Vector3::z()),
        Rad64::FULL_TURN,
    )
    .expect("a full turn should build");

    // Four bands, one per source edge, bounded by the four circles the corners
    // swept — and by nothing else. A seam on either annulus would show up as a
    // fifth edge and as vertices the circles do not need.
    assert_eq!(
        (
            g.iter_vertices().count(),
            g.iter_edges().count(),
            g.iter_faces().count()
        ),
        (4, 4, 4)
    );

    // The two annuli are planar and each has a genuine hole; the two walls are
    // cylindrical rings with no outer loop at all.
    let annuli = g
        .iter_faces()
        .filter(|(_, attr)| matches!(attr.surface, Surface::Plane(_)))
        .map(|(key, _)| key)
        .collect::<Vec<_>>();
    assert_eq!(annuli.len(), 2, "the two radial edges sweep planes");
    for key in annuli {
        let face = g.face_unchecked(key);
        assert!(face.outer_loop().is_some(), "the wider circle bounds it");
        assert_eq!(face.inner_loops().len(), 1, "the narrower circle is a hole");
    }

    // Every pcurve tracks its own edge all the way round, on the annuli and on
    // the walls alike: a band's two circles run against each other so the face
    // lies to the left of both, and the one wound backwards has its own 3D
    // circle swept backwards to match.
    for (key, _) in g.iter_faces() {
        let face = g.face_unchecked(key);
        for edge in face.edges() {
            let pcurve = face.pcurve(edge.dart()).expect("every edge carries one");
            let section = edge.trimmed_curve();
            for fraction in [0.0, 0.25, 0.5, 0.75] {
                let uv = pcurve.point_at(Fraction::new(fraction));
                assert!(
                    face.point_at(uv.x, uv.y)
                        .coincides(section.point_at(Fraction::new(fraction)), LINEAR_TOLERANCE),
                    "pcurve left its edge at {fraction}"
                );
            }
        }
    }

    validate_gmap(g.topology()).expect("a full turn should stay a valid map");
    validate_all_solid_manifolds(&g).expect("a full turn should close its shell");
    validate_solid_manifold(&g, solid).expect("a full turn should close its shell");
    validate_solid_orientation(&g, solid).expect("a full turn should face outward");
}
