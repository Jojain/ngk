use std::collections::HashMap;
use std::f64::consts::{FRAC_PI_2, PI};

use nalgebra::Vector3;
use radians::Rad64;

use ngk::builders::edges::{add_arc, add_edge};
use ngk::builders::revolve::add_revolved_edge;
use ngk::geometry::Axis2;
use ngk::geometry::axis::Axis3;
use ngk::geometry::{
    Circle, Curve, Frame, LINEAR_TOLERANCE, Plane, Point2, Point3, Sphere, Surface, TrimmedCurve2,
};
use ngk::model::Model;
use ngk::modeling::{faces, solids};
use ngk::topology::LoopKind;
use ngk::topology::attributes::{EdgeAttr, FaceAttr, ProfileAttr, VertexAttr};
use ngk::topology::gmap::{Dart, Dim};
use ngk::topology::payload::StandardPayload;
use ngk::topology::unwrapped_face_domain::UnwrappedFaceDomain;

/// A seam is a property of the unwrapped domain, so a planar face has none: no period,
/// nothing to cut, and the loop is already the polygon a winding test wants.
#[test]
fn a_planar_face_unwraps_without_a_cut() {
    let shape = faces::rectangle(Plane::xy(), 2.0, 3.0).expect("face should build");
    let domain = UnwrappedFaceDomain::of_face(&shape.face()).expect("planar face should unwrap");

    assert_eq!(domain.periods(), [None, None]);
    assert_eq!(domain.cut(Axis2::U), None);
    assert_eq!(domain.cut(Axis2::V), None);
    assert_eq!(domain.loops().len(), 1);
    assert_eq!(domain.images(ngk::geometry::Point2::new(0.5, 0.5)).len(), 1);
}

/// An annulus unwraps its hole as a second loop, in storage order.
#[test]
fn a_face_with_a_hole_unwraps_every_loop() {
    let shape = faces::annulus(Plane::xy(), 2.0, 1.0).expect("annulus should build");
    let domain = UnwrappedFaceDomain::of_face(&shape.face()).expect("annulus should unwrap");

    assert_eq!(domain.loops().len(), 2);
    for boundary in domain.loops() {
        assert!(!boundary.is_empty());
    }
}

/// The cylinder wall is where the cut lands today: its loop runs a whole period
/// in `u`, and the unwrapped domain says so rather than leaving a caller to notice.
#[test]
fn a_cylinder_wall_unwraps_one_whole_period_in_u() {
    let shape = solids::cylinder(1.0, 2.0).expect("cylinder should build");
    let wall = shape
        .solid()
        .faces()
        .into_iter()
        .find(|face| matches!(face.surface(), Surface::Cylinder(_)))
        .expect("cylinder should have a lateral face");
    let domain = UnwrappedFaceDomain::of_face(&wall).expect("cylinder wall should unwrap");

    assert_eq!(domain.period(Axis2::U), Some(2.0 * PI));
    assert_eq!(domain.period(Axis2::V), None);

    let (min, max) = domain.bounds();
    assert!(
        (max.x - min.x - 2.0 * PI).abs() < LINEAR_TOLERANCE,
        "the wall should span exactly one period in u, spans {}",
        max.x - min.x
    );
    assert_eq!(domain.cut(Axis2::U), Some(min.x));
    assert_eq!(domain.cut(Axis2::V), None);

    // A query at either end of the period is the same point on the surface, so
    // it must be asked on both branches.
    let images = domain.images(ngk::geometry::Point2::new(min.x, 1.0));
    assert!(
        images
            .iter()
            .any(|image| (image.x - max.x).abs() < LINEAR_TOLERANCE)
    );
}

/// The unwrapped domain's job: a loop arrives as one continuous polyline, never as pieces
/// a whole period apart. A jump of a period is exactly what a winding test
/// would read as a chord straight across the domain.
#[test]
fn an_unwrapped_loop_never_jumps_a_period() {
    let shape = solids::cylinder(1.0, 2.0).expect("cylinder should build");
    for face in shape.solid().faces() {
        let domain = UnwrappedFaceDomain::of_face(&face).expect("cylinder face should unwrap");
        let Some(period) = domain.period(Axis2::U) else {
            continue;
        };
        for boundary in domain.loops() {
            let polyline = boundary.polyline(16);
            for pair in polyline.windows(2) {
                assert!(
                    (pair[1].x - pair[0].x).abs() < period - LINEAR_TOLERANCE,
                    "unwrapped loop jumped {} in u, a period is {period}",
                    pair[1].x - pair[0].x
                );
            }
        }
    }
}

/// A pole collapses a whole row of the domain to one point, so a loop that
/// reaches one walks along it carrying no pcurve. The unwrapped domain records
/// the corner it turns through; without it the loop never closes.
///
/// The subject is a meridian revolved a full turn, not `solids::sphere`: a
/// sphere is now one boundaryless face, with no loop to turn through anything.
/// Revolving an arc whose two ends sit on the axis still sews a seam between
/// two poles, and is the shape this corner exists for.
#[test]
fn a_revolved_meridian_unwraps_the_poles_its_loop_turns_through() {
    let mut g = Model::<StandardPayload>::new();
    let meridian = add_arc(
        &mut g,
        Plane::from_xy(Point3::origin(), Vector3::x(), Vector3::z()),
        1.0,
        FRAC_PI_2,
        -FRAC_PI_2,
    )
    .expect("meridian arc should build");
    let face_key = add_revolved_edge(
        &mut g,
        meridian,
        Axis3::new(Point3::origin(), Vector3::z()),
        Rad64::FULL_TURN,
    )
    .expect("a full revolution of the meridian should build");
    let face = g.face_unchecked(face_key);
    let domain = UnwrappedFaceDomain::of_face(&face).expect("the revolved face should unwrap");
    let boundary = domain
        .loops()
        .first()
        .expect("the revolved face should have a loop");

    let corners = boundary
        .curves()
        .iter()
        .flat_map(|curve| curve.corners().iter().copied())
        .collect::<Vec<_>>();
    assert!(
        !corners.is_empty(),
        "the revolved meridian's seam loop should turn through at least one pole"
    );
    for corner in corners {
        assert!(
            face.surface().is_degenerate_at(corner.x, corner.y),
            "an unwrapped corner should sit on a degenerate row, found {corner:?}"
        );
    }
}

/// The seamless cylinder: no seam edge, and its wall bounded by two wrapping
/// loops that close only on the quotient.
#[test]
fn a_cylinder_wall_is_a_ring_face_with_no_seam() {
    let shape = solids::cylinder(1.0, 2.0).expect("cylinder should build");
    let map = shape.model();
    let solid = shape.solid();

    // Two rims of two darts each, two caps of two, and the four of the cut the
    // wall owns between its rims. The cut is what makes the wall one face
    // without an edge standing in for a seam, so it is darts and not an edge.
    assert_eq!(
        map.dart_count(),
        12,
        "a seamless cylinder should have 12 darts"
    );
    assert_eq!(solid.faces().len(), 3);
    assert_eq!(solid.edges().len(), 2, "the seam edge should be gone");

    let wall = solid
        .faces()
        .into_iter()
        .find(|face| matches!(face.surface(), Surface::Cylinder(_)))
        .expect("cylinder should have a lateral face");
    assert!(wall.outer_loop().is_none(), "a ring face has no outer loop");
    assert_eq!(
        wall.loops()
            .into_iter()
            .map(|loop_| loop_.kind())
            .collect::<Vec<_>>(),
        vec![
            LoopKind::Wrapping { axis: Axis2::U },
            LoopKind::Wrapping { axis: Axis2::U },
        ]
    );
    assert_eq!(
        wall.loops()
            .into_iter()
            .filter_map(|loop_| loop_.wrapping_axis())
            .collect::<Vec<_>>(),
        vec![Axis2::U, Axis2::U]
    );
}

/// A cap closes against its degenerate row, and the result is a real rectangle.
///
/// The loop alone leaves off one period from where it started, so on its own it
/// bounds nothing a winding test could read. Closing it out to the apex row and
/// back is what recovers the polygon a stored seam-and-pole-vertex used to spell
/// out — and the row's parameter comes from the support, not from the loop.
#[test]
fn a_capped_face_closes_its_domain_against_the_degenerate_row() {
    let mut g = Model::<StandardPayload>::new();
    let apex = Point3::origin();
    let rim = Point3::new(1.0, 0.0, 2.0);
    let edge = add_edge(&mut g, rim, apex, Curve::line(rim, apex)).expect("edge should build");
    let face_key = add_revolved_edge(
        &mut g,
        edge,
        Axis3::new(Point3::origin(), Vector3::z()),
        Rad64::FULL_TURN,
    )
    .expect("a cone should build");
    let face = g.face_unchecked(face_key);
    assert!(matches!(face.loops()[0].kind(), LoopKind::Capping { .. }));

    let domain = UnwrappedFaceDomain::of_face(&face).expect("a cap should unwrap");
    let boundary = domain.loops().first().expect("a cap has one boundary");

    // Three corners: the point the loop left off at, then out to the collapsed
    // row and back along it. The first is there because every pcurve drops its
    // final sample on the rule that the next one starts there — and here what
    // follows is the walk to the row, so without it the boundary cuts the
    // corner and the region loses a wedge.
    let corners = boundary
        .curves()
        .iter()
        .flat_map(|curve| curve.corners().iter().copied())
        .collect::<Vec<_>>();
    assert_eq!(corners.len(), 3);
    assert!(
        !face.surface().is_degenerate_at(corners[0].x, corners[0].y),
        "the first corner is where the loop ends, on the loop itself"
    );
    for corner in &corners[1..] {
        assert!(
            face.surface().is_degenerate_at(corner.x, corner.y),
            "the other two sit on the collapsed row, found {corner:?}"
        );
    }

    // The closed boundary spans a whole period one way and reaches the apex the
    // other: the rectangle [0, 2π] x [0, √5].
    let (min, max) = domain.bounds();
    assert!((max.x - min.x - 2.0 * PI).abs() < LINEAR_TOLERANCE);
    assert!((max.y - min.y - 5.0_f64.sqrt()).abs() < LINEAR_TOLERANCE);
}

/// A cap on the far half of a sphere unwraps onto one branch, not two.
///
/// A loop that turns through a pole gets no guidance from it: the pole is one
/// point on the surface, written at whatever parameter its pcurve happened to
/// name, so the placement rule refuses to align across it. What it does instead
/// is keep the offset the walk had accumulated — and that offset was chosen for
/// the pcurves *before* the pole.
///
/// This cap is where the two come apart. Its rim runs `u` from `pi` to `2pi`,
/// so the meridian climbing to the pole — stored at `u = 0` — is placed a whole
/// turn along to meet it. The meridian coming back down is stored at `u = pi`
/// and needs no turn at all, but inherited that one, landing at `3pi`. The
/// domain came back as a parallelogram twice the cap's width, and `ppp0110`'s
/// light cap drew a sheet hanging across the solid where the sphere should have
/// closed.
///
/// What the pole does not say, closure does: the loop must end where it began.
#[test]
fn a_cap_on_the_far_half_of_a_sphere_unwraps_onto_one_branch() {
    const LATITUDE: f64 = 0.2;
    let radius = 2.0;
    let sphere = Sphere::new(Frame::xyz(), radius);
    let surface = Surface::Sphere(sphere.clone());

    let mut g = Model::<StandardPayload>::new();
    let face_key = g
        .transaction(|edit| {
            let d: [Dart; 6] = std::array::from_fn(|_| edit.add_dart());
            for pair in 0..3 {
                edit.link(Dim::Zero, d[2 * pair], d[2 * pair + 1])?;
            }
            for pair in 0..3 {
                edit.link(Dim::One, d[2 * pair + 1], d[(2 * pair + 2) % 6])?;
            }

            // Where the rim starts, where it ends a half-turn later, and the
            // pole the two meridians meet at.
            edit.add_vertex(VertexAttr::new(d[0], sphere.point_at(PI, LATITUDE), ()));
            edit.add_vertex(VertexAttr::new(d[2], sphere.point_at(0.0, LATITUDE), ()));
            edit.add_vertex(VertexAttr::new(d[4], sphere.point_at(0.0, FRAC_PI_2), ()));

            // The rim is a latitude circle; each meridian is a great circle in
            // the plane its own longitude and the polar axis span.
            edit.add_edge(EdgeAttr::new(
                d[0],
                Curve::Circle(Circle::new(
                    Plane::from_xy(
                        Point3::new(0.0, 0.0, radius * LATITUDE.sin()),
                        Vector3::x(),
                        Vector3::y(),
                    ),
                    radius * LATITUDE.cos(),
                )),
                (),
            ));
            for (dart, longitude) in [(d[2], 0.0), (d[4], PI)] {
                let (sin, cos) = f64::sin_cos(longitude);
                edit.add_edge(EdgeAttr::new(
                    dart,
                    Curve::Circle(Circle::new(
                        Plane::from_xy(Point3::origin(), Vector3::new(cos, sin, 0.0), Vector3::z()),
                        radius,
                    )),
                    (),
                ));
            }
            edit.add_profile(ProfileAttr::new(d[0], ()));

            // The rim crosses the domain's far half, so the climb to the pole —
            // written at u = 0, as a writer naming the same meridian either way
            // may — has to be carried a turn along to meet it. The descent is
            // written at u = pi and must not be.
            let pcurves = HashMap::from([
                (
                    d[0],
                    TrimmedCurve2::segment(
                        Point2::new(PI, LATITUDE),
                        Point2::new(2.0 * PI, LATITUDE),
                    ),
                ),
                (
                    d[2],
                    TrimmedCurve2::segment(Point2::new(0.0, LATITUDE), Point2::new(0.0, FRAC_PI_2)),
                ),
                (
                    d[4],
                    TrimmedCurve2::segment(Point2::new(PI, FRAC_PI_2), Point2::new(PI, LATITUDE)),
                ),
            ]);
            let face = edit.add_face(FaceAttr::with_pcurves(
                surface.clone(),
                (),
                d[0],
                Vec::new(),
                pcurves,
            ));
            Ok::<_, ngk::topology::ModelEditError>(face)
        })
        .expect("a half cap should commit");

    let face = g.face(face_key).expect("the cap's face resolves");
    let domain = UnwrappedFaceDomain::of_face(&face).expect("a half cap should unwrap");
    let boundary = domain.loops().first().expect("a cap has one boundary");

    let (first, last) = (
        boundary.curves().first().expect("a placed pcurve"),
        boundary.curves().last().expect("a placed pcurve"),
    );
    assert!(
        (last.end() - first.start()).norm() <= LINEAR_TOLERANCE,
        "a loop written on one branch ends where it began, but it ran from \
         {:?} to {:?}",
        first.start(),
        last.end()
    );

    let polyline = boundary.polyline(8);
    let u_min = polyline.iter().map(|p| p.x).fold(f64::INFINITY, f64::min);
    let u_max = polyline
        .iter()
        .map(|p| p.x)
        .fold(f64::NEG_INFINITY, f64::max);
    assert!(
        (u_max - u_min - PI).abs() <= LINEAR_TOLERANCE,
        "the cap covers the half of the domain its rim runs, so its boundary \
         spans pi — not the {:.4} a second branch would add",
        u_max - u_min
    );
}

/// A sphere written with a seam keeps its two sides a period apart.
///
/// This is the case the correction in
/// [`a_cap_on_the_far_half_of_a_sphere_unwraps_onto_one_branch`] must not touch,
/// and it looks identical from where closure stands: the loop runs up one side
/// of the cut and down the other, so it too ends a whole period from where it
/// began. The difference is what pulling those ends together would do — here it
/// lands the two sides on each other and the domain collapses from the whole
/// sphere to a line, which is why the correction is kept only when the loop
/// still bounds something afterwards.
#[test]
fn a_seamed_sphere_keeps_the_cut_its_loop_closes_across() {
    let mut g = Model::<StandardPayload>::new();
    let meridian = add_arc(
        &mut g,
        Plane::from_xy(Point3::origin(), Vector3::x(), Vector3::z()),
        1.0,
        FRAC_PI_2,
        -FRAC_PI_2,
    )
    .expect("meridian arc should build");
    let face_key = add_revolved_edge(
        &mut g,
        meridian,
        Axis3::new(Point3::origin(), Vector3::z()),
        Rad64::FULL_TURN,
    )
    .expect("a full revolution of the meridian should build");
    let face = g.face_unchecked(face_key);
    let domain = UnwrappedFaceDomain::of_face(&face).expect("the revolved face should unwrap");
    let boundary = domain
        .loops()
        .first()
        .expect("the seamed sphere has a loop");

    // A surface of revolution carries its period in `v`, the sweep angle; `u`
    // runs the profile and does not close.
    let period = domain
        .period(Axis2::V)
        .expect("a full revolution closes in its sweep");
    let polyline = boundary.polyline(8);
    let v_min = polyline.iter().map(|p| p.y).fold(f64::INFINITY, f64::min);
    let v_max = polyline
        .iter()
        .map(|p| p.y)
        .fold(f64::NEG_INFINITY, f64::max);
    assert!(
        (v_max - v_min - period).abs() <= LINEAR_TOLERANCE,
        "the two sides of the cut are a period apart, spanning the whole sweep \
         — they spanned {:.4} of {period:.4}",
        v_max - v_min
    );
}
