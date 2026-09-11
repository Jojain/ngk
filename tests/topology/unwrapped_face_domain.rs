use std::f64::consts::{FRAC_PI_2, PI};

use nalgebra::Vector3;
use radians::Rad64;

use ngk::builders::edges::{add_arc, add_edge};
use ngk::builders::revolve::add_revolved_edge;
use ngk::geometry::Axis2;
use ngk::geometry::axis::Axis3;
use ngk::geometry::{Curve, LINEAR_TOLERANCE, Plane, Point3, Surface};
use ngk::modeling::{faces, solids};
use ngk::topology::LoopKind;
use ngk::topology::gmap::GMap;
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
    let mut g = GMap::<StandardPayload>::new();
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
    let map = shape.map();
    let solid = shape.solid();

    assert_eq!(
        map.dart_count(),
        8,
        "a seamless cylinder should have 8 darts"
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
    let mut g = GMap::<StandardPayload>::new();
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
