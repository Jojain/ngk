use std::f64::consts::PI;

use ngk::geometry::Axis2;
use ngk::geometry::{LINEAR_TOLERANCE, Plane, Surface};
use ngk::modeling::{faces, solids};
use ngk::topology::chart::Chart;

/// A seam is a property of the chart, so a planar face has none: no period,
/// nothing to cut, and the loop is already the polygon a winding test wants.
#[test]
fn a_planar_face_charts_without_a_cut() {
    let shape = faces::rectangle(Plane::xy(), 2.0, 3.0).expect("face should build");
    let chart = Chart::of_face(&shape.face()).expect("planar face should chart");

    assert_eq!(chart.periods(), [None, None]);
    assert_eq!(chart.cut(Axis2::U), None);
    assert_eq!(chart.cut(Axis2::V), None);
    assert_eq!(chart.loops().len(), 1);
    assert_eq!(chart.images(ngk::geometry::Point2::new(0.5, 0.5)).len(), 1);
}

/// An annulus charts its hole as a second loop, in storage order.
#[test]
fn a_face_with_a_hole_charts_every_loop() {
    let shape = faces::annulus(Plane::xy(), 2.0, 1.0).expect("annulus should build");
    let chart = Chart::of_face(&shape.face()).expect("annulus should chart");

    assert_eq!(chart.loops().len(), 2);
    for boundary in chart.loops() {
        assert!(!boundary.is_empty());
    }
}

/// The cylinder wall is where the cut lands today: its loop runs a whole period
/// in `u`, and the chart says so rather than leaving a caller to notice.
#[test]
fn a_cylinder_wall_charts_one_whole_period_in_u() {
    let shape = solids::cylinder(1.0, 2.0).expect("cylinder should build");
    let wall = shape
        .solid()
        .faces()
        .into_iter()
        .find(|face| matches!(face.surface(), Surface::Cylinder(_)))
        .expect("cylinder should have a lateral face");
    let chart = Chart::of_face(&wall).expect("cylinder wall should chart");

    assert_eq!(chart.period(Axis2::U), Some(2.0 * PI));
    assert_eq!(chart.period(Axis2::V), None);

    let (min, max) = chart.bounds();
    assert!(
        (max.x - min.x - 2.0 * PI).abs() < LINEAR_TOLERANCE,
        "the wall should span exactly one period in u, spans {}",
        max.x - min.x
    );
    assert_eq!(chart.cut(Axis2::U), Some(min.x));
    assert_eq!(chart.cut(Axis2::V), None);

    // A query at either end of the period is the same point on the surface, so
    // it must be asked on both branches.
    let images = chart.images(ngk::geometry::Point2::new(min.x, 1.0));
    assert!(
        images
            .iter()
            .any(|image| (image.x - max.x).abs() < LINEAR_TOLERANCE)
    );
}

/// The chart's job: a loop arrives as one continuous polyline, never as pieces
/// a whole period apart. A jump of a period is exactly what a winding test
/// would read as a chord straight across the domain.
#[test]
fn a_charted_loop_never_jumps_a_period() {
    let shape = solids::cylinder(1.0, 2.0).expect("cylinder should build");
    for face in shape.solid().faces() {
        let chart = Chart::of_face(&face).expect("cylinder face should chart");
        let Some(period) = chart.period(Axis2::U) else {
            continue;
        };
        for boundary in chart.loops() {
            let polyline = boundary.polyline(16);
            for pair in polyline.windows(2) {
                assert!(
                    (pair[1].x - pair[0].x).abs() < period - LINEAR_TOLERANCE,
                    "charted loop jumped {} in u, a period is {period}",
                    pair[1].x - pair[0].x
                );
            }
        }
    }
}

/// A sphere's poles collapse a whole row of the domain to one point, so the
/// loop walks along one carrying no pcurve. The chart records the corner it
/// turns through; without it the loop never closes.
#[test]
fn a_sphere_charts_the_poles_its_loop_turns_through() {
    let shape = solids::sphere(1.0).expect("sphere should build");
    let face = shape
        .solid()
        .faces()
        .into_iter()
        .next()
        .expect("sphere should have a face");
    let chart = Chart::of_face(&face).expect("sphere face should chart");
    let boundary = chart
        .loops()
        .first()
        .expect("sphere face should have a loop");

    let corners = boundary
        .curves()
        .iter()
        .filter_map(|curve| curve.corner())
        .collect::<Vec<_>>();
    assert!(
        !corners.is_empty(),
        "the sphere's seam loop should turn through at least one pole"
    );
    for corner in corners {
        assert!(
            face.surface().is_degenerate_at(corner.x, corner.y),
            "a charted corner should sit on a degenerate row, found {corner:?}"
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
    let boundary = wall.boundary();

    assert_eq!(boundary.outer(), None, "a ring face has no outer loop");
    assert!(boundary.is_ring());
    assert_eq!(
        boundary
            .wrapping()
            .map(|(_, axis)| axis)
            .collect::<Vec<_>>(),
        vec![Axis2::U, Axis2::U]
    );
}
