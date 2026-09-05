use ngk::geometry::{Curve, Interval, LINEAR_TOLERANCE, Plane, Point3, Surface};
use ngk::healing::predicates::{SurfaceMatch, join_curves, surfaces_match};
use std::f64::consts::FRAC_PI_2;

const ANGULAR: f64 = 1.0e-9;

fn split(curve: &Curve, at: f64) -> (Curve, Curve) {
    (
        curve
            .trimmed(Interval::new(0.0, at))
            .expect("first half should trim"),
        curve
            .trimmed(Interval::new(at, 1.0))
            .expect("second half should trim"),
    )
}

#[test]
fn the_two_halves_of_a_split_line_rejoin_into_the_original_span() {
    let start = Point3::new(0.0, 0.0, 0.0);
    let end = Point3::new(4.0, 0.0, 0.0);
    let line = Curve::line(start, end);
    let (first, second) = split(&line, 0.25);
    let through = line.point_at(0.25);

    let joined = join_curves(
        &first,
        &second,
        start,
        through,
        end,
        LINEAR_TOLERANCE,
        ANGULAR,
    )
    .expect("halves of one line should rejoin");
    assert!((joined.point_at(0.0) - start).norm() <= LINEAR_TOLERANCE);
    assert!((joined.point_at(1.0) - end).norm() <= LINEAR_TOLERANCE);
    assert!((joined.length(0.0, 1.0) - 4.0).abs() <= 1.0e-9);
}

#[test]
fn the_two_halves_of_a_split_arc_rejoin_onto_the_same_circle() {
    let plane = Plane::xy();
    let arc = Curve::arc(plane, 2.0, Interval::new(0.0, FRAC_PI_2));
    let start = arc.point_at(0.0);
    let through = arc.point_at(0.4);
    let end = arc.point_at(1.0);
    let (first, second) = split(&arc, 0.4);

    let joined = join_curves(&first, &second, start, through, end, 1.0e-7, 1.0e-7)
        .expect("halves of one arc should rejoin");
    for step in 0..=8 {
        let point = joined.point_at(step as f64 / 8.0);
        assert!(
            (point.coords.norm() - 2.0).abs() <= 1.0e-6,
            "the fused arc should stay on the original circle"
        );
        assert!(point.z.abs() <= 1.0e-6, "the fused arc should stay planar");
    }
}

#[test]
fn two_segments_meeting_at_an_angle_do_not_rejoin() {
    let start = Point3::new(0.0, 0.0, 0.0);
    let through = Point3::new(1.0, 0.0, 0.0);
    let end = Point3::new(1.0, 1.0, 0.0);
    let first = Curve::line(start, through);
    let second = Curve::line(through, end);

    assert!(
        join_curves(
            &first,
            &second,
            start,
            through,
            end,
            LINEAR_TOLERANCE,
            ANGULAR
        )
        .is_none(),
        "a corner carries shape and must not be fused away"
    );
}

#[test]
fn a_doubling_back_pair_does_not_rejoin() {
    let start = Point3::new(0.0, 0.0, 0.0);
    let through = Point3::new(2.0, 0.0, 0.0);
    let end = Point3::new(1.0, 0.0, 0.0);
    let first = Curve::line(start, through);
    let second = Curve::line(through, end);

    assert!(
        join_curves(
            &first,
            &second,
            start,
            through,
            end,
            LINEAR_TOLERANCE,
            ANGULAR
        )
        .is_none(),
        "the shared vertex is not interior to the fused span"
    );
}

#[test]
fn one_plane_matches_itself_identically() {
    let surface = Surface::Plane(Plane::xy());
    assert_eq!(
        surfaces_match(&surface, &surface, LINEAR_TOLERANCE, ANGULAR),
        Some(SurfaceMatch::Identical)
    );
}

#[test]
fn coplanar_planes_with_different_frames_match_as_coplanar() {
    let first = Surface::Plane(Plane::xy());
    let second = Surface::Plane(Plane::new(
        Point3::new(3.0, 1.0, 0.0),
        nalgebra::Vector3::new(0.0, 1.0, 0.0),
        nalgebra::Vector3::z(),
    ));
    assert_eq!(
        surfaces_match(&first, &second, LINEAR_TOLERANCE, ANGULAR),
        Some(SurfaceMatch::Coplanar)
    );
}

#[test]
fn parallel_planes_at_different_heights_do_not_match() {
    let first = Surface::Plane(Plane::xy());
    let second = Surface::Plane(Plane::new(
        Point3::new(0.0, 0.0, 1.0),
        nalgebra::Vector3::x(),
        nalgebra::Vector3::z(),
    ));
    assert!(surfaces_match(&first, &second, LINEAR_TOLERANCE, ANGULAR).is_none());
}

#[test]
fn a_plane_and_a_cylinder_do_not_match() {
    let plane = Surface::Plane(Plane::xy());
    let cylinder = Surface::Cylinder(ngk::geometry::Cylinder::new(
        Point3::origin(),
        nalgebra::Vector3::x(),
        nalgebra::Vector3::z(),
        1.0,
    ));
    assert!(surfaces_match(&plane, &cylinder, LINEAR_TOLERANCE, ANGULAR).is_none());
}
