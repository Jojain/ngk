use std::f64::consts::{FRAC_PI_2, PI, TAU};

use ngk::geometry::{
    Curve, Frame, Interval, LINEAR_TOLERANCE, Plane, Point3, PointCoincidence, TrimmedCurve,
};

fn unit_circle() -> Curve {
    Curve::circle(Plane::xy(), 1.0)
}

/// The arc of the unit circle in the xy plane sweeping `sweep` from angle zero.
fn unit_arc(sweep: f64) -> TrimmedCurve {
    TrimmedCurve::arc(Plane::xy(), 1.0, sweep)
}

/// The point at `angle` on the unit circle in the xy plane.
fn at_angle(angle: f64) -> Point3 {
    Point3::new(angle.cos(), angle.sin(), 0.0)
}

fn assert_point_near(actual: Point3, expected: Point3) {
    assert!(
        actual.coincides(expected, LINEAR_TOLERANCE),
        "expected {expected:?}, got {actual:?}"
    );
}

#[test]
fn a_span_traverses_its_own_extent_from_fraction_zero_to_one() {
    let quarter = unit_arc(FRAC_PI_2);

    assert_point_near(quarter.start(), Point3::new(1.0, 0.0, 0.0));
    assert_point_near(quarter.end(), Point3::new(0.0, 1.0, 0.0));
    assert_point_near(quarter.point_at(0.5), at_angle(FRAC_PI_2 / 2.0));
    assert!((quarter.length() - FRAC_PI_2).abs() <= LINEAR_TOLERANCE);
}

#[test]
fn two_arcs_between_the_same_points_are_told_apart_only_by_their_spans() {
    // This is why a span is carried rather than derived: the minor and major
    // arcs share both endpoints, so the endpoints alone cannot name either.
    let minor = unit_arc(FRAC_PI_2);
    let major = unit_arc(FRAC_PI_2 - TAU);

    assert_point_near(minor.start(), major.start());
    assert_point_near(minor.end(), major.end());

    assert_point_near(minor.point_at(0.5), at_angle(FRAC_PI_2 / 2.0));
    // The major arc's midpoint is on the far side of the circle.
    assert!(major.point_at(0.5).x < 0.0, "got {:?}", major.point_at(0.5));
    assert!((minor.length() - FRAC_PI_2).abs() <= LINEAR_TOLERANCE);
    assert!((major.length() - (TAU - FRAC_PI_2)).abs() <= LINEAR_TOLERANCE);
}

#[test]
fn a_point_off_the_span_is_not_on_it_even_though_it_is_on_the_support() {
    let quarter = unit_arc(FRAC_PI_2);

    assert!(quarter.contains(Point3::new(1.0, 0.0, 0.0), LINEAR_TOLERANCE));
    assert!(quarter.contains(Point3::new(0.0, 1.0, 0.0), LINEAR_TOLERANCE));
    // On the circle, a half turn away from the arc.
    assert!(!quarter.contains(Point3::new(-1.0, 0.0, 0.0), LINEAR_TOLERANCE));
    assert!(!quarter.contains(Point3::new(0.0, -1.0, 0.0), LINEAR_TOLERANCE));
    // Not on the support at all.
    assert!(!quarter.contains(Point3::new(0.5, 0.5, 0.0), LINEAR_TOLERANCE));
}

#[test]
fn a_segment_does_not_carry_the_rest_of_its_line() {
    let segment = TrimmedCurve::between(
        Curve::line(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)),
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
    );

    assert!(segment.contains(Point3::new(1.0, 0.0, 0.0), LINEAR_TOLERANCE));
    assert!(!segment.contains(Point3::new(-2.0, 0.0, 0.0), LINEAR_TOLERANCE));
    assert!(!segment.contains(Point3::new(4.0, 0.0, 0.0), LINEAR_TOLERANCE));
}

#[test]
fn a_span_crossing_the_branch_cut_measures_against_its_own_extent() {
    // The support reports its parameter on `atan2`'s `(-pi, pi]`, so this span
    // runs off the end of that branch. A point just past it must read as being
    // near the end of this arc, not as having wrapped back to the start.
    let arc = TrimmedCurve::new(unit_circle(), Interval::new(PI - 0.2, PI + 0.2));
    let past_the_cut = arc.point_at(0.9);

    assert!(arc.contains(past_the_cut, LINEAR_TOLERANCE));
    let fraction = arc.parameter_at(past_the_cut);
    assert!(
        (fraction - 0.9).abs() <= 1e-9,
        "expected the arc's own fraction, got {fraction}"
    );
}

#[test]
fn reversing_a_span_keeps_its_geometry_and_swaps_its_ends() {
    let arc = unit_arc(FRAC_PI_2);
    let reversed = arc.reversed();

    assert_point_near(reversed.start(), arc.end());
    assert_point_near(reversed.end(), arc.start());
    assert_point_near(reversed.point_at(0.25), arc.point_at(0.75));
    assert!((reversed.length() - arc.length()).abs() <= LINEAR_TOLERANCE);
}

#[test]
fn narrowing_a_span_keeps_the_analytic_support() {
    let arc = unit_arc(FRAC_PI_2);
    let half = arc.sub(Interval::new(0.25, 0.75));

    assert!(
        matches!(half.curve(), Curve::Circle(_)),
        "narrowing must not degrade the support to NURBS"
    );
    assert_point_near(half.start(), arc.point_at(0.25));
    assert_point_near(half.end(), arc.point_at(0.75));
}

#[test]
fn the_cut_down_copy_is_the_span_over_a_unit_domain() {
    let arc = unit_arc(FRAC_PI_2);
    let standalone = arc.to_curve().expect("an arc converts exactly");

    assert_point_near(standalone.point_at(0.0), arc.start());
    assert_point_near(standalone.point_at(1.0), arc.end());
    for step in 0..=8 {
        let fraction = f64::from(step) / 8.0;
        let on_arc = arc.point_at(fraction);
        let projected = standalone.point_at(standalone.param_at(on_arc));
        assert!(
            (projected - on_arc).norm() <= 1e-9,
            "the exact copy must carry every point of the span"
        );
    }
}

#[test]
fn arc_helpers_anchor_the_support_and_span_the_sweep() {
    let arc = TrimmedCurve::arc(Plane::xy(), 2.0, FRAC_PI_2);

    assert_eq!(arc.interval(), Interval::new(0.0, FRAC_PI_2));
    assert_point_near(arc.start(), Point3::new(2.0, 0.0, 0.0));
    assert_point_near(arc.end(), Point3::new(0.0, 2.0, 0.0));
    assert!(matches!(arc.curve(), Curve::Circle(_)));

    // A negative sweep is the other way round the same support.
    let back = TrimmedCurve::arc(Plane::xy(), 2.0, -FRAC_PI_2);
    assert_point_near(back.end(), Point3::new(0.0, -2.0, 0.0));
    assert_eq!(back.curve(), arc.curve());

    let ellipse = TrimmedCurve::ellipse_arc(Frame::xyz(), 3.0, 1.0, PI);
    assert_eq!(ellipse.interval(), Interval::new(0.0, PI));
    assert_point_near(ellipse.start(), Point3::new(3.0, 0.0, 0.0));
    assert_point_near(ellipse.end(), Point3::new(-3.0, 0.0, 0.0));
    assert_point_near(ellipse.point_at(0.5), Point3::new(0.0, 1.0, 0.0));
}

#[test]
fn a_segment_spans_the_unit_window_of_the_line_through_its_ends() {
    let segment = TrimmedCurve::segment(Point3::new(1.0, 0.0, 0.0), Point3::new(4.0, 0.0, 0.0));

    assert_eq!(segment.interval(), Interval::new(0.0, 1.0));
    assert_point_near(segment.start(), Point3::new(1.0, 0.0, 0.0));
    assert_point_near(segment.end(), Point3::new(4.0, 0.0, 0.0));
    // The support runs on; only the span stops.
    assert!(matches!(segment.curve(), Curve::Line(_)));
    assert!(!segment.contains(Point3::new(9.0, 0.0, 0.0), LINEAR_TOLERANCE));
}
