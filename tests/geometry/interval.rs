use ngk::geometry::{Fraction, Interval, Native, NativeParam};

/// The parameter space is never inferred from bare numbers, so a span built
/// outside a typed position names the space it lives in.
fn native(start: f64, end: f64) -> Interval<Native> {
    Interval::new(start, end)
}

#[test]
fn reversed_interval_preserves_signed_traversal() {
    let interval = native(0.0, std::f64::consts::FRAC_PI_2);
    let reversed = interval.reversed();

    assert_eq!(reversed, native(std::f64::consts::FRAC_PI_2, 0.0));
    assert_eq!(reversed.delta(), -std::f64::consts::FRAC_PI_2);
    assert_eq!(
        reversed.at(Fraction::new(0.5)),
        NativeParam::new(std::f64::consts::FRAC_PI_4)
    );
}

#[test]
fn interval_orders_bounds_without_changing_original() {
    let interval = native(4.0, 1.0);
    let ordered = interval.ordered();

    assert_eq!(interval.start, NativeParam::new(4.0));
    assert_eq!(interval.end, NativeParam::new(1.0));
    assert_eq!(ordered, native(1.0, 4.0));
}

#[test]
fn interval_length_is_absolute() {
    assert_eq!(native(1.0, 4.0).length(), 3.0);
    assert_eq!(native(4.0, 1.0).length(), 3.0);
}

#[test]
fn interval_contains_uses_tolerance() {
    let interval = native(1.0, 4.0);

    assert!(interval.contains(NativeParam::new(0.95), 0.1));
    assert!(interval.contains(NativeParam::new(4.05), 0.1));
    assert!(!interval.contains(NativeParam::new(4.2), 0.1));
}

#[test]
fn interval_detects_degenerate_lengths() {
    assert!(native(1.0, 1.01).is_degenerate(0.02));
    assert!(!native(1.0, 1.03).is_degenerate(0.02));
}

#[test]
fn interval_intersection_returns_overlap() {
    let a = native(0.0, 3.0);
    let b = native(2.0, 4.0);

    assert_eq!(a.intersection(b, 0.0), Some(native(2.0, 3.0)));
    assert!(a.intersects(b, 0.0));
    assert!(!a.intersects(native(4.0, 5.0), 0.0));
}

#[test]
fn interval_intersection_with_tolerance_returns_degenerate_gap_midpoint() {
    let a = native(0.0, 1.0);
    let b = native(1.05, 2.0);

    assert_eq!(a.intersection(b, 0.1), Some(native(1.025, 1.025)));
    assert_eq!(a.intersection(b, 0.01), None);
}

#[test]
fn or_extent_substitutes_only_infinite_endpoints() {
    let bounded = native(0.0, std::f64::consts::TAU);
    assert_eq!(
        bounded.or_extent(1.0),
        bounded,
        "a finite domain keeps its real extent even when wider than the window"
    );
    assert_eq!(
        Interval::<Native>::unbounded().or_extent(3.0),
        native(-3.0, 3.0)
    );
    assert_eq!(native(2.0, f64::INFINITY).or_extent(5.0), native(2.0, 5.0));
}

#[test]
fn is_finite_distinguishes_bounded_domains() {
    assert!(native(0.0, 1.0).is_finite());
    assert!(!Interval::<Native>::unbounded().is_finite());
    assert!(!native(0.0, f64::INFINITY).is_finite());
}

/// `at` and `fraction_of` are the only two ways across the two parameter
/// spaces, and each is the other's inverse on the span it was asked of.
#[test]
fn at_and_fraction_of_invert_each_other_on_the_span_they_are_asked_of() {
    let span = native(2.0, 6.0);

    assert_eq!(span.at(Fraction::START), span.start);
    assert_eq!(span.at(Fraction::END), span.end);
    assert_eq!(span.at(Fraction::new(0.25)), NativeParam::new(3.0));
    assert_eq!(span.fraction_of(NativeParam::new(3.0)), Fraction::new(0.25));

    for value in [0.0, 0.3, 0.5, 1.0, 1.4, -0.2] {
        let fraction = Fraction::new(value);
        let round_trip = span.fraction_of(span.at(fraction));
        assert!(
            (round_trip - fraction).abs() <= 1.0e-15,
            "{fraction} came back as {round_trip}",
        );
    }
}

/// A fraction is a fraction *of a span*, so the same number names different
/// parameters on different spans. This is the mistake the brand cannot catch
/// on its own, and the reason a fraction is only ever reachable through the
/// span it measures.
#[test]
fn the_same_fraction_names_different_parameters_on_different_spans() {
    let half = Fraction::new(0.5);

    assert_eq!(native(0.0, 1.0).at(half), NativeParam::new(0.5));
    assert_eq!(native(1.0, 2.0).at(half), NativeParam::new(1.5));
}

/// A fraction outside `[0, 1]` names a point beyond the span's ends. That is
/// information — a solver hit just off an edge — so it is carried rather than
/// refused, and a caller that needs a window asks for one.
#[test]
fn a_fraction_may_fall_outside_the_span_it_measures() {
    let past_the_end = Fraction::new(1.4);

    assert!(!past_the_end.is_inside_unit());
    assert!(Fraction::new(0.4).is_inside_unit());
    assert_eq!(past_the_end.clamped_to_unit(), Fraction::END);
    assert_eq!(native(0.0, 10.0).at(past_the_end), NativeParam::new(14.0));
}

/// A degenerate span has no direction to measure along.
#[test]
fn a_degenerate_span_reports_every_parameter_at_its_start() {
    let point = native(3.0, 3.0);

    assert_eq!(point.fraction_of(NativeParam::new(3.0)), Fraction::START);
    assert_eq!(point.fraction_of(NativeParam::new(9.0)), Fraction::START);
}

#[test]
fn midpoint_is_the_parameter_halfway_along() {
    assert_eq!(native(2.0, 6.0).midpoint(), NativeParam::new(4.0));
    assert_eq!(native(6.0, 2.0).midpoint(), NativeParam::new(4.0));
}
