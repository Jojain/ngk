use ngk::geometry::Interval;

#[test]
fn interval_orders_bounds_without_changing_original() {
    let interval = Interval::new(4.0, 1.0);
    let ordered = interval.ordered();

    assert_eq!(interval.start, 4.0);
    assert_eq!(interval.end, 1.0);
    assert_eq!(ordered, Interval::new(1.0, 4.0));
}

#[test]
fn interval_length_is_absolute() {
    assert_eq!(Interval::new(1.0, 4.0).length(), 3.0);
    assert_eq!(Interval::new(4.0, 1.0).length(), 3.0);
}

#[test]
fn interval_contains_uses_tolerance() {
    let interval = Interval::new(1.0, 4.0);

    assert!(interval.contains(0.95, 0.1));
    assert!(interval.contains(4.05, 0.1));
    assert!(!interval.contains(4.2, 0.1));
}

#[test]
fn interval_detects_degenerate_lengths() {
    assert!(Interval::new(1.0, 1.01).is_degenerate(0.02));
    assert!(!Interval::new(1.0, 1.03).is_degenerate(0.02));
}

#[test]
fn interval_intersection_returns_overlap() {
    let a = Interval::new(0.0, 3.0);
    let b = Interval::new(2.0, 4.0);

    assert_eq!(a.intersection(b, 0.0), Some(Interval::new(2.0, 3.0)));
    assert!(a.intersects(b, 0.0));
    assert!(!a.intersects(Interval::new(4.0, 5.0), 0.0));
}

#[test]
fn interval_intersection_with_tolerance_returns_degenerate_gap_midpoint() {
    let a = Interval::new(0.0, 1.0);
    let b = Interval::new(1.05, 2.0);

    assert_eq!(a.intersection(b, 0.1), Some(Interval::new(1.025, 1.025)));
    assert_eq!(a.intersection(b, 0.01), None);
}
