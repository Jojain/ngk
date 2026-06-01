use ngk::geometry::{Curve2, Line2, Point2, Polyline2};

#[test]
fn line2_split_at_returns_two_lines_sharing_split_point() {
    let curve = Curve2::Line(Line2::new(Point2::new(0.0, 0.0), Point2::new(2.0, 0.0)));

    let (first, second) = curve.split_at(0.25);

    let Curve2::Line(first) = first else {
        panic!("line should split into a line");
    };
    let Curve2::Line(second) = second else {
        panic!("line should split into a line");
    };
    assert_eq!(first.start, Point2::new(0.0, 0.0));
    assert_eq!(first.end, Point2::new(0.5, 0.0));
    assert_eq!(second.start, Point2::new(0.5, 0.0));
    assert_eq!(second.end, Point2::new(2.0, 0.0));
}

#[test]
fn polyline2_split_at_inserts_split_point_in_both_halves() {
    let curve = Curve2::Polyline(Polyline2::new(vec![
        Point2::new(0.0, 0.0),
        Point2::new(1.0, 0.0),
        Point2::new(1.0, 1.0),
    ]));

    let (first, second) = curve.split_at(0.75);

    let Curve2::Polyline(first) = first else {
        panic!("polyline should split into a polyline");
    };
    let Curve2::Polyline(second) = second else {
        panic!("polyline should split into a polyline");
    };
    assert_eq!(
        first.points,
        vec![
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
            Point2::new(1.0, 0.5),
        ]
    );
    assert_eq!(
        second.points,
        vec![Point2::new(1.0, 0.5), Point2::new(1.0, 1.0)]
    );
}
