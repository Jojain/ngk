use ngk::builders::errors::EdgeCreationError;
use ngk::geometry::{Curve, Interval, Plane, Point3};
use ngk::modeling::edges;
use ngk::topology::closed::Closeable;

#[test]
fn line_returns_owned_line_edge_shape() {
    let start = Point3::new(0.0, 0.0, 0.0);
    let end = Point3::new(0.0, 3.0, 0.0);

    let shape = edges::line(start, end).expect("line should build");
    let edge = shape.edge();

    assert_eq!(*edge.bounded_unchecked().start().point(), start);
    assert_eq!(*edge.bounded_unchecked().end().point(), end);
    assert!(matches!(edge.curve(), Curve::Line(_)));
}

#[test]
fn circle_returns_owned_closed_edge_shape() {
    let shape = edges::circle(Plane::xy(), 2.0).expect("circle should build");
    let edge = shape.edge();

    assert_eq!(shape.model().iter_edges().count(), 1);
    assert!(edge.is_closed());
    assert!(matches!(
        edge.curve(),
        Curve::Circle(circle) if (circle.radius() - 2.0).abs() <= f64::EPSILON
    ));
}

#[test]
fn arc_returns_owned_open_circle_edge_shape() {
    let shape =
        edges::arc(Plane::xy(), 2.0, 0.0, std::f64::consts::FRAC_PI_2).expect("arc should build");
    let edge = shape.edge();

    assert_eq!(shape.model().iter_edges().count(), 1);
    assert!(!edge.is_closed());
    assert!(matches!(
        edge.curve(),
        Curve::Circle(circle) if (circle.radius() - 2.0).abs() <= f64::EPSILON
    ));
}

#[test]
fn reversed_arc_uses_the_same_span_with_a_negative_parameter_delta() {
    let shape =
        edges::arc(Plane::xy(), 2.0, 0.0, std::f64::consts::FRAC_PI_2).expect("arc should build");
    let edge = shape.edge();

    assert_eq!(
        edge.parameter_interval(),
        Interval::new(0.0, std::f64::consts::FRAC_PI_2)
    );
    assert_eq!(
        edge.reversed().parameter_interval(),
        Interval::new(std::f64::consts::FRAC_PI_2, 0.0)
    );
}

#[test]
fn circle_edge_vertices_recover_a_span_wider_than_half_a_turn() {
    let span = 3.0 * std::f64::consts::FRAC_PI_2;
    let shape = edges::arc(Plane::xy(), 1.0, 0.0, span).expect("arc should build");

    assert_eq!(shape.edge().parameter_interval(), Interval::new(0.0, span));
}

#[test]
fn circle_rejects_invalid_radius() {
    let zero_radius = edges::circle(Plane::xy(), 0.0);
    assert!(matches!(
        zero_radius,
        Err(EdgeCreationError::InvalidRadius { radius: 0.0 })
    ));
    assert!(matches!(
        edges::circle(Plane::xy(), f64::NAN),
        Err(EdgeCreationError::InvalidRadius { radius }) if radius.is_nan()
    ));
}
