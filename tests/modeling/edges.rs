use ngk::builders::errors::EdgeCreationError;
use ngk::geometry::{Curve, Plane, Point3};
use ngk::modeling::edges;
use ngk::topology::closed::Closeable;

#[test]
fn line_returns_owned_line_edge_shape() {
    let start = Point3::new(0.0, 0.0, 0.0);
    let end = Point3::new(0.0, 3.0, 0.0);

    let shape = edges::line(start, end).expect("line should build");
    let edge = shape.edge();

    assert_eq!(edge.start().point(), Some(&start));
    assert_eq!(edge.end().point(), Some(&end));
    assert!(matches!(edge.curve(), Some(Curve::Line(_))));
}

#[test]
fn circle_returns_owned_closed_edge_shape() {
    let shape = edges::circle(Plane::xy(), 2.0).expect("circle should build");
    let edge = shape.edge();

    assert_eq!(shape.map().iter_edges().count(), 1);
    assert!(edge.is_closed());
    assert!(matches!(
        edge.curve(),
        Some(Curve::Circle(circle)) if (circle.radius() - 2.0).abs() <= f64::EPSILON
    ));
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
