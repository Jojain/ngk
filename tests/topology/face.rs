use ngk::geometry::{LINEAR_TOLERANCE, Plane, Point3, PointCoincidence};
use ngk::modeling::faces;

#[test]
fn face_point_at_evaluates_its_support_surface() {
    let shape = faces::rectangle(Plane::xy(), 2.0, 3.0).expect("face should build");
    let point = shape.face().point_at(0.5, 1.25);

    assert!(point.coincides(&Point3::new(0.5, 1.25, 0.0), LINEAR_TOLERANCE));
}

#[test]
fn face_point_at_is_defined_inside_a_trimmed_hole() {
    let shape = faces::annulus(Plane::xy(), 2.0, 1.0).expect("face should build");
    let point = shape.face().point_at(0.0, 0.0);

    assert!(point.coincides(&Point3::origin(), LINEAR_TOLERANCE));
}
