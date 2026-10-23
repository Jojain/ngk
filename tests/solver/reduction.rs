use ngk::geometry::dim2::{trimmed::TrimmedCurve2, utils::Point2};
use ngk::solver::{Constraint2d as C, SolveOptions, System2d};

#[test]
fn horizontal_alignment_can_rotate_a_vertical_segment_without_a_length_dimension() {
    let mut s = System2d::default();
    let line = s
        .add_segment(TrimmedCurve2::segment(
            Point2::origin(),
            Point2::new(0., 10.),
        ))
        .unwrap();
    s.add_constraint(C::horizontal(line)).unwrap();
    let solved = s.solve().unwrap();
    let line = solved.segment(line).unwrap();
    assert!((line.start().y - line.end().y).abs() < 1e-8);
    assert!((line.start() - line.end()).norm() > 1e-6);
    assert_eq!(solved.report().degrees_of_freedom, 3);
    assert!(solved.report().redundancies.is_empty());
}

#[test]
fn coincidence_preserves_the_unreduced_minimum_motion_metric() {
    let mut results = vec![];
    for linear_reduction in [false, true] {
        let mut s = System2d::new(SolveOptions {
            linear_reduction,
            ..SolveOptions::default()
        });
        let points: Vec<_> = [0., 6., 12.]
            .into_iter()
            .map(|x| s.add_point(Point2::new(x, 3.)).unwrap())
            .collect();
        s.add_constraint(C::coincident(points[0], points[1]))
            .unwrap();
        s.add_constraint(C::coincident(points[1], points[2]))
            .unwrap();
        let solved = s.solve().unwrap();
        assert_eq!(solved.report().degrees_of_freedom, 2);
        assert!(solved.report().redundancies.is_empty());
        results.push(solved.point(points[0]).unwrap());
    }
    assert!((results[0] - results[1]).norm() < 1e-7);
    assert!((results[1] - Point2::new(6., 3.)).norm() < 1e-7);
}

#[test]
fn removing_an_alignment_restores_rotation_and_original_endpoint_identity() {
    let mut s = System2d::default();
    let line = s
        .add_segment(TrimmedCurve2::segment(
            Point2::origin(),
            Point2::new(10., 0.),
        ))
        .unwrap();
    s.add_constraint(C::length(line, 10.)).unwrap();
    let alignment = s.add_constraint(C::horizontal(line)).unwrap();
    let solved = s.solve().unwrap();
    assert_eq!(solved.report().degrees_of_freedom, 2);
    assert!(solved.report().redundancies.is_empty());
    s.remove_constraint(alignment).unwrap();
    assert_eq!(s.solve().unwrap().report().degrees_of_freedom, 3);
}

#[test]
fn inconsistent_affine_cycles_are_validated_in_the_original_problem() {
    let mut s = System2d::default();
    let a = s.add_point(Point2::origin()).unwrap();
    let b = s.add_point(Point2::new(3., 0.)).unwrap();
    let c = s.add_point(Point2::new(6., 0.)).unwrap();
    s.add_constraint(C::OffsetX(a.into(), b.into(), 3.))
        .unwrap();
    s.add_constraint(C::OffsetX(b.into(), c.into(), 3.))
        .unwrap();
    let bad = s
        .add_constraint(C::OffsetX(a.into(), c.into(), 7.))
        .unwrap();
    assert!(s.solve().is_err());
    assert!(s.last_solution().is_none());
    s.remove_constraint(bad).unwrap();
    assert_eq!(s.solve().unwrap().report().degrees_of_freedom, 4);
}
