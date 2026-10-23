use nalgebra::Vector2;
use ngk::geometry::dim2::{curves::Circle2, trimmed::TrimmedCurve2, utils::Point2};
use ngk::solver::{Constraint2d, SolveError, System2d, Target2d};

fn segment(a: (f64, f64), b: (f64, f64)) -> TrimmedCurve2 {
    TrimmedCurve2::segment(Point2::new(a.0, a.1), Point2::new(b.0, b.1))
}

#[test]
fn underconstrained_length_solves_and_reports_remaining_freedom() {
    let mut system = System2d::default();
    let line = system.add_segment(segment((0., 0.), (7., 2.))).unwrap();
    system
        .add_constraint(Constraint2d::length(line, 10.))
        .unwrap();
    let solution = system.solve().unwrap();
    let line = solution.segment(line).unwrap();
    assert!(((line.end() - line.start()).norm() - 10.).abs() < 1e-7);
    assert_eq!(solution.report().degrees_of_freedom, 3);
}

#[test]
fn compatible_redundancy_is_a_warning() {
    let mut system = System2d::default();
    let line = system.add_segment(segment((0., 0.), (7., 2.))).unwrap();
    system
        .add_constraint(Constraint2d::length(line, 10.))
        .unwrap();
    let duplicate = system
        .add_constraint(Constraint2d::length(line, 10.))
        .unwrap();
    let solution = system.solve().unwrap();
    assert_eq!(solution.report().degrees_of_freedom, 3);
    assert!(
        solution
            .report()
            .redundancies
            .iter()
            .any(|r| r.constraint == duplicate)
    );
}

#[test]
fn failure_keeps_the_previous_solution_and_edits_can_be_repaired() {
    let mut system = System2d::default();
    let line = system.add_segment(segment((0., 0.), (7., 2.))).unwrap();
    let length = system
        .add_constraint(Constraint2d::length(line, 10.))
        .unwrap();
    let old = system.solve().unwrap();
    let conflict = system
        .add_constraint(Constraint2d::length(line, 20.))
        .unwrap();
    assert!(
        matches!(system.solve(), Err(SolveError::Conflict { constraints }) if constraints.contains(&length) && constraints.contains(&conflict))
    );
    assert!(system.solution().is_none());
    assert_eq!(system.last_solution().unwrap().revision(), old.revision());
    system.remove_constraint(conflict).unwrap();
    assert!(system.solve().is_ok());
}

#[test]
fn dragging_a_point_preserves_its_permanent_circle_constraint() {
    let mut system = System2d::default();
    let circle = system
        .add_circle(Circle2::new(Point2::origin(), Vector2::x(), 10.))
        .unwrap();
    let p = system.add_point(Point2::new(10., 0.)).unwrap();
    system.add_constraint(Constraint2d::fixed(circle)).unwrap();
    system
        .add_constraint(Constraint2d::point_on_circle(p, circle))
        .unwrap();
    let solution = system
        .solve_toward(&[Target2d::point(p, Point2::new(0., 15.))])
        .unwrap();
    let point = solution.point(p).unwrap();
    assert!((point.coords.norm() - 10.).abs() < 1e-7);
    assert!((point - Point2::new(0., 10.)).norm() < 1e-4);
    assert_eq!(solution.report().degrees_of_freedom, 1);
    assert!(!solution.report().targets[0].reached);
}

#[test]
fn removing_coincidence_detaches_endpoint_identity() {
    let mut system = System2d::default();
    let a = system.add_segment(segment((0., 0.), (10., 0.))).unwrap();
    let b = system.add_segment(segment((20., 0.), (30., 5.))).unwrap();
    system.add_constraint(Constraint2d::fixed(a)).unwrap();
    let joint = system
        .add_constraint(Constraint2d::coincident(a.end(), b.start()))
        .unwrap();
    let joined = system.solve().unwrap();
    assert!((joined.point(a.end()).unwrap() - joined.point(b.start()).unwrap()).norm() < 1e-7);
    system.remove_constraint(joint).unwrap();
    let detached = system
        .solve_toward(&[Target2d::point(b.start(), Point2::new(20., 8.))])
        .unwrap();
    assert!((detached.point(a.end()).unwrap() - Point2::new(10., 0.)).norm() < 1e-7);
    assert!((detached.point(b.start()).unwrap() - Point2::new(20., 8.)).norm() < 1e-6);
}
