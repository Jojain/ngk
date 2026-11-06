use nalgebra::Vector2;
use ngk::geometry::dim2::{curves::Circle2, trimmed::Segment2, utils::Point2};
use ngk::solver::{
    CircleId, Constraint2d, ConstraintId, GeometryId, PointId, SegmentId, SolveError, System2d,
    Target2d,
};

fn segment(a: (f64, f64), b: (f64, f64)) -> Segment2 {
    Segment2::new(Point2::new(a.0, a.1), Point2::new(b.0, b.1))
}

#[test]
fn underconstrained_length_solves_and_reports_remaining_freedom() {
    let mut system = System2d::default();
    let line = system
        .add_segment(SegmentId::new(1), segment((0., 0.), (7., 2.)))
        .unwrap();
    system
        .add_constraint(ConstraintId::new(2), Constraint2d::length(line, 10.))
        .unwrap();
    let solution = system.solve().unwrap();
    let line = solution.segment(line).unwrap();
    assert!(((line.end() - line.start()).norm() - 10.).abs() < 1e-7);
    assert_eq!(solution.report().degrees_of_freedom, 3);
}

#[test]
fn compatible_redundancy_is_a_warning() {
    let mut system = System2d::default();
    let line = system
        .add_segment(SegmentId::new(3), segment((0., 0.), (7., 2.)))
        .unwrap();
    system
        .add_constraint(ConstraintId::new(4), Constraint2d::length(line, 10.))
        .unwrap();
    let duplicate = system
        .add_constraint(ConstraintId::new(5), Constraint2d::length(line, 10.))
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
    let line = system
        .add_segment(SegmentId::new(6), segment((0., 0.), (7., 2.)))
        .unwrap();
    let length = system
        .add_constraint(ConstraintId::new(7), Constraint2d::length(line, 10.))
        .unwrap();
    let old = system.solve().unwrap();
    let conflict = system
        .add_constraint(ConstraintId::new(8), Constraint2d::length(line, 20.))
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
        .add_circle(
            CircleId::new(9),
            Circle2::new(Point2::origin(), Vector2::x(), 10.),
        )
        .unwrap();
    let p = system
        .add_point(PointId::new(10), Point2::new(10., 0.))
        .unwrap();
    system
        .add_constraint(ConstraintId::new(11), Constraint2d::fixed(circle))
        .unwrap();
    system
        .add_constraint(
            ConstraintId::new(12),
            Constraint2d::point_on_circle(p, circle),
        )
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
fn freedom_is_reported_per_primitive() {
    let mut system = System2d::default();
    let free = system
        .add_point(PointId::new(13), Point2::new(1., 1.))
        .unwrap();
    let swing = system
        .add_segment(SegmentId::new(14), segment((0., 0.), (10., 0.)))
        .unwrap();
    let locked = system
        .add_segment(SegmentId::new(15), segment((20., 0.), (30., 0.)))
        .unwrap();
    let circle = system
        .add_circle(
            CircleId::new(16),
            Circle2::new(Point2::new(50., 0.), Vector2::x(), 5.),
        )
        .unwrap();
    system
        .add_constraint(
            ConstraintId::new(17),
            Constraint2d::fixed_point(swing.start()),
        )
        .unwrap();
    system
        .add_constraint(ConstraintId::new(18), Constraint2d::length(swing, 10.))
        .unwrap();
    system
        .add_constraint(ConstraintId::new(19), Constraint2d::fixed(locked))
        .unwrap();
    system
        .add_constraint(ConstraintId::new(20), Constraint2d::radius(circle, 5.))
        .unwrap();
    let solution = system.solve().unwrap();
    let report = solution.report();
    let freedom = |geometry: GeometryId| {
        report
            .freedoms
            .iter()
            .find(|freedom| freedom.geometry == geometry)
            .unwrap()
            .degrees
    };
    assert_eq!(freedom(free.into()), 2);
    assert_eq!(freedom(swing.into()), 1);
    assert_eq!(freedom(locked.into()), 0);
    assert_eq!(freedom(circle.into()), 2);
    assert_eq!(report.degrees_of_freedom, 5);
}

#[test]
fn a_motion_shared_by_primitives_counts_in_each() {
    let mut system = System2d::default();
    let a = system
        .add_segment(SegmentId::new(21), segment((0., 0.), (10., 0.)))
        .unwrap();
    let b = system
        .add_segment(SegmentId::new(22), segment((10., 0.), (10., 10.)))
        .unwrap();
    system
        .add_constraint(
            ConstraintId::new(23),
            Constraint2d::coincident(a.end(), b.start()),
        )
        .unwrap();
    system
        .add_constraint(ConstraintId::new(24), Constraint2d::fixed_point(a.start()))
        .unwrap();
    system
        .add_constraint(ConstraintId::new(25), Constraint2d::fixed_point(b.end()))
        .unwrap();
    let solution = system.solve().unwrap();
    let report = solution.report();
    assert_eq!(report.degrees_of_freedom, 2);
    assert!(report.freedoms.iter().all(|freedom| freedom.degrees == 2));
}

#[test]
fn removing_coincidence_detaches_endpoint_identity() {
    let mut system = System2d::default();
    let a = system
        .add_segment(SegmentId::new(26), segment((0., 0.), (10., 0.)))
        .unwrap();
    let b = system
        .add_segment(SegmentId::new(27), segment((20., 0.), (30., 5.)))
        .unwrap();
    system
        .add_constraint(ConstraintId::new(28), Constraint2d::fixed(a))
        .unwrap();
    let joint = system
        .add_constraint(
            ConstraintId::new(29),
            Constraint2d::coincident(a.end(), b.start()),
        )
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
