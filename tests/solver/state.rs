use nalgebra::Vector2;
use ngk::geometry::dim2::{curves::Circle2, trimmed::TrimmedCurve2, utils::Point2};
use ngk::solver::{Constraint2d as C, EditError, SolveError, System2d, Target2d};

#[test]
fn fixing_an_arc_preserves_its_directed_shape_after_reseeding() {
    let mut s = System2d::default();
    let arc = s
        .add_arc(TrimmedCurve2::arc(Point2::origin(), Vector2::x(), 3., 1.))
        .unwrap();
    let end = s.point(arc.end()).unwrap();
    s.add_constraint(C::fixed(arc)).unwrap();
    s.update_arc(
        arc,
        TrimmedCurve2::arc(Point2::origin(), Vector2::x(), 3., -1.),
    )
    .unwrap();
    let solved = s.solve().unwrap();
    assert!((solved.point(arc.end()).unwrap() - end).norm() < 1e-8);
    assert_eq!(solved.report().degrees_of_freedom, 0);
}

#[test]
fn foreign_stale_and_aborted_batch_keys_are_rejected() {
    let mut a = System2d::default();
    let mut b = System2d::default();
    let p = a.add_point(Point2::origin()).unwrap();
    let q = b.add_point(Point2::origin()).unwrap();
    assert!(matches!(
        a.add_constraint(C::coincident(p, q)),
        Err(EditError::InvalidGeometryReference(_))
    ));
    a.remove_geometry(p).unwrap();
    let next = a.add_point(Point2::origin()).unwrap();
    assert!(a.point(p).is_err());
    assert!(a.point(next).is_ok());
    let mut leaked = None;
    let result: Result<(), EditError> = a.edit(|s| {
        leaked = Some(s.add_point(Point2::new(1., 2.))?);
        Err(EditError::InvalidConstraint("abort"))
    });
    assert!(result.is_err());
    let fresh = a.add_point(Point2::new(1., 2.)).unwrap();
    assert!(a.point(leaked.unwrap()).is_err());
    assert!(a.point(fresh).is_ok());
}

#[test]
fn batches_are_atomic_and_old_snapshots_own_deleted_geometry() {
    let mut s = System2d::default();
    let before = s.revision();
    let (a, b) = s
        .edit(|s| {
            let a = s.add_point(Point2::new(1., 2.))?;
            let b = s.add_point(Point2::new(3., 4.))?;
            s.add_constraint(C::distance(a, b, 10.))?;
            Ok((a, b))
        })
        .unwrap();
    assert_eq!(s.revision(), before + 1);
    let old = s.solve().unwrap();
    assert!(matches!(
        s.remove_geometry(a),
        Err(EditError::ReferencedGeometry { .. })
    ));
    let removed = s.remove_cascade(a).unwrap();
    assert_eq!(removed.constraints.len(), 1);
    assert!(s.point(a).is_err());
    assert!(old.point(a).is_ok());
    assert!(old.point(b).is_ok());
    assert!(s.solution().is_none());
    assert!(s.last_solution().is_some());
}

#[test]
fn edits_after_failure_are_not_overwritten_by_historical_seed_values() {
    let mut s = System2d::default();
    let a = s.add_point(Point2::origin()).unwrap();
    let b = s.add_point(Point2::new(10., 0.)).unwrap();
    let dimension = s.add_constraint(C::distance(a, b, 10.)).unwrap();
    s.solve().unwrap();
    let conflict = s.add_constraint(C::distance(a, b, 20.)).unwrap();
    assert!(s.solve().is_err());
    let c = s.add_point(Point2::new(100., 200.)).unwrap();
    s.update_point(a, Point2::new(40., 50.)).unwrap();
    s.remove_constraint(conflict).unwrap();
    s.remove_constraint(dimension).unwrap();
    let solved = s.solve().unwrap();
    assert_eq!(solved.point(a).unwrap(), Point2::new(40., 50.));
    assert_eq!(solved.point(c).unwrap(), Point2::new(100., 200.));
}

#[test]
fn fixed_values_are_captured_and_refresh_only_on_explicit_constraint_update() {
    let mut s = System2d::default();
    let p = s.add_point(Point2::new(1., 2.)).unwrap();
    let fix = s.add_constraint(C::fixed_point(p)).unwrap();
    s.solve().unwrap();
    s.update_point(p, Point2::new(9., 10.)).unwrap();
    let solved = s.solve().unwrap();
    assert!((solved.point(p).unwrap() - Point2::new(1., 2.)).norm() < 1e-7);
    s.update_point(p, Point2::new(9., 10.)).unwrap();
    s.update_constraint(fix, C::fixed_point(p)).unwrap();
    let solved = s.solve().unwrap();
    assert_eq!(solved.point(p).unwrap(), Point2::new(9., 10.));
}

#[test]
fn invalid_edits_and_targets_do_not_change_state() {
    let mut s = System2d::default();
    let p = s.add_point(Point2::new(1., 2.)).unwrap();
    let old = s.solve().unwrap();
    let revision = s.revision();
    assert!(s.update_point(p, Point2::new(f64::NAN, 0.)).is_err());
    assert!(
        s.add_circle(Circle2::new(Point2::origin(), Vector2::x(), 0.))
            .is_err()
    );
    assert!(
        s.add_arc(TrimmedCurve2::arc(
            Point2::origin(),
            Vector2::x(),
            1.,
            std::f64::consts::TAU
        ))
        .is_err()
    );
    assert_eq!(s.revision(), revision);
    assert!(matches!(
        s.solve_toward(&[Target2d::point(p, Point2::new(4., 5.)).with_weight(-1.)]),
        Err(SolveError::InvalidInput(_))
    ));
    assert_eq!(s.last_solution().unwrap().sequence(), old.sequence());
}

#[test]
fn failure_of_one_component_does_not_publish_another_components_candidate() {
    let mut s = System2d::default();
    let p = s.add_point(Point2::new(1., 2.)).unwrap();
    let a = s.add_point(Point2::origin()).unwrap();
    let b = s.add_point(Point2::new(1., 0.)).unwrap();
    s.add_constraint(C::distance(a, b, 3.)).unwrap();
    s.add_constraint(C::distance(a, b, 4.)).unwrap();
    assert!(
        s.solve_toward(&[Target2d::point(p, Point2::new(9., 10.))])
            .is_err()
    );
    assert_eq!(s.point(p).unwrap(), Point2::new(1., 2.));
    assert!(s.last_solution().is_none());
}

#[test]
fn a_fixed_system_keeps_constraints_when_a_target_is_unreachable() {
    let mut s = System2d::default();
    let p = s.add_point(Point2::new(1., 2.)).unwrap();
    s.add_constraint(C::fixed(p)).unwrap();
    let revision = s.revision();
    let first = s.solve().unwrap();
    let next = s
        .solve_toward(&[Target2d::point(p, Point2::new(50., 100.))])
        .unwrap();
    assert_eq!(next.point(p).unwrap(), first.point(p).unwrap());
    assert_eq!(s.revision(), revision);
    assert!(next.sequence() > first.sequence());
    assert_eq!(next.report().degrees_of_freedom, 0);
    assert!(!next.report().targets[0].reached);
}
