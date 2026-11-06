use nalgebra::Vector2;
use ngk::geometry::dim2::{
    curves::Circle2,
    trimmed::{Arc2, Segment2},
    utils::Point2,
};
use ngk::solver::{
    ArcId, CircleId, Constraint2d as C, ConstraintId, EditError, PointId, SegmentId, SolveError,
    System2d, Target2d,
};
use radians::Rad64;
use std::f64::consts::FRAC_PI_2;

#[test]
fn fixing_an_arc_preserves_its_directed_shape_after_reseeding() {
    let mut s = System2d::default();
    let arc = s
        .add_arc(Arc2::new(Point2::origin(), Vector2::x(), 3., 1.))
        .unwrap();
    let end = s.point(arc.end()).unwrap();
    s.add_constraint(C::fixed(arc)).unwrap();
    s.update_arc(arc, Arc2::new(Point2::origin(), Vector2::x(), 3., -1.))
        .unwrap();
    let solved = s.solve().unwrap();
    assert!((solved.point(arc.end()).unwrap() - end).norm() < 1e-8);
    assert_eq!(solved.report().degrees_of_freedom, 0);
}

#[test]
fn removed_and_aborted_ids_are_not_in_the_system() {
    let mut a = System2d::default();
    let p = a.add_point(Point2::origin()).unwrap();
    let absent = PointId::new(99);
    assert!(matches!(
        a.add_constraint(C::coincident(p, absent)),
        Err(EditError::InvalidGeometryReference(_))
    ));
    a.remove_geometry(p).unwrap();
    assert!(a.point(p).is_err());
    let aborted = PointId::new(2);
    let result: Result<(), EditError> = a.edit(|s| {
        s.insert_point(aborted, Point2::new(1., 2.))?;
        Err(EditError::InvalidConstraint("abort"))
    });
    assert!(result.is_err());
    assert!(a.point(aborted).is_err());
}

#[test]
fn an_id_is_refused_while_in_use_and_can_be_inserted_back_once_free() {
    let mut s = System2d::default();
    let p = s.add_point(Point2::origin()).unwrap();
    assert!(matches!(
        s.insert_point(p, Point2::new(5., 5.)),
        Err(EditError::DuplicateGeometry(_))
    ));
    let c = s.add_constraint(C::fixed_point(p)).unwrap();
    assert!(matches!(
        s.insert_constraint(c, C::fixed_point(p)),
        Err(EditError::DuplicateConstraint(_))
    ));
    s.remove_cascade(p).unwrap();
    s.insert_point(p, Point2::new(5., 5.)).unwrap();
    assert_eq!(s.point(p).unwrap(), Point2::new(5., 5.));
}

#[test]
fn ids_of_different_kinds_do_not_collide() {
    let mut s = System2d::default();
    s.insert_point(PointId::new(1), Point2::origin()).unwrap();
    s.insert_circle(
        CircleId::new(1),
        Circle2::new(Point2::origin(), Vector2::x(), 1.),
    )
    .unwrap();
    assert_eq!(s.geometry_count(), 2);
}

#[test]
fn an_issued_id_is_never_issued_again() {
    let mut s = System2d::default();
    let first = s.add_point(Point2::origin()).unwrap();
    s.remove_geometry(first).unwrap();
    let second = s.add_point(Point2::origin()).unwrap();
    assert_ne!(first, second);
    assert!(s.point(first).is_err());

    let mut aborted = None;
    let result: Result<(), EditError> = s.edit(|s| {
        aborted = Some(s.add_point(Point2::origin())?);
        Err(EditError::InvalidConstraint("abort"))
    });
    assert!(result.is_err());
    let aborted = aborted.unwrap();
    assert!(s.point(aborted).is_err());
    assert_ne!(s.add_point(Point2::origin()).unwrap(), aborted);
}

#[test]
fn an_inserted_id_is_not_issued_later() {
    let mut s = System2d::default();
    s.insert_point(PointId::new(40), Point2::origin()).unwrap();
    assert!(s.add_point(Point2::origin()).unwrap().value() > 40);
    s.insert_constraint(ConstraintId::new(70), C::fixed_point(PointId::new(40)))
        .unwrap();
    assert!(s.add_point(Point2::origin()).unwrap().value() > 70);
}

#[test]
fn a_stored_system_keeps_issuing_new_ids() {
    let mut s = System2d::default();
    let removed = s.add_point(Point2::origin()).unwrap();
    s.remove_geometry(removed).unwrap();
    let mut restored: System2d = serde_json::from_str(&serde_json::to_string(&s).unwrap()).unwrap();
    assert_ne!(restored.add_point(Point2::origin()).unwrap(), removed);
}

#[test]
fn the_same_problem_added_in_another_order_solves_the_same() {
    let (a, b) = (PointId::new(1), PointId::new(2));
    let mut forward = System2d::default();
    forward.insert_point(a, Point2::origin()).unwrap();
    forward.insert_point(b, Point2::new(3., 4.)).unwrap();
    forward.add_constraint(C::distance(a, b, 10.)).unwrap();
    let mut backward = System2d::default();
    backward.insert_point(b, Point2::new(3., 4.)).unwrap();
    backward.insert_point(a, Point2::origin()).unwrap();
    backward.add_constraint(C::distance(a, b, 10.)).unwrap();
    let (forward, backward) = (forward.solve().unwrap(), backward.solve().unwrap());
    assert_eq!(forward.point(b).unwrap(), backward.point(b).unwrap());
}

#[test]
fn a_stored_system_solves_where_it_was_and_keeps_what_it_captured() {
    let (bottom, right) = (SegmentId::new(1), SegmentId::new(2));
    let arc = ArcId::new(1);
    let mut s = System2d::default();
    s.insert_segment(bottom, Segment2::new(Point2::origin(), Point2::new(9., 1.)))
        .unwrap();
    s.insert_segment(
        right,
        Segment2::new(Point2::new(9., 0.), Point2::new(9., 5.)),
    )
    .unwrap();
    s.insert_arc(arc, Arc2::new(Point2::new(20., 0.), Vector2::x(), 3., -2.))
        .unwrap();
    s.add_constraint(C::coincident(bottom.end(), right.start()))
        .unwrap();
    s.add_constraint(C::fixed_point(bottom.start())).unwrap();
    let width = s.add_constraint(C::length(bottom, 10.)).unwrap();
    s.add_constraint(C::angle(bottom, right, Rad64::new(FRAC_PI_2)))
        .unwrap();
    s.add_constraint(C::fixed(arc)).unwrap();
    let solved = s.solve().unwrap();

    let json = serde_json::to_string(&s).unwrap();
    let mut restored: System2d = serde_json::from_str(&json).unwrap();
    assert_eq!(restored.geometry_count(), 3);
    assert_eq!(restored.constraint_count(), 5);
    assert!(restored.last_solution().is_none());
    assert!(matches!(
        restored.constraint(width),
        Ok(C::Length(line, length)) if *line == bottom && *length == 10.
    ));

    let again = restored.solve().unwrap();
    assert_eq!(again.report().iterations, 0);
    let (before, after) = (
        solved.segment(right).unwrap(),
        again.segment(right).unwrap(),
    );
    assert!((before.end() - after.end()).norm() < 1e-12);
    let (before, after) = (solved.arc(arc).unwrap(), again.arc(arc).unwrap());
    assert!((before.end() - after.end()).norm() < 1e-12);
    assert_eq!(
        again.report().degrees_of_freedom,
        solved.report().degrees_of_freedom
    );
}

#[test]
fn a_stored_fixed_value_survives_an_edited_seed() {
    let p = PointId::new(1);
    let mut s = System2d::default();
    s.insert_point(p, Point2::new(1., 2.)).unwrap();
    s.add_constraint(C::fixed_point(p)).unwrap();
    s.update_point(p, Point2::new(9., 10.)).unwrap();
    let mut restored: System2d = serde_json::from_str(&serde_json::to_string(&s).unwrap()).unwrap();
    let solved = restored.solve().unwrap();
    assert!((solved.point(p).unwrap() - Point2::new(1., 2.)).norm() < 1e-9);
}

#[test]
fn a_stored_system_is_refused_when_an_edit_could_not_have_produced_it() {
    let mut s = System2d::default();
    let line = s
        .add_segment(Segment2::new(Point2::origin(), Point2::new(1., 0.)))
        .unwrap();
    s.add_constraint(C::length(line, 1.)).unwrap();
    let json = serde_json::to_value(&s).unwrap();
    let refused = |mutate: &dyn Fn(&mut serde_json::Value)| {
        let mut forged = json.clone();
        mutate(&mut forged);
        serde_json::from_value::<System2d>(forged).is_err()
    };
    assert!(serde_json::from_value::<System2d>(json.clone()).is_ok());
    assert!(refused(&|v| {
        v["primitives"][0][1]["values"] = serde_json::json!([0., 0., 1.]);
    }));
    assert!(refused(&|v| {
        v["primitives"][0][1]["values"] = serde_json::json!([1., 1., 1., 1.]);
    }));
    assert!(refused(&|v| {
        v["primitives"][0][0]["kind"] = serde_json::json!("Arc");
    }));
    assert!(refused(&|v| {
        v["next_id"] = serde_json::json!(1);
    }));
    assert!(refused(&|v| {
        let duplicate = v["primitives"][0].clone();
        v["primitives"].as_array_mut().unwrap().push(duplicate);
    }));
    assert!(refused(&|v| {
        v["constraints"][0][1]["definition"]["Length"][0] = serde_json::json!(99);
    }));
    assert!(refused(&|v| {
        v["constraints"][0][1]["definition"]["Length"][1] = serde_json::json!(-1.);
    }));
    assert!(refused(&|v| {
        v["constraints"][0][1]["definition"] = serde_json::json!({
            "FixedPoint": { "geometry": { "kind": "Segment", "value": 1 }, "role": "Center" }
        });
    }));
    assert!(refused(&|v| {
        v["constraints"][0][1]["definition"] = serde_json::json!({
            "Radius": [{ "kind": "Segment", "value": 1 }, 1.]
        });
    }));
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
        s.add_arc(Arc2::new(
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
