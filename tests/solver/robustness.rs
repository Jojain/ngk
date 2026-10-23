use ngk::geometry::dim2::{trimmed::TrimmedCurve2, utils::Point2};
use ngk::solver::{Constraint2d as C, SolveError, SolveOptions, System2d, Target2d};
use std::f64::consts::PI;

#[test]
fn published_world_geometry_satisfies_tolerance_even_at_large_offsets() {
    let mut s = System2d::default();
    let a = s.add_point(Point2::new(1e12, 1e12)).unwrap();
    let b = s.add_point(Point2::new(1e12 + 3., 1e12 + 3.)).unwrap();
    s.add_constraint(C::fixed(a)).unwrap();
    s.add_constraint(C::distance(a, b, PI)).unwrap();
    match s.solve() {
        Ok(solution) => assert!(
            ((solution.point(a).unwrap() - solution.point(b).unwrap()).norm() - PI).abs()
                <= s.options().linear_tolerance
        ),
        Err(SolveError::NonConvergence { .. }) => assert!(s.last_solution().is_none()),
        Err(error) => panic!("unexpected failure: {error:?}"),
    }
}

#[test]
fn explicitly_reseeding_an_unsigned_angle_can_select_the_other_side() {
    let mut s = System2d::default();
    let a = s
        .add_segment(TrimmedCurve2::segment(
            Point2::origin(),
            Point2::new(10., 0.),
        ))
        .unwrap();
    let b = s
        .add_segment(TrimmedCurve2::segment(
            Point2::origin(),
            Point2::new(3_f64.sqrt(), 1.),
        ))
        .unwrap();
    s.add_constraint(C::fixed(a)).unwrap();
    s.add_constraint(C::angle(a, b, PI / 6.)).unwrap();
    s.add_constraint(C::length(b, 2.)).unwrap();
    s.solve().unwrap();
    let negative = TrimmedCurve2::segment(Point2::origin(), Point2::new(3_f64.sqrt(), -1.));
    s.update_segment(b, negative.clone()).unwrap();
    let solved = s.solve().unwrap().segment(b).unwrap();
    assert!((solved.start() - negative.start()).norm() < 1e-7);
    assert!((solved.end() - negative.end()).norm() < 1e-7);
}

#[test]
fn dimensions_solve_across_geometric_scales() {
    for scale in [1e-6, 1., 1e6] {
        let options = SolveOptions {
            linear_tolerance: scale * 1e-8,
            ..SolveOptions::default()
        };
        let mut s = System2d::new(options);
        let a = s.add_point(Point2::new(2. * scale, -3. * scale)).unwrap();
        let b = s.add_point(Point2::new(4. * scale, 5. * scale)).unwrap();
        s.add_constraint(C::fixed(a)).unwrap();
        s.add_constraint(C::distance(a, b, 10. * scale)).unwrap();
        let solved = s.solve().unwrap();
        let distance = (solved.point(a).unwrap() - solved.point(b).unwrap()).norm();
        assert!((distance - 10. * scale).abs() < scale * 1e-7);
    }
}

#[test]
fn target_rounding_cannot_publish_a_degenerate_segment() {
    let mut s = System2d::default();
    let line = s
        .add_segment(TrimmedCurve2::segment(
            Point2::new(1e16, 0.),
            Point2::new(1e16 + 4., 0.),
        ))
        .unwrap();
    let old = s.solve().unwrap();
    let target = Point2::new(1e16 + 2., 0.);
    match s.solve_toward(&[
        Target2d::point(line.start(), target),
        Target2d::point(line.end(), target),
    ]) {
        Ok(solution) => {
            let a = solution.point(line.start()).unwrap();
            let b = solution.point(line.end()).unwrap();
            assert_ne!(a, b, "published endpoints must define a nonzero segment");
        }
        Err(SolveError::NonConvergence { .. }) => {
            assert_eq!(s.last_solution().unwrap().sequence(), old.sequence());
        }
        Err(error) => panic!("unexpected failure: {error:?}"),
    }
}
