use nalgebra::Vector2;
use ngk::geometry::dim2::{
    curves::Circle2,
    trimmed::{TrimmedCircle2, TrimmedLine2},
    utils::Point2,
};
use ngk::geometry::parameter::{Fraction, NativeParam};
use ngk::solver::{
    ArcId, CircleId, Constraint2d as C, ConstraintId, PointId, SegmentId, System2d, TangencyMode,
    Target2d,
};
use radians::Rad64;
use std::f64::consts::{FRAC_PI_2, FRAC_PI_3, PI};

fn line(a: (f64, f64), b: (f64, f64)) -> TrimmedLine2 {
    TrimmedLine2::segment(Point2::new(a.0, a.1), Point2::new(b.0, b.1))
}
fn circle(x: f64, y: f64, r: f64) -> Circle2 {
    Circle2::new(Point2::new(x, y), Vector2::x(), r)
}
fn near(a: f64, b: f64) {
    assert!((a - b).abs() < 2e-7, "{a} != {b}");
}

#[test]
fn segment_directions_solve_from_orthogonal_initial_guesses() {
    for constraint in 0..5 {
        let mut s = System2d::default();
        let a = s
            .add_segment(SegmentId::new(1), line((0., 0.), (10., 0.)))
            .unwrap();
        let b = s
            .add_segment(SegmentId::new(2), line((3., 4.), (3., 12.)))
            .unwrap();
        s.add_constraint(ConstraintId::new(3), C::fixed(a)).unwrap();
        s.add_constraint(ConstraintId::new(4), C::length(b, 8.))
            .unwrap();
        let c = match constraint {
            0 => C::horizontal(b),
            1 => C::vertical(b),
            2 => C::parallel(a, b),
            3 => C::perpendicular(a, b),
            _ => C::angle(a, b, Rad64::new(FRAC_PI_3)),
        };
        s.add_constraint(ConstraintId::new(5), c).unwrap();
        let solved = s.solve().unwrap();
        let b = solved.segment(b).unwrap();
        let v = b.end() - b.start();
        match constraint {
            0 | 2 => near(v.y, 0.),
            1 | 3 => near(v.x, 0.),
            _ => near(v.y.atan2(v.x), FRAC_PI_3),
        }
        near(v.norm(), 8.);
    }
}

#[test]
fn unsigned_angles_keep_the_initial_side_and_oriented_angles_choose_a_side() {
    for side in [-1., 1.] {
        for oriented in [false, true] {
            let mut s = System2d::default();
            let a = s
                .add_segment(SegmentId::new(6), line((0., 0.), (10., 0.)))
                .unwrap();
            let b = s
                .add_segment(SegmentId::new(7), line((0., 0.), (8., side * 2.)))
                .unwrap();
            s.add_constraint(ConstraintId::new(8), C::fixed(a)).unwrap();
            s.add_constraint(ConstraintId::new(9), C::length(b, 8.))
                .unwrap();
            s.add_constraint(
                ConstraintId::new(10),
                if oriented {
                    C::oriented_angle(a, b, Rad64::new(FRAC_PI_3))
                } else {
                    C::angle(a, b, Rad64::new(FRAC_PI_3))
                },
            )
            .unwrap();
            let solved = s.solve().unwrap();
            let b = solved.segment(b).unwrap();
            let v = b.end() - b.start();
            near(
                v.y.atan2(v.x),
                if oriented {
                    FRAC_PI_3
                } else {
                    side * FRAC_PI_3
                },
            );
        }
    }
}

#[test]
fn zero_and_half_turn_angles_have_usable_derivatives() {
    for angle in [0., PI, -PI] {
        let mut s = System2d::default();
        let a = s
            .add_segment(SegmentId::new(11), line((0., 0.), (10., 0.)))
            .unwrap();
        let b = s
            .add_segment(SegmentId::new(12), line((1., 1.), (5., 8.)))
            .unwrap();
        s.add_constraint(ConstraintId::new(13), C::fixed(a))
            .unwrap();
        s.add_constraint(ConstraintId::new(14), C::length(b, 5.))
            .unwrap();
        s.add_constraint(
            ConstraintId::new(15),
            C::oriented_angle(a, b, Rad64::new(angle)),
        )
        .unwrap();
        let solved = s.solve().unwrap();
        let b = solved.segment(b).unwrap();
        let v = b.end() - b.start();
        near(v.y, 0.);
        near(v.x, if angle == 0. { 5. } else { -5. });
    }
}

#[test]
fn positional_relations_and_symmetries_recover_expected_coordinates() {
    for mode in 0..6 {
        let mut s = System2d::default();
        let axis = s
            .add_segment(SegmentId::new(16), line((-10., 0.), (10., 0.)))
            .unwrap();
        let a = s.add_point(PointId::new(17), Point2::new(2., 4.)).unwrap();
        let p = s.add_point(PointId::new(18), Point2::new(8., 7.)).unwrap();
        s.add_constraint(ConstraintId::new(19), C::fixed(axis))
            .unwrap();
        s.add_constraint(ConstraintId::new(20), C::fixed(a))
            .unwrap();
        let constraint = match mode {
            0 => C::point_on_line(p, axis),
            1 => C::distance_to_line(p, axis, 3.),
            2 => C::midpoint(p, axis),
            3 => C::symmetric_about_line(a, p, axis),
            4 => C::symmetric_about_point(a, p, axis.start()),
            _ => C::midpoint_between(p, a, axis.end()),
        };
        s.add_constraint(ConstraintId::new(21), constraint).unwrap();
        let solved = s.solve().unwrap();
        let p = solved.point(p).unwrap();
        match mode {
            0 => near(p.y, 0.),
            1 => near(p.y, 3.),
            2 => {
                near(p.x, 0.);
                near(p.y, 0.);
            }
            3 => {
                near(p.x, 2.);
                near(p.y, -4.);
            }
            4 => {
                near(p.x, -22.);
                near(p.y, -4.);
            }
            _ => {
                near(p.x, 6.);
                near(p.y, 2.);
            }
        }
    }
}

#[test]
fn signed_offsets_and_distance_from_coincident_points_are_solvable() {
    let mut s = System2d::default();
    let a = s.add_point(PointId::new(22), Point2::origin()).unwrap();
    let b = s.add_point(PointId::new(23), Point2::origin()).unwrap();
    s.add_constraint(ConstraintId::new(24), C::fixed(a))
        .unwrap();
    s.add_constraint(ConstraintId::new(25), C::distance(a, b, 5.))
        .unwrap();
    let solved = s.solve().unwrap();
    near(solved.point(b).unwrap().coords.norm(), 5.);
    let mut s = System2d::default();
    let a = s.add_point(PointId::new(26), Point2::new(5., 4.)).unwrap();
    let b = s.add_point(PointId::new(27), Point2::new(10., 8.)).unwrap();
    s.add_constraint(ConstraintId::new(28), C::fixed(a))
        .unwrap();
    s.add_constraint(ConstraintId::new(29), C::offset_x(a, b, -3.))
        .unwrap();
    s.add_constraint(ConstraintId::new(30), C::offset_y(a, b, 2.))
        .unwrap();
    let solved = s.solve().unwrap();
    near(solved.point(b).unwrap().x, 2.);
    near(solved.point(b).unwrap().y, 6.);
}

#[test]
fn equal_lengths_radii_and_concentricity_apply_to_supported_geometry() {
    let mut s = System2d::default();
    let a = s
        .add_segment(SegmentId::new(31), line((0., 0.), (6., 8.)))
        .unwrap();
    let b = s
        .add_segment(SegmentId::new(32), line((2., 1.), (4., 1.)))
        .unwrap();
    s.add_constraint(ConstraintId::new(33), C::fixed(a))
        .unwrap();
    s.add_constraint(ConstraintId::new(34), C::equal_length(a, b))
        .unwrap();
    let circle = s.add_circle(CircleId::new(35), circle(3., 5., 7.)).unwrap();
    let arc = s
        .add_arc(
            ArcId::new(36),
            TrimmedCircle2::arc(Point2::new(9., 8.), Vector2::y(), 2., 1.5 * PI),
        )
        .unwrap();
    s.add_constraint(ConstraintId::new(37), C::fixed(circle))
        .unwrap();
    s.add_constraint(ConstraintId::new(38), C::equal_radius(circle, arc))
        .unwrap();
    s.add_constraint(ConstraintId::new(39), C::concentric(circle, arc))
        .unwrap();
    let solved = s.solve().unwrap();
    let b = solved.segment(b).unwrap();
    near((b.end() - b.start()).norm(), 10.);
    let arc = solved.arc(arc).unwrap();
    near((arc.start() - Point2::new(3., 5.)).norm(), 7.);
    near(arc.interval().delta(), 1.5 * PI);
}

#[test]
fn circle_dimensions_and_line_tangency_preserve_the_initial_side() {
    for side in [-1., 1.] {
        let mut s = System2d::default();
        let axis = s
            .add_segment(SegmentId::new(40), line((-10., 0.), (10., 0.)))
            .unwrap();
        let c = s
            .add_circle(CircleId::new(41), circle(0., side * 7., 2.))
            .unwrap();
        s.add_constraint(ConstraintId::new(42), C::fixed(axis))
            .unwrap();
        s.add_constraint(ConstraintId::new(43), C::diameter(c, 6.))
            .unwrap();
        s.add_constraint(ConstraintId::new(44), C::tangent_line_circle(axis, c))
            .unwrap();
        let solved = s.solve().unwrap();
        let c = solved.circle(c).unwrap();
        near(c.radius(), 3.);
        near(c.center().y, side * 3.);
    }
}

#[test]
fn circle_tangency_supports_external_internal_and_automatic_modes() {
    for mode in [
        TangencyMode::External,
        TangencyMode::Internal,
        TangencyMode::Automatic,
        TangencyMode::FirstContainsSecond,
    ] {
        let mut s = System2d::default();
        let a = s
            .add_circle(CircleId::new(45), circle(0., 0., 10.))
            .unwrap();
        let b = s.add_circle(CircleId::new(46), circle(4., 1., 3.)).unwrap();
        s.add_constraint(ConstraintId::new(47), C::fixed(a))
            .unwrap();
        s.add_constraint(ConstraintId::new(48), C::radius(b, 3.))
            .unwrap();
        s.add_constraint(ConstraintId::new(49), C::tangent_circles(a, b, mode))
            .unwrap();
        let solved = s.solve().unwrap();
        let b = solved.circle(b).unwrap();
        near(
            b.center().coords.norm(),
            if mode == TangencyMode::External {
                13.
            } else {
                7.
            },
        );
    }
}

#[test]
fn major_clockwise_arc_endpoints_move_without_losing_sweep_or_direction() {
    let mut s = System2d::default();
    let arc = s
        .add_arc(
            ArcId::new(50),
            TrimmedCircle2::arc(Point2::origin(), Vector2::x(), 10., -1.5 * PI),
        )
        .unwrap();
    s.add_constraint(ConstraintId::new(51), C::fixed_point(arc.center()))
        .unwrap();
    s.add_constraint(ConstraintId::new(52), C::radius(arc, 10.))
        .unwrap();
    let first = s.solve().unwrap();
    near(first.arc(arc).unwrap().interval().delta(), -1.5 * PI);
    let moved = s
        .solve_toward(&[
            Target2d::point(arc.start(), Point2::new(0., 10.)),
            Target2d::point(arc.end(), Point2::new(-10., 0.)),
        ])
        .unwrap();
    near(moved.arc(arc).unwrap().interval().delta(), -1.5 * PI);
    assert!((moved.point(arc.start()).unwrap() - Point2::new(0., 10.)).norm() < 1e-5);
    assert!((moved.point(arc.end()).unwrap() - Point2::new(-10., 0.)).norm() < 1e-5);
    assert_eq!(moved.report().degrees_of_freedom, 2);
}

#[test]
fn fixed_curve_sample_targets_resize_circles_and_drag_segment_interiors() {
    let mut s = System2d::default();
    let c = s.add_circle(CircleId::new(53), circle(0., 0., 3.)).unwrap();
    s.add_constraint(ConstraintId::new(54), C::fixed_point(c.center()))
        .unwrap();
    let solved = s
        .solve_toward(&[Target2d::on_circle(
            c,
            NativeParam::new(FRAC_PI_2),
            Point2::new(0., 8.),
        )])
        .unwrap();
    near(solved.circle(c).unwrap().radius(), 8.);
    let a = s
        .add_segment(SegmentId::new(55), line((0., 0.), (10., 0.)))
        .unwrap();
    let solved = s
        .solve_toward(&[Target2d::on_segment(
            a,
            Fraction::new(0.5),
            Point2::new(8., 4.),
        )])
        .unwrap();
    let line = solved.segment(a).unwrap();
    near((line.start().x + line.end().x) / 2., 8.);
    near((line.start().y + line.end().y) / 2., 4.);
}

#[test]
fn point_on_arc_uses_the_support_outside_its_span() {
    let mut s = System2d::default();
    let arc = s
        .add_arc(
            ArcId::new(56),
            TrimmedCircle2::arc(Point2::origin(), Vector2::x(), 10., FRAC_PI_2),
        )
        .unwrap();
    let p = s
        .add_point(PointId::new(57), Point2::new(-10., 0.))
        .unwrap();
    s.add_constraint(ConstraintId::new(58), C::fixed(arc))
        .unwrap();
    s.add_constraint(ConstraintId::new(59), C::fixed(p))
        .unwrap();
    s.add_constraint(ConstraintId::new(60), C::point_on_circle(p, arc))
        .unwrap();
    assert!(s.solve().is_ok());
}
