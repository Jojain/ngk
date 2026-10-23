use super::{
    curve::Curve,
    differentiate::{Jet, JetVector2},
    evaluate, prepare,
};
use crate::geometry::dim2::{curves::Circle2, trimmed::TrimmedCurve2, utils::Point2};
use crate::solver::{Constraint2d as C, System2d, TangencyMode, Target2d};
use nalgebra::Vector2;

#[test]
fn geometric_jacobians_match_independent_centered_differences() {
    let mut s = System2d::default();
    let a = s
        .add_segment(TrimmedCurve2::segment(
            Point2::new(1., 2.),
            Point2::new(7., 5.),
        ))
        .unwrap();
    let b = s
        .add_segment(TrimmedCurve2::segment(
            Point2::new(3., -2.),
            Point2::new(8., 6.),
        ))
        .unwrap();
    let c = s
        .add_circle(Circle2::new(Point2::new(-2., 4.), Vector2::x(), 3.))
        .unwrap();
    let arc = s
        .add_arc(TrimmedCurve2::arc(
            Point2::new(5., -4.),
            Vector2::new(1., 2.),
            2.,
            -2.4,
        ))
        .unwrap();
    let constraints = vec![
        C::coincident(a.end(), arc.start()),
        C::point_on_line(arc.end(), b),
        C::point_on_circle(a.start(), c),
        C::horizontal(a),
        C::vertical(b),
        C::distance(a.start(), arc.end(), 4.),
        C::distance_to_line(c.center(), b, 2.),
        C::offset_x(a.start(), b.end(), 1.),
        C::offset_y(a.start(), b.end(), 2.),
        C::length(a, 10.),
        C::radius(c, 5.),
        C::equal_length(a, b),
        C::equal_radius(c, arc),
        C::parallel(a, b),
        C::perpendicular(a, b),
        C::angle(a, b, 0.4),
        C::oriented_angle(a, b, -0.3),
        C::concentric(c, arc),
        C::midpoint(arc.center(), a),
        C::symmetric_about_line(c.center(), arc.end(), b),
        C::symmetric_about_point(a.end(), arc.start(), c.center()),
        C::fixed_point(b.start()),
        C::fixed(arc),
        C::tangent_line_circle(a, c),
        C::tangent_circles(c, arc, TangencyMode::External),
    ];
    for c in constraints {
        s.add_constraint(c).unwrap();
    }
    let targets = vec![
        Target2d::point(arc.end(), Point2::new(2., 3.)),
        Target2d::on_arc(arc, 0.4, Point2::new(1., 4.)).with_weight(3.),
        Target2d::on_circle(c, 0.8, Point2::new(-5., 4.)),
        Target2d::on_segment(b, 0.3, Point2::origin()),
    ];
    let components = prepare::prepare(s.owner, &s.geometry, &s.constraints, &targets, false);
    for component in components {
        for soft in [false, true] {
            let evaluate = |x: &nalgebra::DVector<f64>| {
                if soft {
                    evaluate::soft(&component, x)
                } else {
                    evaluate::hard(&component, x)
                }
            };
            let baseline = evaluate(&component.initial);
            for column in 0..component.initial.len() {
                let step = 1e-6;
                let mut plus = component.initial.clone();
                plus[column] += step;
                let mut minus = component.initial.clone();
                minus[column] -= step;
                let difference = (evaluate(&plus).values - evaluate(&minus).values) / (2. * step);
                for row in 0..difference.len() {
                    let actual = baseline.jacobian[(row, column)];
                    let expected = difference[row];
                    assert!(
                        (actual - expected).abs() < 2e-6 * expected.abs().max(1.),
                        "soft={soft}, row={row}, col={column}: {actual} != {expected}"
                    );
                }
            }
        }
    }
}

#[test]
fn curve_points_and_derivatives_match_centered_differences() {
    let shapes: [&[f64]; 3] = [&[1., 2., 7., -3.], &[-2., 4., 3.], &[5., -4., 2., 1.1, 2.4]];
    for shape in shapes {
        let t = 0.35;
        let t_index = shape.len();
        let at = |p: &[f64], t: f64, derivative: bool| {
            let curve = curve(p, t_index);
            let t = Jet::variable(t, t_index);
            if derivative {
                curve.derivative_at(t)
            } else {
                curve.point_at(t)
            }
        };
        let step = 1e-6;
        // The hand-written derivative is the point's rate of change along t.
        let ahead = at(shape, t + step, false);
        let behind = at(shape, t - step, false);
        let tangent = at(shape, t, true);
        for (actual, (a, b)) in [
            (&tangent.x, (&ahead.x, &behind.x)),
            (&tangent.y, (&ahead.y, &behind.y)),
        ] {
            let expected = (a.value - b.value) / (2. * step);
            assert!(
                (actual.value - expected).abs() < 1e-6 * expected.abs().max(1.),
                "{shape:?}: derivative {} != {expected}",
                actual.value
            );
        }
        // Both evaluations differentiate exactly through t and every shape parameter.
        for derivative in [false, true] {
            let baseline = at(shape, t, derivative);
            for unknown in 0..=t_index {
                let shifted = |delta: f64| {
                    let mut p = shape.to_vec();
                    if unknown < t_index {
                        p[unknown] += delta;
                        at(&p, t, derivative)
                    } else {
                        at(&p, t + delta, derivative)
                    }
                };
                let (plus, minus) = (shifted(step), shifted(-step));
                for (actual, (a, b)) in [
                    (&baseline.x, (&plus.x, &minus.x)),
                    (&baseline.y, (&plus.y, &minus.y)),
                ] {
                    let expected = (a.value - b.value) / (2. * step);
                    let slope = actual
                        .gradient
                        .iter()
                        .find(|(index, _)| *index == unknown)
                        .map_or(0., |(_, slope)| *slope);
                    assert!(
                        (slope - expected).abs() < 2e-6 * expected.abs().max(1.),
                        "{shape:?}, derivative={derivative}, unknown {unknown}: {slope} != {expected}"
                    );
                }
            }
        }
    }
}

/// A segment, circle or arc, told apart by its parameter count, with every
/// shape parameter an unknown numbered from 0.
fn curve(p: &[f64], count: usize) -> Curve {
    let v = |i: usize| Jet::variable(p[i], i);
    let point = |i: usize| JetVector2 {
        x: v(i),
        y: v(i + 1),
    };
    match count {
        4 => Curve::Segment {
            start: point(0),
            end: point(2),
        },
        3 => Curve::Circle {
            center: point(0),
            radius: v(2),
            axis: Vector2::new(0.6, 0.8),
            sense: -1.,
        },
        _ => Curve::Arc {
            center: point(0),
            radius: v(2),
            start_angle: v(3),
            sweep: v(4),
            sense: -1.,
        },
    }
}
