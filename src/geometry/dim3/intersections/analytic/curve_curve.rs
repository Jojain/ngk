//! Closed-form curve/curve intersections for recognized operand pairs.
//!
//! Parameters follow the same convention as the curve/surface table: each is
//! reported in its own curve's parameterization, not in the NURBS one.

use std::f64::consts::TAU;

use nalgebra::Vector3;

use super::roots::{harmonic_roots, quadratic_roots, wrapped};
use crate::geometry::dim3::intersections::error::IntersectionError;
use crate::geometry::dim3::intersections::options::IntersectionOptions;
use crate::geometry::parameter::NativeParam;
use crate::geometry::{
    Circle, Curve, CurveCurveIntersection, CurveCurveIntersections, IntersectionCoverage, Interval,
    Line, Point3,
};

/// Intersects two curves in closed form, or declines the pair.
///
/// Overlaps are reported only for the exactly coincident supports the table
/// recognizes -- two collinear lines, two identical circles -- because those
/// are the cases the embedding search cannot settle at all. Everything else
/// is a finite set of crossings and tangencies.
pub fn intersect_analytic_curves(
    a: &Curve,
    b: &Curve,
    options: IntersectionOptions,
) -> Option<Result<CurveCurveIntersections, IntersectionError>> {
    if !options.validate() {
        return Some(Err(IntersectionError::InvalidOptions));
    }
    let first = Window::of(a)?;
    let second = Window::of(b)?;
    let solved = match (a, b) {
        (Curve::Line(first), Curve::Line(second)) => line_line(first, second, options),
        (Curve::Line(line), Curve::Circle(circle)) => line_circle(line, circle, options),
        (Curve::Circle(circle), Curve::Line(line)) => swapped(line_circle(line, circle, options)),
        (Curve::Circle(first), Curve::Circle(second)) => circle_circle(first, second, options),
        _ => return None,
    }?;
    // An overlap is reported as one interval per curve, which only a trimmed
    // curve has: an unbounded support would give an unbounded region, and a
    // periodic one would need the interval to cross its own seam. Both decline
    // to the general solver rather than reporting something unusable.
    if matches!(solved, Solved::Coincident) {
        return None;
    }
    Some(Ok(report(solved, &first, &second, a, b, options)))
}

/// Pairs of base parameters, or a statement that the supports coincide.
enum Solved {
    Crossings(Vec<(f64, f64)>),
    /// The supports are the same curve; the shared interval is each window.
    Coincident,
}

/// Reverses a solve's operand order.
fn swapped(solved: Option<Solved>) -> Option<Solved> {
    Some(match solved? {
        Solved::Crossings(pairs) => {
            Solved::Crossings(pairs.into_iter().map(|(a, b)| (b, a)).collect())
        }
        Solved::Coincident => Solved::Coincident,
    })
}

/// The parameter window one curve exposes, and how results read inside it.
enum Window {
    Whole,
    Periodic(f64),
}

impl Window {
    fn of(curve: &Curve) -> Option<Self> {
        match curve {
            Curve::Line(_) => Some(Window::Whole),
            Curve::Circle(_) => Some(Window::Periodic(TAU)),
            _ => None,
        }
    }

    fn report(&self, parameter: f64, _options: IntersectionOptions) -> Option<f64> {
        match self {
            Window::Whole => Some(parameter),
            Window::Periodic(period) => Some(parameter.rem_euclid(*period)),
        }
    }
}

/// Turns base parameters into the reported results.
fn report(
    solved: Solved,
    first: &Window,
    second: &Window,
    a: &Curve,
    b: &Curve,
    options: IntersectionOptions,
) -> CurveCurveIntersections {
    let pairs = match solved {
        Solved::Coincident => {
            // Coincident *supports* are not a coincident *overlap*: two
            // collinear segments share only the part both windows cover, and
            // reporting each whole window instead would move both edges' split
            // points to their far ends.
            let Some((interval_a, interval_b)) = shared_window(a, b, options) else {
                return CurveCurveIntersections::new(Vec::new(), IntersectionCoverage::Complete);
            };
            return CurveCurveIntersections::new(
                vec![CurveCurveIntersection::Overlap {
                    interval_a,
                    interval_b,
                }],
                IntersectionCoverage::Complete,
            );
        }
        Solved::Crossings(pairs) => pairs,
    };
    let mut intersections = Vec::new();
    for (u, v) in pairs {
        let (Some(u), Some(v)) = (first.report(u, options), second.report(v, options)) else {
            continue;
        };
        let point_a = a.point_at(NativeParam::new(u));
        let point_b = b.point_at(NativeParam::new(v));
        // The two supports were solved independently; disagreement here means
        // the crossing is outside one of them however the parameters mapped.
        if (point_a - point_b).norm() > options.linear_tolerance {
            continue;
        }
        intersections.push(CurveCurveIntersection::Point {
            point: Point3::from((point_a.coords + point_b.coords) * 0.5),
            u_a: u,
            u_b: v,
        });
    }
    CurveCurveIntersections::new(intersections, IntersectionCoverage::Complete)
}

/// The part of two coincident supports that both curves' windows cover.
///
/// Each window's ends are read in the other curve's parameter and clipped
/// against it, which needs nothing but the two curves' own inverses.
fn shared_window(
    a: &Curve,
    b: &Curve,
    options: IntersectionOptions,
) -> Option<(Interval, Interval)> {
    let ends = |curve: &Curve| {
        [
            curve.point_at(NativeParam::new(0.0)),
            curve.point_at(NativeParam::new(1.0)),
        ]
    };
    let [b_start, b_end] = ends(b);
    let (first, second) = (a.parameter_at(b_start), a.parameter_at(b_end));
    let low = first.min(second).max(NativeParam::new(0.0));
    let high = first.max(second).min(NativeParam::new(1.0));
    if high - low <= options.parameter_tolerance {
        return None;
    }
    let interval_a = Interval::new(low, high);
    let mapped = |parameter: f64| b.parameter_at(a.point_at(NativeParam::new(parameter)));
    Some((
        interval_a,
        Interval::new(mapped(low.value()), mapped(high.value())),
    ))
}

/// Two lines cross at one point, are collinear, or are skew or parallel.
fn line_line(a: &Line, b: &Line, options: IntersectionOptions) -> Option<Solved> {
    let (p, q) = (a.origin(), b.origin());
    let (u, v) = (
        a.derivative_at(NativeParam::new(0.0), 1),
        b.derivative_at(NativeParam::new(0.0), 1),
    );
    let offset = q - p;
    let cross = u.cross(&v);
    let denominator = cross.norm_squared();
    if denominator <= options.angular_tolerance.powi(2) * u.norm_squared() * v.norm_squared() {
        // Parallel: collinear if one origin lies on the other line.
        return Some(if offset.cross(&u).norm() <= options.linear_tolerance {
            Solved::Coincident
        } else {
            Solved::Crossings(Vec::new())
        });
    }
    // Skew lines have no intersection; the closest-approach parameters are
    // still what a crossing would be, so they are computed once and the
    // separation decides whether they describe one.
    let s = offset.cross(&v).dot(&cross) / denominator;
    let t = offset.cross(&u).dot(&cross) / denominator;
    if (a.point_at(NativeParam::new(s)) - b.point_at(NativeParam::new(t))).norm()
        > options.linear_tolerance
    {
        return Some(Solved::Crossings(Vec::new()));
    }
    Some(Solved::Crossings(vec![(s, t)]))
}

/// A line meets a circle in its plane, at its plane, or nowhere.
fn line_circle(line: &Line, circle: &Circle, options: IntersectionOptions) -> Option<Solved> {
    let normal = *circle.plane().normal();
    let origin = line.origin();
    let direction = line.derivative_at(NativeParam::new(0.0), 1);
    let slope = direction.dot(&normal);
    let offset = (origin - circle.plane().origin()).dot(&normal);
    let parameters = if slope.abs() <= options.angular_tolerance * direction.norm() {
        if offset.abs() > options.linear_tolerance {
            return Some(Solved::Crossings(Vec::new()));
        }
        // Coplanar: the line meets the circle where it is at the radius.
        let radial = origin - circle.plane().origin();
        quadratic_roots(
            direction.norm_squared(),
            2.0 * direction.dot(&radial),
            radial.norm_squared() - circle.radius() * circle.radius(),
            options.angular_tolerance,
        )
    } else {
        vec![-offset / slope]
    };
    Some(Solved::Crossings(
        parameters
            .into_iter()
            .filter_map(|parameter| {
                let point = line.point_at(NativeParam::new(parameter));
                let radial = point - circle.plane().origin();
                (radial.norm() - circle.radius())
                    .abs()
                    .le(&options.linear_tolerance)
                    .then(|| (parameter, wrapped(circle.parameter_at(point).value())))
            })
            .collect(),
    ))
}

/// Two circles meet at points, coincide, or miss.
fn circle_circle(a: &Circle, b: &Circle, options: IntersectionOptions) -> Option<Solved> {
    let (a_normal, b_normal) = (*a.plane().normal(), *b.plane().normal());
    let parallel = a_normal.cross(&b_normal).norm() <= options.angular_tolerance;
    let concentric = (a.plane().origin() - b.plane().origin()).norm() <= options.linear_tolerance;
    if parallel && concentric {
        return Some(
            if (a.radius() - b.radius()).abs() <= options.linear_tolerance {
                Solved::Coincident
            } else {
                Solved::Crossings(Vec::new())
            },
        );
    }

    // Where the first circle meets the second's plane is `A cos t + B sin t = C`
    // whether the planes are parallel or not; the candidates it produces are
    // then kept only where they also sit at the second circle's radius.
    let centre = a.plane().origin();
    let x_dir = *a.plane().x_dir() * a.radius();
    let y_dir = *a.plane().y_dir() * a.radius();
    let candidates = harmonic_roots(
        x_dir.dot(&b_normal),
        y_dir.dot(&b_normal),
        -(centre - b.plane().origin()).dot(&b_normal),
        options.angular_tolerance,
    );
    let candidates = match candidates {
        Some(candidates) => candidates,
        None => {
            // The first circle is parallel to the second's plane: it either
            // lies in that plane, where every angle is a candidate, or misses
            // it entirely.
            if (centre - b.plane().origin()).dot(&b_normal).abs() > options.linear_tolerance {
                return Some(Solved::Crossings(Vec::new()));
            }
            return Some(coplanar_circles(a, b, options));
        }
    };
    Some(Solved::Crossings(
        candidates
            .into_iter()
            .filter_map(|angle| {
                let point = a.point_at(NativeParam::new(angle));
                let radial = point - b.plane().origin();
                ((radial.norm() - b.radius()).abs() <= options.linear_tolerance)
                    .then(|| (wrapped(angle), wrapped(b.parameter_at(point).value())))
            })
            .collect(),
    ))
}

/// Two distinct coplanar circles meet on their radical line.
fn coplanar_circles(a: &Circle, b: &Circle, options: IntersectionOptions) -> Solved {
    let offset = b.plane().origin() - a.plane().origin();
    let separation = offset.norm();
    if separation <= options.linear_tolerance {
        return Solved::Crossings(Vec::new());
    }
    let along = offset / separation;
    let distance = (separation * separation + a.radius() * a.radius() - b.radius() * b.radius())
        / (2.0 * separation);
    let half_chord_squared = a.radius() * a.radius() - distance * distance;
    if half_chord_squared < -options.linear_tolerance * a.radius() {
        return Solved::Crossings(Vec::new());
    }
    let foot = a.plane().origin() + along * distance;
    let across: Vector3<f64> = a.plane().normal().cross(&along);
    let half_chord = half_chord_squared.max(0.0).sqrt();
    let offsets: Vec<f64> = if half_chord <= options.linear_tolerance {
        vec![0.0]
    } else {
        vec![half_chord, -half_chord]
    };
    Solved::Crossings(
        offsets
            .into_iter()
            .map(|side| {
                let point: Point3 = foot + across * side;
                (
                    wrapped(a.parameter_at(point).value()),
                    wrapped(b.parameter_at(point).value()),
                )
            })
            .collect(),
    )
}
