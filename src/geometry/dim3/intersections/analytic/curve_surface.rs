//! Closed-form curve/surface intersections for recognized operand pairs.
//!
//! # Parameter convention
//!
//! `curve_u` is reported in the **curve's own** parameterization -- the one
//! [`Curve::point_at`] uses -- so a line reports arc length from its origin, a
//! circle an angle, and a bounded curve its normalized `[0, 1]`. The general
//! solver reports the parameter of the NURBS the curve converts to, which is
//! not the same thing and, for a circle, not even monotone in it. Callers that
//! feed `curve_u` back to [`Curve::derivative_at`] want this one.

use std::f64::consts::TAU;

use nalgebra::Vector3;

use super::roots::{harmonic_roots, quadratic_roots, wrapped};
use crate::geometry::counters::count_curve_surface_analytic_call;
use crate::geometry::dim3::intersections::error::IntersectionError;
use crate::geometry::dim3::intersections::options::IntersectionOptions;
use crate::geometry::{
    Circle, Curve, CurveSurfaceIntersection, CurveSurfaceIntersections, IntersectionCoverage,
    Interval, Line, Point3, Surface,
};

/// Whether a straight curve against this surface is answered in closed form.
///
/// Callers that would otherwise build a decomposition per query -- ray casting
/// builds one per ray -- ask this once and skip building it at all. It reports
/// the table's structural coverage, so a configuration that declines for its
/// own reasons still falls back on the query itself.
pub fn line_surface_is_analytic(surface: &Surface) -> bool {
    match surface {
        Surface::Plane(_) | Surface::Sphere(_) | Surface::Cylinder(_) => true,
        // A quarter-turn half angle makes the cone a plane, which its own
        // entry describes properly and this one cannot.
        Surface::Cone(cone) => cone.half_angle().tan().is_finite(),
        _ => false,
    }
}

/// Intersects a curve with a surface in closed form, or declines the pair.
///
/// `None` means the pair is not in the table and the caller must fall back to
/// the general solver. An `Ok` result is complete: every intersection over the
/// curve's domain is present, including tangencies, which are reported once
/// rather than as two coincident crossings.
pub fn intersect_analytic_curve_surface(
    curve: &Curve,
    surface: &Surface,
    options: IntersectionOptions,
) -> Option<Result<CurveSurfaceIntersections, IntersectionError>> {
    if !options.validate() {
        return Some(Err(IntersectionError::InvalidOptions));
    }
    let restriction = Restriction::of(curve)?;
    let solved = match curve.base() {
        Curve::Line(line) => line_surface(line, surface, options)?,
        Curve::Circle(circle) => circle_surface(circle, surface, options)?,
        _ => return None,
    };
    // A curve contained in a surface over an unbounded window is an overlap no
    // caller can act on, so it is declined rather than reported as infinite.
    if matches!(solved, Solved::Contained) && matches!(restriction, Restriction::Whole) {
        return None;
    }
    count_curve_surface_analytic_call();
    Some(Ok(restriction.apply(solved, curve, surface, options)))
}

/// What a solve produced, before the curve's own bounds are applied.
enum Solved {
    /// Parameters on the base curve, in the base curve's parameterization.
    Parameters(Vec<f64>),
    /// The whole base curve lies on the surface.
    Contained,
}

/// The window a curve's parameterization exposes, and how to report inside it.
enum Restriction {
    /// An unbounded base curve, reported in its own parameter.
    Whole,
    /// A curve periodic in its base parameter, reported folded into one period.
    Periodic(f64),
    /// A trimmed curve, reported normalized over `[0, 1]`.
    Bounded {
        bounds: Interval,
        period: Option<f64>,
    },
}

impl Restriction {
    fn of(curve: &Curve) -> Option<Self> {
        match curve {
            Curve::Line(_) => Some(Restriction::Whole),
            Curve::Circle(_) => Some(Restriction::Periodic(TAU)),
            Curve::Bounded(bounded) => Some(Restriction::Bounded {
                bounds: bounded.bounds(),
                period: match bounded.inner().base() {
                    Curve::Circle(_) => Some(TAU),
                    _ => None,
                },
            }),
            _ => None,
        }
    }

    /// Keeps the parameters inside the window and reports them in its terms.
    fn apply(
        &self,
        solved: Solved,
        curve: &Curve,
        surface: &Surface,
        options: IntersectionOptions,
    ) -> CurveSurfaceIntersections {
        let parameters = match solved {
            Solved::Contained => {
                return CurveSurfaceIntersections::new(
                    vec![CurveSurfaceIntersection::Overlap {
                        curve_interval: self.domain(),
                    }],
                    IntersectionCoverage::Complete,
                );
            }
            Solved::Parameters(parameters) => parameters,
        };
        let mut intersections = Vec::new();
        for parameter in parameters {
            let Some(reported) = self.report(parameter, options) else {
                continue;
            };
            let point = curve.point_at(reported);
            let Ok(uv) = surface.closest_parameter(point) else {
                continue;
            };
            intersections.push(CurveSurfaceIntersection::Point {
                point,
                curve_u: reported,
                surface_u: uv.x,
                surface_v: uv.y,
            });
        }
        CurveSurfaceIntersections::new(intersections, IntersectionCoverage::Complete)
    }

    /// The window itself, in the parameter the results are reported in.
    fn domain(&self) -> Interval {
        match self {
            Restriction::Whole => Interval::unbounded(),
            Restriction::Periodic(period) => Interval::new(0.0, *period),
            Restriction::Bounded { .. } => Interval::new(0.0, 1.0),
        }
    }

    /// Converts one base parameter, or drops it as outside the window.
    fn report(&self, parameter: f64, options: IntersectionOptions) -> Option<f64> {
        match self {
            Restriction::Whole => Some(parameter),
            Restriction::Periodic(period) => Some(parameter.rem_euclid(*period)),
            Restriction::Bounded { bounds, period } => {
                let span = bounds.end - bounds.start;
                if span.abs() <= options.parameter_tolerance {
                    return None;
                }
                // A periodic base curve reports one fixed branch, which need
                // not be the branch this trim lives on, so the parameter is
                // shifted by whole periods towards the bounds before it is
                // judged to be outside them.
                let parameter = match period {
                    Some(period) => {
                        let low = bounds.start.min(bounds.end) - options.parameter_tolerance;
                        low + (parameter - low).rem_euclid(*period)
                    }
                    None => parameter,
                };
                let local = (parameter - bounds.start) / span;
                let slack = options.parameter_tolerance / span.abs();
                (-slack..=1.0 + slack)
                    .contains(&local)
                    .then(|| local.clamp(0.0, 1.0))
            }
        }
    }
}

/// Solves a line against a recognized surface.
fn line_surface(line: &Line, surface: &Surface, options: IntersectionOptions) -> Option<Solved> {
    let origin = line.origin();
    let direction = *line.direction();
    match surface {
        Surface::Plane(plane) => {
            let normal = *plane.normal();
            let slope = direction.dot(&normal);
            let offset = (origin - plane.origin()).dot(&normal);
            if slope.abs() <= options.angular_tolerance {
                return Some(if offset.abs() <= options.linear_tolerance {
                    Solved::Contained
                } else {
                    Solved::Parameters(Vec::new())
                });
            }
            Some(Solved::Parameters(vec![-offset / slope]))
        }
        Surface::Sphere(sphere) => {
            let offset = origin - sphere.frame().origin;
            Some(Solved::Parameters(quadratic_roots(
                direction.norm_squared(),
                2.0 * direction.dot(&offset),
                offset.norm_squared() - sphere.radius() * sphere.radius(),
                options.angular_tolerance,
            )))
        }
        Surface::Cylinder(cylinder) => {
            let axis = *cylinder.axis();
            let offset = perpendicular_part(origin - cylinder.origin(), axis);
            let step = perpendicular_part(direction, axis);
            if step.norm() <= options.angular_tolerance {
                // Parallel to the axis: the line is either a ruling or misses
                // the cylinder entirely, and never crosses it.
                return Some(
                    if (offset.norm() - cylinder.radius).abs() <= options.linear_tolerance {
                        Solved::Contained
                    } else {
                        Solved::Parameters(Vec::new())
                    },
                );
            }
            Some(Solved::Parameters(quadratic_roots(
                step.norm_squared(),
                2.0 * step.dot(&offset),
                offset.norm_squared() - cylinder.radius * cylinder.radius,
                options.angular_tolerance,
            )))
        }
        Surface::Cone(cone) => {
            let slope = cone.half_angle().tan();
            if !slope.is_finite() {
                // A half angle of a quarter turn is a plane in the cone's
                // clothing; its own entry answers that shape properly.
                return None;
            }
            let local = |point: Point3| cone.frame().coordinates_of(point);
            let base = local(origin);
            let step = local(origin + direction) - base;
            // In the cone's frame the surface is `x^2 + y^2 = (r0 + z tan a)^2`,
            // and the line is affine, so the condition is a quadratic.
            let radial = cone.reference_radius();
            let a = step.x * step.x + step.y * step.y - (step.z * slope).powi(2);
            let b = 2.0
                * (base.x * step.x + base.y * step.y - (radial + base.z * slope) * step.z * slope);
            let c = base.x * base.x + base.y * base.y - (radial + base.z * slope).powi(2);
            Some(Solved::Parameters(quadratic_roots(
                a,
                b,
                c,
                options.angular_tolerance,
            )))
        }
        _ => None,
    }
}

/// Solves a circle against a recognized surface.
///
/// A circle meets a plane or a sphere where `A cos t + B sin t = C`, which is
/// closed form. Against a cylinder or a cone the same elimination leaves a
/// degree-four trigonometric polynomial, which none of the curve types can
/// carry and no closed form here would certify, so those decline.
fn circle_surface(
    circle: &Circle,
    surface: &Surface,
    options: IntersectionOptions,
) -> Option<Solved> {
    let centre = circle.plane().origin();
    let x_dir = *circle.plane().x_dir() * circle.radius();
    let y_dir = *circle.plane().y_dir() * circle.radius();
    let (a, b, c) = match surface {
        Surface::Plane(plane) => {
            let normal = *plane.normal();
            (
                x_dir.dot(&normal),
                y_dir.dot(&normal),
                -(centre - plane.origin()).dot(&normal),
            )
        }
        Surface::Sphere(sphere) => {
            let offset = centre - sphere.frame().origin;
            (
                2.0 * x_dir.dot(&offset),
                2.0 * y_dir.dot(&offset),
                sphere.radius() * sphere.radius()
                    - offset.norm_squared()
                    - circle.radius() * circle.radius(),
            )
        }
        _ => return None,
    };
    match harmonic_roots(a, b, c, options.angular_tolerance) {
        Some(roots) => Some(Solved::Parameters(roots.into_iter().map(wrapped).collect())),
        // The equation does not depend on the angle: the circle either lies on
        // the surface or is uniformly clear of it.
        None => Some(if c.abs() <= options.linear_tolerance {
            Solved::Contained
        } else {
            Solved::Parameters(Vec::new())
        }),
    }
}

/// The component of `vector` perpendicular to a unit `axis`.
fn perpendicular_part(vector: Vector3<f64>, axis: Vector3<f64>) -> Vector3<f64> {
    vector - axis * vector.dot(&axis)
}
