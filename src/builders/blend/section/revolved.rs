//! A circular edge between two surfaces of revolution about its own axis.
//!
//! A cylindrical boss on a block, a hole's rim, a cone meeting a plane: the
//! whole crease is one meridian section turned about the axis, so the blend
//! is too. One section is solved with the general solver, at the edge's start,
//! and revolved — a fillet's ball centre sweeps a circle and the round is the
//! torus about it; a chamfer's segment sweeps a cone, or a cylinder where
//! both rails share a radius. The rails are circles about the axis.
//!
//! Everything shares one frame: its `x` points at the edge's start and its
//! `z` runs so the edge turns positively about it. Each rail is then the
//! isoline of the blend surface at its level, with the surface's `u` as its
//! own parameter, and its pcurve on the blend face is exact.

use std::f64::consts::{PI, TAU};

use nalgebra::Vector3;

use super::super::errors::BlendError;
use super::super::law::{BlendLaw, ChamferLaw, FilletLaw};
use super::contact::{ball_at, setbacks_at};
use super::crease::Crease;
use super::edge_section::{EdgeSection, SectionForm, SectionShape};
use crate::geometry::parameter::Fraction;
use crate::geometry::{
    Circle, Cone, Curve, Cylinder, Frame, LINEAR_TOLERANCE, Plane, Point3, Surface, Torus,
};

/// Fractions a curve is sampled at to recognise it as a circle.
const CIRCLE_SAMPLES: usize = 8;

/// The axis of a circle: its centre and unit normal.
#[derive(Debug, Clone, Copy)]
struct CircleAxis {
    centre: Point3,
    normal: Vector3<f64>,
}

/// Solves the section of a circular edge between two surfaces revolving
/// about the circle's axis, or `None` when the edge and faces are not that.
pub(super) fn revolved_section(
    crease: &Crease<'_>,
    law: BlendLaw,
    convex: bool,
) -> Option<Result<EdgeSection, BlendError>> {
    let axis = circle_axis(crease)?;
    if !crease
        .surfaces
        .iter()
        .all(|surface| revolves_about(surface, axis))
    {
        return None;
    }
    Some(solve_revolved(crease, law, convex, axis))
}

fn solve_revolved(
    crease: &Crease<'_>,
    law: BlendLaw,
    convex: bool,
    axis: CircleAxis,
) -> Result<EdgeSection, BlendError> {
    let unsolved = BlendError::EdgeDoesNotFit {
        edge: crease.key,
        reason: "no blend touches both of its faces",
    };
    let start = crease.span.point_at(Fraction::START);
    let tangent = crease.span.derivative_at(Fraction::START, 1);
    let radial = start - axis.centre;
    let x = (radial - axis.normal * radial.dot(&axis.normal)).normalize();
    let z = x.cross(&tangent).normalize();
    let origin = axis.centre;
    // Height along the axis and distance from it, in the shared frame.
    let meridian = |point: Point3| {
        let offset = point - origin;
        let height = offset.dot(&z);
        (height, (offset - z * height).norm())
    };
    let rail = |point: Point3| {
        let (height, radius) = meridian(point);
        Curve::Circle(Circle::new(Plane::new(origin + z * height, x, z), radius))
    };

    match law {
        BlendLaw::Fillet(FilletLaw::Radius(radius)) => {
            let ball = ball_at(crease, Fraction::START, radius, convex, None).ok_or(unsolved)?;
            let (height, spine) = meridian(ball.centre);
            if spine <= radius + LINEAR_TOLERANCE.sqrt() {
                return Err(BlendError::EdgeDoesNotFit {
                    edge: crease.key,
                    reason: "the round is wider than the circle it runs round",
                });
            }
            let level = |point: Point3| {
                let (point_height, point_radius) = meridian(point);
                (point_height - height).atan2(point_radius - spine)
            };
            let first = level(ball.contacts[0].point);
            let second = first + wrapped(level(ball.contacts[1].point) - first);
            let frame = Frame::from_xz(origin + z * height, x, z);
            Ok(EdgeSection {
                surface: Surface::Torus(Torus::new(frame, spine, radius)),
                rails: ball.contacts.map(|foot| rail(foot.point)),
                convex,
                form: SectionForm::Revolved {
                    levels: [first, second],
                    shape: SectionShape::Arc { radius },
                },
            })
        }
        BlendLaw::Chamfer(ChamferLaw::Distance(distance)) => {
            let feet = setbacks_at(crease, Fraction::START, distance, None).ok_or(unsolved)?;
            let (first_height, first_radius) = meridian(feet[0].point);
            let (second_height, second_radius) = meridian(feet[1].point);
            let rise = second_height - first_height;
            let spread = second_radius - first_radius;
            if rise.abs() <= LINEAR_TOLERANCE.sqrt() {
                return Err(BlendError::UnsupportedEdge {
                    edge: crease.key,
                    reason: "its chamfer is a flat ring square to its axis",
                });
            }
            let base = origin + z * first_height;
            let (surface, levels) = if spread.abs() <= LINEAR_TOLERANCE.sqrt() {
                let wall = Cylinder::new(base, x, z, first_radius);
                (Surface::Cylinder(wall), [0.0, rise])
            } else {
                // The half-angle is kept inside a quarter turn, so the cone's
                // slant runs down the axis when the chamfer does.
                let half_angle = (spread / rise).atan();
                let cone = Cone::new(Frame::from_xz(base, x, z), first_radius, half_angle);
                (Surface::Cone(cone), [0.0, rise / half_angle.cos()])
            };
            Ok(EdgeSection {
                surface,
                rails: feet.map(|foot| rail(foot.point)),
                convex,
                form: SectionForm::Revolved {
                    levels,
                    shape: SectionShape::Segment,
                },
            })
        }
    }
}

/// The axis of the circle the edge runs along, if it runs along one.
///
/// A Boolean may write a circle as a NURBS curve, so the curve's type is not
/// asked: the circle through three points of the span is, and every sample
/// must lie on it.
fn circle_axis(crease: &Crease<'_>) -> Option<CircleAxis> {
    if let Curve::Circle(circle) = crease.span.curve() {
        let plane = circle.plane();
        return Some(CircleAxis {
            centre: plane.origin(),
            normal: *plane.normal(),
        });
    }
    let at = |fraction: f64| crease.span.point_at(Fraction::new(fraction));
    let [a, b, c] = [at(0.0), at(1.0 / 3.0), at(2.0 / 3.0)];
    let (ab, ac) = (b - a, c - a);
    let normal = ab.cross(&ac);
    if normal.norm() <= LINEAR_TOLERANCE {
        return None;
    }
    // The circumcentre of the triangle abc.
    let centre = a
        + (normal.cross(&ab) * ac.norm_squared() + ac.cross(&normal) * ab.norm_squared())
            / (2.0 * normal.norm_squared());
    let radius = (a - centre).norm();
    let normal = normal.normalize();
    let on_circle = (0..=CIRCLE_SAMPLES).all(|index| {
        let point = at(index as f64 / CIRCLE_SAMPLES as f64);
        let offset = point - centre;
        offset.dot(&normal).abs() <= LINEAR_TOLERANCE.sqrt()
            && (offset.norm() - radius).abs() <= LINEAR_TOLERANCE.sqrt()
    });
    on_circle.then_some(CircleAxis { centre, normal })
}

/// Whether `surface` is carried onto itself by every turn about `axis`.
fn revolves_about(surface: &Surface, axis: CircleAxis) -> bool {
    let parallel = |direction: Vector3<f64>| {
        direction.normalize().cross(&axis.normal).norm() <= LINEAR_TOLERANCE.sqrt()
    };
    let on_axis = |point: Point3| {
        let offset = point - axis.centre;
        (offset - axis.normal * offset.dot(&axis.normal)).norm() <= LINEAR_TOLERANCE.sqrt()
    };
    match surface {
        Surface::Plane(plane) => parallel(*plane.normal()),
        Surface::Cylinder(cylinder) => parallel(*cylinder.axis()) && on_axis(cylinder.origin()),
        Surface::Cone(cone) => parallel(*cone.frame().z_dir) && on_axis(cone.frame().origin),
        Surface::Torus(torus) => parallel(*torus.frame().z_dir) && on_axis(torus.frame().origin),
        Surface::Sphere(sphere) => on_axis(sphere.frame().origin),
        Surface::Revolution(_) | Surface::Ruled(_) | Surface::Nurbs(_) => false,
    }
}

/// An angle moved by whole turns into `(-π, π]`.
fn wrapped(angle: f64) -> f64 {
    let turned = angle.rem_euclid(TAU);
    if turned > PI { turned - TAU } else { turned }
}
