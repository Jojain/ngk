//! Exact bounding boxes of topology views, in a caller-chosen frame.
//!
//! Along any direction, an entity reaches its extreme at a vertex, at an edge
//! point where the edge's tangent is perpendicular to that direction, or at a
//! face point where the face's normal is parallel to it. Those candidate points
//! are found in closed form for each axis of the frame and the box is fitted
//! over them: nothing is sampled, so the box is the entity's own, to rounding.
//!
//! - **Lines** reach their extremes at their ends. **Circles, ellipses and
//!   helices** are `o + a cos t + b sin t + k t`, whose height along an axis is
//!   a sinusoid plus a slope with closed-form critical angles.
//! - **Planes, cylinders, cones and extrusions** are developable: where the
//!   normal is parallel to the axis the height is constant along a ruling, and
//!   the ruling runs to the face's boundary, so the edges hold the extremes. A
//!   cone face may also hold its apex, which no edge need reach.
//! - **Spheres and tori** add the points whose normal is plus or minus the
//!   axis, kept unless the trim excludes them. Where those points fill a whole
//!   circle (a torus seen along its axis), one stands for it: the height is
//!   constant on the circle, and any arc of it the face holds either is the
//!   whole circle or ends on the boundary.
//! - **Surfaces of revolution** of a line, circle or ellipse drawn in a plane
//!   through the axis reduce to their profile's extremes along two directions
//!   of that plane, at the two angles where the axis direction meets the
//!   meridian square on.
//!
//! NURBS curves and surfaces, and revolutions of any other profile, are refused
//! with [`MeasureError::EdgeBoundingBoxNotImplemented`] or
//! [`MeasureError::FaceBoundingBoxNotImplemented`] rather than bounded loosely.

use std::f64::consts::{PI, TAU};

use nalgebra::Vector3;

use crate::geometry::parameter::NativeParam;
use crate::geometry::{
    ANGULAR_TOLERANCE, BBox, Curve, Frame, Interval, LINEAR_TOLERANCE, Point2, Point3, Surface,
};
use crate::topology::edge::{Edge, EdgeCore};
use crate::topology::face::Face;
use crate::topology::payload::Payload;
use crate::topology::trim_domain::{FaceTrimDomain, TrimLocation};

use super::integrate::MeasureError;

/// The box of `edge` in `frame`.
pub(crate) fn edge_bounding_box<P: Payload>(
    edge: &EdgeCore<'_, P>,
    frame: &Frame,
) -> Result<BBox, MeasureError> {
    let mut points = Vec::new();
    edge_candidates(edge, frame, &mut points)?;
    fit(frame, points)
}

/// The box of `edges` in `frame`.
pub(crate) fn edges_bounding_box<'a, P: Payload + 'a>(
    edges: impl IntoIterator<Item = Edge<'a, P>>,
    frame: &Frame,
) -> Result<BBox, MeasureError> {
    let mut points = Vec::new();
    for edge in edges {
        edge_candidates(&edge, frame, &mut points)?;
    }
    fit(frame, points)
}

/// The box of `faces` in `frame`, their edges included.
pub(crate) fn faces_bounding_box<'a, P: Payload + 'a>(
    faces: impl IntoIterator<Item = Face<'a, P>>,
    frame: &Frame,
) -> Result<BBox, MeasureError> {
    let mut points = Vec::new();
    for face in faces {
        for edge in face.edges() {
            edge_candidates(&edge, frame, &mut points)?;
        }
        face_interior_candidates(&face, frame, &mut points)?;
    }
    fit(frame, points)
}

fn fit(frame: &Frame, points: Vec<Point3>) -> Result<BBox, MeasureError> {
    BBox::from_points_in_frame(frame.clone(), points).ok_or(MeasureError::Degenerate)
}

fn axes(frame: &Frame) -> [Vector3<f64>; 3] {
    [*frame.x_dir, *frame.y_dir, *frame.z_dir]
}

/// `direction`'s components along `frame`'s axes.
fn components_in(frame: &Frame, direction: Vector3<f64>) -> Vector3<f64> {
    Vector3::new(
        direction.dot(&frame.x_dir),
        direction.dot(&frame.y_dir),
        direction.dot(&frame.z_dir),
    )
}

fn edge_candidates<P: Payload>(
    edge: &EdgeCore<'_, P>,
    frame: &Frame,
    out: &mut Vec<Point3>,
) -> Result<(), MeasureError> {
    let interval = edge.parameter_interval().ordered();
    if !interval.is_finite() {
        return Err(MeasureError::UnboundedEdge { edge: edge.key() });
    }
    let curve = edge.curve();
    for axis in axes(frame) {
        let parameters = critical_parameters(curve, interval, axis).map_err(|curve| {
            MeasureError::EdgeBoundingBoxNotImplemented {
                edge: edge.key(),
                curve,
            }
        })?;
        out.extend(
            parameters
                .into_iter()
                .map(|t| curve.point_at(NativeParam::new(t))),
        );
    }
    Ok(())
}

/// The parameters in `interval` where `direction · curve(t)` can be extreme:
/// both ends and every interior critical point. Errs with the curve's kind
/// when it has no closed form here.
fn critical_parameters(
    curve: &Curve,
    interval: Interval,
    direction: Vector3<f64>,
) -> Result<Vec<f64>, &'static str> {
    let (start, end) = (interval.start.value(), interval.end.value());
    let mut parameters = vec![start, end];
    let Some(sinusoid) = Sinusoid::of(curve)? else {
        return Ok(parameters);
    };
    parameters.extend(sinusoid.critical_angles(direction, start, end));
    Ok(parameters)
}

/// A curve written as `o + a cos t + b sin t + k t`.
struct Sinusoid {
    a: Vector3<f64>,
    b: Vector3<f64>,
    k: Vector3<f64>,
}

impl Sinusoid {
    /// `None` for a line, whose height is linear and peaks at its ends.
    fn of(curve: &Curve) -> Result<Option<Self>, &'static str> {
        let sinusoid = match curve {
            Curve::Line(_) => return Ok(None),
            Curve::Circle(circle) => Self {
                a: *circle.plane().x_dir() * circle.radius(),
                b: *circle.plane().y_dir() * circle.radius(),
                k: Vector3::zeros(),
            },
            Curve::Ellipse(ellipse) => Self {
                a: *ellipse.frame().x_dir * ellipse.major_radius(),
                b: *ellipse.frame().y_dir * ellipse.minor_radius(),
                k: Vector3::zeros(),
            },
            Curve::Helix(helix) => Self {
                a: *helix.frame().x_dir * helix.radius(),
                b: *helix.frame().y_dir * helix.radius(),
                k: *helix.frame().z_dir * (helix.pitch() / TAU),
            },
            Curve::Nurbs(_) => return Err("NURBS curve"),
        };
        Ok(Some(sinusoid))
    }

    /// The angles in `[start, end]` where the height along `direction` has a
    /// zero derivative: `B cos t - A sin t + K = 0`, that is
    /// `R cos(t + α) = -K` with `R cos α = B` and `R sin α = A`.
    fn critical_angles(&self, direction: Vector3<f64>, start: f64, end: f64) -> Vec<f64> {
        let (a, b, k) = (
            direction.dot(&self.a),
            direction.dot(&self.b),
            direction.dot(&self.k),
        );
        let r = a.hypot(b);
        if r == 0.0 || k.abs() > r {
            return Vec::new();
        }
        let alpha = a.atan2(b);
        let spread = (-k / r).clamp(-1.0, 1.0).acos();
        let mut angles = Vec::new();
        for base in [spread - alpha, -spread - alpha] {
            let first = ((start - base) / TAU).ceil() as i64;
            let last = ((end - base) / TAU).floor() as i64;
            angles.extend((first..=last).map(|turn| base + turn as f64 * TAU));
        }
        angles
    }
}

/// Points inside `face`, away from its edges, where its normal is parallel to
/// one of the frame's axes and no ruling carries the height to the boundary.
fn face_interior_candidates<P: Payload>(
    face: &Face<'_, P>,
    frame: &Frame,
    out: &mut Vec<Point3>,
) -> Result<(), MeasureError> {
    let surface = face.surface();
    if matches!(
        surface,
        Surface::Plane(_) | Surface::Cylinder(_) | Surface::Ruled(_)
    ) {
        return Ok(());
    }
    if let Surface::Nurbs(_) = surface {
        return Err(MeasureError::FaceBoundingBoxNotImplemented {
            face: face.key(),
            surface: "NURBS surface",
        });
    }
    let trim = FaceTrimDomain::new(face, LINEAR_TOLERANCE).map_err(|source| {
        MeasureError::UnreadableBoundary {
            face: face.key(),
            source,
        }
    })?;
    // A row the surface collapses to a point, or a circle of equal height,
    // is represented at this face's own `u` so that its trim can place it.
    let (low, high) = trim.domain_bounds();
    let u_center = (low.x + high.x) / 2.0;
    let mut candidates: Vec<Point2> = Vec::new();
    match surface {
        Surface::Cone(cone) => {
            if let Some(apex) = cone.apex_parameter() {
                candidates.push(Point2::new(u_center, apex));
            }
        }
        Surface::Sphere(sphere) => {
            for axis in axes(frame) {
                let local = components_in(sphere.frame(), axis);
                for sign in [1.0, -1.0] {
                    let normal = local * sign;
                    let u = azimuth(normal, u_center);
                    candidates.push(Point2::new(u, normal.z.clamp(-1.0, 1.0).asin()));
                }
            }
        }
        Surface::Torus(torus) => {
            for axis in axes(frame) {
                let local = components_in(torus.frame(), axis);
                let radial = local.x.hypot(local.y);
                let u = azimuth(local, u_center);
                let turned = u + PI;
                for (u, cos_v, sin_v) in [
                    (u, radial, local.z),
                    (turned, -radial, local.z),
                    (u, -radial, -local.z),
                    (turned, radial, -local.z),
                ] {
                    candidates.push(Point2::new(u, sin_v.atan2(cos_v)));
                }
            }
        }
        Surface::Revolution(revolution) => {
            let profile = revolution.curve();
            let axis = revolution.axis;
            let u_range = Interval::new(low.x, high.x);
            let radial = meridian_direction(profile, &axis, u_range).ok_or(
                MeasureError::FaceBoundingBoxNotImplemented {
                    face: face.key(),
                    surface: "surface of revolution whose profile is not a line, circle or \
                              ellipse in a plane through its axis",
                },
            )?;
            let tangential = axis.direction.cross(&radial);
            let v_center = (low.y + high.y) / 2.0;
            for direction in axes(frame) {
                let along = direction.dot(&axis.direction);
                let (p, q) = (direction.dot(&radial), direction.dot(&tangential));
                let reach = p.hypot(q);
                // At angle `v` the meridian's radial direction is
                // `cos v radial + sin v tangential`; the height peaks at
                // `atan2(q, p)` and bottoms out half a turn on.
                let peak = if reach == 0.0 { v_center } else { q.atan2(p) };
                for (v, sign) in [(peak, 1.0), (peak + PI, -1.0)] {
                    let in_meridian = *axis.direction * along + radial * (sign * reach);
                    let parameters = critical_parameters(profile, u_range, in_meridian)
                        .expect("a meridian profile is a line, circle or ellipse");
                    candidates.extend(parameters.into_iter().map(|u| Point2::new(u, v)));
                }
            }
        }
        Surface::Plane(_) | Surface::Cylinder(_) | Surface::Ruled(_) | Surface::Nurbs(_) => {
            unreachable!("handled above")
        }
    }
    out.extend(
        candidates
            .into_iter()
            .filter(|uv| !matches!(trim.classify(*uv), TrimLocation::Outside { .. }))
            .map(|uv| surface.point_at(uv.x, uv.y)),
    );
    Ok(())
}

/// The azimuth of `local` about its frame's `z` axis, or `fallback` on the
/// axis itself, where every azimuth names the same point.
fn azimuth(local: Vector3<f64>, fallback: f64) -> f64 {
    if local.x == 0.0 && local.y == 0.0 {
        fallback
    } else {
        local.y.atan2(local.x)
    }
}

/// The unit radial direction of the meridian half-plane that holds `profile`,
/// when `profile` is a line, circle or ellipse lying in a plane through `axis`.
fn meridian_direction(
    profile: &Curve,
    axis: &crate::geometry::Axis3,
    u_range: Interval,
) -> Option<Vector3<f64>> {
    let samples = [0.0, 0.5, 1.0].map(|fraction| {
        let t = u_range.start.value() + fraction * (u_range.end.value() - u_range.start.value());
        let point = profile.point_at(NativeParam::new(t));
        point - axis.project(point)
    });
    let radial = samples
        .into_iter()
        .max_by(|a, b| a.norm().total_cmp(&b.norm()))?
        .try_normalize(LINEAR_TOLERANCE)?;
    let normal = axis.direction.cross(&radial);
    let in_plane = |point: Point3| (point - axis.origin).dot(&normal).abs() <= LINEAR_TOLERANCE;
    let parallel_to_plane =
        |direction: Vector3<f64>| direction.dot(&normal).abs() <= ANGULAR_TOLERANCE;
    let square_to_plane =
        |direction: Vector3<f64>| direction.cross(&normal).norm() <= ANGULAR_TOLERANCE;
    let planar = match profile {
        Curve::Line(line) => in_plane(line.axis.origin) && parallel_to_plane(*line.axis.direction),
        Curve::Circle(circle) => {
            in_plane(circle.plane().origin()) && square_to_plane(*circle.plane().normal())
        }
        Curve::Ellipse(ellipse) => {
            in_plane(ellipse.frame().origin) && square_to_plane(*ellipse.frame().z_dir)
        }
        Curve::Helix(_) | Curve::Nurbs(_) => false,
    };
    planar.then_some(radial)
}
