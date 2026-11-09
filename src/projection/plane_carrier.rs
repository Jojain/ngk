//! The affine map that carries 3D points onto a plane.
//!
//! A perpendicular and a directional projection onto a plane are both
//! parallel maps: every point moves along one fixed direction until it meets
//! the plane. An affine map takes a line to a line, a conic to a conic and a
//! NURBS curve to the NURBS curve on the mapped control points with the same
//! weights, so every image here is exact.

use nalgebra::Vector3;

use super::error::ProjectionError;
use super::mode::Projection;
use super::options::ProjectOptions;
use crate::geometry::{
    ANGULAR_TOLERANCE, Circle, Circle2, ControlPolygon, ControlPolygon2, Curve, Curve2,
    CurveSurfaceIntersection, Ellipse, Ellipse2, Frame, HPoint, HPoint2, IntersectionCoverage,
    Interval, Line, Line2, NativeParam, NurbsCurve, NurbsCurve2, Periodicity, Plane, Point2,
    Point3, Surface, TrimmedCurve, TrimmedCurve2, Vector2,
};

/// How close two parameters are before they are one: the width below which a
/// piece of a span is not a piece.
pub(super) const PARAMETER_EPSILON: f64 = 1.0e-9;

/// A part of a span, and whether the plane casts a shadow of it.
#[derive(Debug, Clone, Copy)]
pub(super) struct Piece {
    pub(super) interval: Interval,
    /// Whether the part lies in front of the plane along the direction of
    /// projection, so that its shadow exists.
    pub(super) front: bool,
}

/// A plane together with the direction its points are carried along.
pub(super) struct PlaneCarrier<'a> {
    plane: &'a Plane,
    /// The direction points move along: the normal for a perpendicular
    /// projection, the given direction otherwise.
    carry: Vector3<f64>,
    /// The component of `carry` along the normal, which is how far a point
    /// moves per unit of its height above the plane.
    slope: f64,
    /// Whether a half-line, rather than a line, carries each point.
    directional: bool,
    tolerance: f64,
}

impl<'a> PlaneCarrier<'a> {
    /// The carrier of `mode` onto `plane`.
    ///
    /// A direction parallel to the plane carries nothing onto it: the result
    /// is [`RayInSurface`](ProjectionError::RayInSurface) when `probe`, a
    /// point of the source, lies in the plane, and a
    /// [`Miss`](ProjectionError::Miss) otherwise.
    pub(super) fn new(
        plane: &'a Plane,
        mode: Projection,
        options: &ProjectOptions,
        probe: Point3,
    ) -> Result<Self, ProjectionError> {
        let normal = *plane.normal();
        let (carry, directional) = match mode {
            Projection::Normal => (normal, false),
            Projection::Along(direction) => (*direction, true),
        };
        let slope = carry.dot(&normal);
        if slope.abs() <= ANGULAR_TOLERANCE {
            let offset = (probe - plane.origin()).dot(&normal);
            return Err(if offset.abs() <= options.tolerance {
                ProjectionError::RayInSurface
            } else {
                ProjectionError::Miss
            });
        }
        Ok(Self {
            plane,
            carry,
            slope,
            directional,
            tolerance: options.tolerance,
        })
    }

    /// The `(u, v)` of the image of `point`.
    fn image(&self, point: Point3) -> Point2 {
        let offset = point - self.plane.origin();
        self.coordinates(offset - self.carry * (offset.dot(&self.plane.normal()) / self.slope))
    }

    /// The image of the direction `vector`, in the plane's `(u, v)`.
    fn linear_image(&self, vector: Vector3<f64>) -> Vector2 {
        let moved = vector - self.carry * (vector.dot(&self.plane.normal()) / self.slope);
        self.coordinates(moved).coords
    }

    fn coordinates(&self, in_plane: Vector3<f64>) -> Point2 {
        Point2::new(
            in_plane.dot(&self.plane.x_dir()),
            in_plane.dot(&self.plane.y_dir()),
        )
    }

    fn lift(&self, point: Point2) -> Point3 {
        self.plane.point_at(point.x, point.y)
    }

    fn lift_direction(&self, vector: Vector2) -> Vector3<f64> {
        *self.plane.x_dir() * vector.x + *self.plane.y_dir() * vector.y
    }

    /// How far `point` must travel along the direction to reach the plane;
    /// negative when the plane is behind it.
    fn lead(&self, point: Point3) -> f64 {
        -(point - self.plane.origin()).dot(&self.plane.normal()) / self.slope
    }

    /// The parts of `span` that do and do not cast a shadow, in the order the
    /// span runs, with neighbours of the same kind merged.
    ///
    /// A perpendicular projection has no behind, so it is one part. A
    /// directional one is cut where the span crosses the plane, and a part is
    /// in front when the plane lies ahead of its midpoint.
    pub(super) fn pieces(&self, span: &TrimmedCurve) -> Result<Vec<Piece>, ProjectionError> {
        let interval = span.interval();
        if !self.directional {
            return Ok(vec![Piece {
                interval,
                front: true,
            }]);
        }
        let mut bounds = vec![interval.start.value()];
        bounds.extend(self.crossings(span)?);
        bounds.push(interval.end.value());
        let mut pieces: Vec<Piece> = Vec::new();
        for pair in bounds.windows(2) {
            let part = Interval::new(pair[0], pair[1]);
            if part.length() <= PARAMETER_EPSILON {
                continue;
            }
            let front = self.lead(span.curve().point_at(part.midpoint())) >= -self.tolerance;
            match pieces.last_mut() {
                Some(last) if last.front == front => {
                    last.interval = Interval::new(last.interval.start, part.end);
                }
                _ => pieces.push(Piece {
                    interval: part,
                    front,
                }),
            }
        }
        if pieces.is_empty() {
            return Err(ProjectionError::DegenerateImage);
        }
        Ok(pieces)
    }

    /// The parameters strictly inside `span` where its support meets the
    /// plane, in the order the span runs.
    fn crossings(&self, span: &TrimmedCurve) -> Result<Vec<f64>, ProjectionError> {
        crossings_with_plane(span, self.plane)
    }

    /// The image of the part of `curve` over `interval`: the piece on the
    /// plane in 3D, and the same piece in the plane's `(u, v)`.
    ///
    /// A line keeps the parameter it had, a conic's angle is turned by the
    /// phase of its image's own axes, and a NURBS curve is cut to the part and
    /// then mapped, so the part's own knots are its parameter.
    pub(super) fn project_piece(
        &self,
        curve: &Curve,
        interval: Interval,
    ) -> Result<(TrimmedCurve, TrimmedCurve2), ProjectionError> {
        match curve {
            Curve::Line(line) => self.line_image(line, interval),
            Curve::Circle(circle) => {
                let radius = circle.radius();
                self.conic_image(&circle.plane().frame, radius, radius, interval)
            }
            Curve::Ellipse(ellipse) => self.conic_image(
                ellipse.frame(),
                ellipse.major_radius(),
                ellipse.minor_radius(),
                interval,
            ),
            Curve::Nurbs(nurbs) => self.nurbs_image(nurbs, interval),
            Curve::Helix(_) => Err(ProjectionError::UnsupportedCurve { curve: "helix" }),
        }
    }

    fn line_image(
        &self,
        line: &Line,
        interval: Interval,
    ) -> Result<(TrimmedCurve, TrimmedCurve2), ProjectionError> {
        let first = self.image(line.point_at(NativeParam::new(0.0)));
        let second = self.image(line.point_at(NativeParam::new(1.0)));
        if (second - first).norm() * interval.length() <= self.tolerance {
            return Err(ProjectionError::DegenerateImage);
        }
        let in_3d = Curve::Line(Line::through(self.lift(first), self.lift(second)));
        let in_2d = Curve2::Line(Line2::new(first, second - first));
        Ok((
            TrimmedCurve::new(in_3d, interval),
            TrimmedCurve2::new(in_2d, interval),
        ))
    }

    /// The image of the conic `frame.origin + major cos t x + minor sin t y`.
    ///
    /// Its image is `c + cos t p + sin t q` for the images `p`, `q` of the two
    /// semi-axes, which is an ellipse whose own axes are the conjugate
    /// diameters at the phase where `p` and `q` come out perpendicular. The
    /// image is a circle when those two are equally long, and runs clockwise
    /// in the plane when the map reverses its sense.
    fn conic_image(
        &self,
        frame: &Frame,
        major: f64,
        minor: f64,
        interval: Interval,
    ) -> Result<(TrimmedCurve, TrimmedCurve2), ProjectionError> {
        let center = self.image(frame.origin);
        let p = self.linear_image(*frame.x_dir * major);
        let q = self.linear_image(*frame.y_dir * minor);
        let (pp, qq, pq) = (p.norm_squared(), q.norm_squared(), p.dot(&q));
        let scale = pp + qq;
        // Axes that are already perpendicular and equal are a circle, whose
        // phase is arbitrary and kept at zero.
        let phase = if (pp - qq).abs() <= 1.0e-12 * scale && pq.abs() <= 1.0e-12 * scale {
            0.0
        } else {
            0.5 * (2.0 * pq).atan2(pp - qq)
        };
        let (sin, cos) = phase.sin_cos();
        let axis_major = p * cos + q * sin;
        let axis_minor = q * cos - p * sin;
        let (image_major, image_minor) = (axis_major.norm(), axis_minor.norm());
        if image_minor <= self.tolerance {
            return Err(ProjectionError::DegenerateImage);
        }
        let x_dir = axis_major / image_major;
        let counter_clockwise = axis_major.perp(&axis_minor) > 0.0;
        let sense = if counter_clockwise { 1.0 } else { -1.0 };
        let y_dir = Vector2::new(-x_dir.y, x_dir.x) * sense;
        let swept = Interval::new(interval.start.value() - phase, interval.end.value() - phase);
        let origin = self.lift(center);
        let (in_3d, in_2d) = if (image_major - image_minor).abs() <= self.tolerance {
            let plane = Plane::new(
                origin,
                self.lift_direction(x_dir),
                *self.plane.normal() * sense,
            );
            let flat = Circle2::new(center, x_dir, image_major);
            (
                Curve::Circle(Circle::new(plane, image_major)),
                Curve2::Circle(if counter_clockwise {
                    flat
                } else {
                    flat.reversed()
                }),
            )
        } else {
            let axes = Frame::from_xy(
                origin,
                self.lift_direction(x_dir),
                self.lift_direction(y_dir),
            );
            let flat = Ellipse2::new(center, x_dir, image_major, image_minor);
            (
                Curve::Ellipse(Ellipse::new(axes, image_major, image_minor)),
                Curve2::Ellipse(if counter_clockwise {
                    flat
                } else {
                    flat.reversed()
                }),
            )
        };
        Ok((
            TrimmedCurve::new(in_3d, swept),
            TrimmedCurve2::new(in_2d, swept),
        ))
    }

    fn nurbs_image(
        &self,
        nurbs: &NurbsCurve,
        interval: Interval,
    ) -> Result<(TrimmedCurve, TrimmedCurve2), ProjectionError> {
        let part = nurbs.trimmed(interval.start.value(), interval.end.value())?;
        let mapped: Vec<(Point2, f64)> = part
            .control_points()
            .iter()
            .map(|point| (self.image(point.to_cartesian()), point.weight()))
            .collect();
        let anchor = mapped[0].0;
        if mapped
            .iter()
            .all(|(point, _)| (point - anchor).norm() <= self.tolerance)
        {
            return Err(ProjectionError::DegenerateImage);
        }
        let in_3d = NurbsCurve::new(
            part.degree(),
            ControlPolygon::new(
                mapped
                    .iter()
                    .map(|(point, weight)| HPoint::from_cartesian(self.lift(*point), *weight))
                    .collect(),
            )?,
            part.knots().clone(),
        )?;
        let in_2d = NurbsCurve2::new(
            part.degree(),
            ControlPolygon2::new(
                mapped
                    .iter()
                    .map(|(point, weight)| HPoint2::from_cartesian(*point, *weight))
                    .collect(),
            )?,
            part.knots().clone(),
        )?;
        let domain = part.domain();
        Ok((
            TrimmedCurve::new(Curve::Nurbs(in_3d), domain),
            TrimmedCurve2::new(Curve2::Nurbs(in_2d), domain),
        ))
    }
}

/// The parameters strictly inside `span` where its support meets the
/// plane, in the order the span runs.
pub(super) fn crossings_with_plane(
    span: &TrimmedCurve,
    plane: &Plane,
) -> Result<Vec<f64>, ProjectionError> {
    let found = span
        .curve()
        .intersect_surface(&Surface::Plane(plane.clone()))?;
    if !matches!(found.coverage(), IntersectionCoverage::Complete) {
        return Err(ProjectionError::IncompleteCrossings);
    }
    let mut candidates = Vec::new();
    for hit in found.intersections() {
        match hit {
            CurveSurfaceIntersection::Point { curve_u, .. } => candidates.push(*curve_u),
            CurveSurfaceIntersection::Overlap { curve_interval } => {
                candidates.push(curve_interval.start.value());
                candidates.push(curve_interval.end.value());
            }
        }
    }
    let ordered = span.interval().ordered();
    let (low, high) = (ordered.start.value(), ordered.end.value());
    let mut roots = Vec::new();
    for candidate in candidates.into_iter().filter(|c| c.is_finite()) {
        match span.curve().periodicity() {
            Periodicity::Periodic(period) => {
                let mut root = candidate + ((low - candidate) / period).ceil() * period;
                while root <= high {
                    roots.push(root);
                    root += period;
                }
            }
            Periodicity::None => roots.push(candidate),
        }
    }
    roots.retain(|root| *root > low + PARAMETER_EPSILON && *root < high - PARAMETER_EPSILON);
    roots.sort_by(f64::total_cmp);
    roots.dedup_by(|a, b| (*a - *b).abs() <= PARAMETER_EPSILON);
    if span.interval().delta() < 0.0 {
        roots.reverse();
    }
    Ok(roots)
}
