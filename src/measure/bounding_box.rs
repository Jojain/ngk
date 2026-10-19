//! Exact bounding boxes of topology views.
//!
//! Along any direction, an entity reaches its extreme at a vertex, at an edge
//! point where the edge's tangent is perpendicular to that direction, or at a
//! face point where the face's normal is parallel to it. Those candidate points
//! are found in closed form for each axis of a frame and the box is fitted over
//! them: nothing is sampled, so the box is the entity's own, to rounding.
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
//!
//! # Oriented box
//!
//! The oriented box is the smallest of the exact boxes in a list of candidate
//! frames, so its extents are as exact as any other box; only its orientation
//! is chosen, and it is the best of the candidates, not a proven minimum. The
//! candidates pair every direction the geometry names -- a plane's normal, a
//! straight edge, the axis of a circle, cylinder, cone, torus, helix or
//! revolution, an extrusion's direction, the principal axes of inertia and the
//! world axes -- with each other one, made square to it. A prismatic or turned
//! part has its smallest box along such a pair. Nothing is random and the order
//! is fixed, so one input always gives one box; where several orientations tie,
//! as on a sphere, the first candidate keeps it.

use std::f64::consts::{PI, TAU};

use nalgebra::{Matrix3, Vector3};

use crate::geometry::parameter::NativeParam;
use crate::geometry::{
    ANGULAR_TOLERANCE, BBox, Curve, Frame, Interval, LINEAR_TOLERANCE, Point2, Point3, Surface,
};
use crate::topology::edge::{Edge, EdgeCore};
use crate::topology::face::Face;
use crate::topology::payload::Payload;
use crate::topology::trim_domain::{FaceTrimDomain, TrimLocation};

use super::integrate::MeasureError;

/// What a set of edges and faces needs to place its extremes along any frame,
/// read and checked once so that many frames cost one setup.
pub(crate) struct Extremes<'a> {
    edges: Vec<EdgeExtremes<'a>>,
    faces: Vec<FaceExtremes>,
}

impl<'a> Extremes<'a> {
    pub(crate) fn of_edge<P: Payload>(edge: &EdgeCore<'a, P>) -> Result<Self, MeasureError> {
        Ok(Self {
            edges: vec![EdgeExtremes::of(edge)?],
            faces: Vec::new(),
        })
    }

    pub(crate) fn of_edges<P: Payload + 'a>(
        edges: impl IntoIterator<Item = Edge<'a, P>>,
    ) -> Result<Self, MeasureError> {
        Ok(Self {
            edges: edges
                .into_iter()
                .map(|edge| EdgeExtremes::of(&edge))
                .collect::<Result<_, _>>()?,
            faces: Vec::new(),
        })
    }

    /// The faces and every edge they run along.
    pub(crate) fn of_faces<P: Payload + 'a>(
        faces: impl IntoIterator<Item = Face<'a, P>>,
    ) -> Result<Self, MeasureError> {
        let mut extremes = Self {
            edges: Vec::new(),
            faces: Vec::new(),
        };
        for face in faces {
            for edge in face.edges() {
                extremes.edges.push(EdgeExtremes::of(&edge)?);
            }
            extremes.faces.push(FaceExtremes::of(&face)?);
        }
        Ok(extremes)
    }

    /// The tightest box whose ranges run along `frame`'s axes.
    pub(crate) fn box_in(&self, frame: &Frame) -> Result<BBox, MeasureError> {
        let mut points = Vec::new();
        for axis in axes(frame) {
            for edge in &self.edges {
                edge.extend_candidates(axis, &mut points);
            }
            for face in &self.faces {
                face.extend_candidates(axis, &mut points);
            }
        }
        BBox::from_points_in_frame(frame.clone(), points).ok_or(MeasureError::Degenerate)
    }

    /// The smallest box among the candidate frames, in a canonical frame:
    /// centred on the box, axes from the longest extent to the shortest.
    ///
    /// `principal` holds the entity's principal axes of inertia as columns.
    pub(crate) fn oriented_box(&self, principal: Matrix3<f64>) -> Result<BBox, MeasureError> {
        let mut directions = Directions::default();
        for axis in axes(&Frame::xyz()) {
            directions.add(axis);
        }
        for column in principal.column_iter() {
            directions.add(column.into_owned());
        }
        for edge in &self.edges {
            edge.feature_directions(&mut directions);
        }
        for face in &self.faces {
            face.feature_directions(&mut directions);
        }

        let world = self.box_in(&Frame::xyz())?;
        let scale = world.diagonal_length().max(LINEAR_TOLERANCE);
        let mut best = (Frame::xyz(), Size::of(&world, scale));
        let mut tried: Vec<Frame> = vec![Frame::xyz()];
        for normal in &directions.0 {
            for other in &directions.0 {
                let Some(frame) = frame_from(*normal, *other) else {
                    continue;
                };
                if tried.iter().any(|seen| same_axes(seen, &frame)) {
                    continue;
                }
                let size = Size::of(&self.box_in(&frame)?, scale);
                if size.smaller_than(&best.1) {
                    best = (frame.clone(), size);
                }
                tried.push(frame);
            }
        }

        let canonical = canonical_frame(&self.box_in(&best.0)?);
        self.box_in(&canonical)
    }
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

/// An edge's curve, span and, unless it is straight, its sinusoid.
struct EdgeExtremes<'a> {
    curve: &'a Curve,
    interval: Interval,
    sinusoid: Option<Sinusoid>,
}

impl<'a> EdgeExtremes<'a> {
    fn of<P: Payload>(edge: &EdgeCore<'a, P>) -> Result<Self, MeasureError> {
        let interval = edge.parameter_interval().ordered();
        if !interval.is_finite() {
            return Err(MeasureError::UnboundedEdge { edge: edge.key() });
        }
        let curve = edge.curve();
        let sinusoid =
            Sinusoid::of(curve).map_err(|curve| MeasureError::EdgeBoundingBoxNotImplemented {
                edge: edge.key(),
                curve,
            })?;
        Ok(Self {
            curve,
            interval,
            sinusoid,
        })
    }

    fn extend_candidates(&self, direction: Vector3<f64>, out: &mut Vec<Point3>) {
        out.extend(
            critical_parameters(self.sinusoid.as_ref(), self.interval, direction)
                .map(|t| self.curve.point_at(NativeParam::new(t))),
        );
    }

    fn feature_directions(&self, directions: &mut Directions) {
        match self.curve {
            Curve::Line(line) => directions.add(*line.axis.direction),
            Curve::Circle(circle) => directions.add(*circle.plane().normal()),
            Curve::Ellipse(ellipse) => {
                directions.add(*ellipse.frame().z_dir);
                directions.add(*ellipse.frame().x_dir);
            }
            Curve::Helix(helix) => directions.add(*helix.frame().z_dir),
            Curve::Nurbs(_) => {}
        }
    }
}

/// The parameters in `interval` where `direction · curve(t)` can be extreme:
/// both ends and, for a curved one, every interior critical point.
fn critical_parameters(
    sinusoid: Option<&Sinusoid>,
    interval: Interval,
    direction: Vector3<f64>,
) -> impl Iterator<Item = f64> {
    let (start, end) = (interval.start.value(), interval.end.value());
    let interior = sinusoid
        .map(|sinusoid| sinusoid.critical_angles(direction, start, end))
        .unwrap_or_default();
    [start, end].into_iter().chain(interior)
}

/// A curve written as `o + a cos t + b sin t + k t`.
struct Sinusoid {
    a: Vector3<f64>,
    b: Vector3<f64>,
    k: Vector3<f64>,
}

impl Sinusoid {
    /// `None` for a line, whose height is linear and peaks at its ends; the
    /// curve's kind when it has no closed form here.
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

/// A face whose interior can hold an extreme no edge reaches, with what
/// placing those points needs; developable faces have none.
enum FaceExtremes {
    Developable(Surface),
    Curved {
        surface: Surface,
        trim: FaceTrimDomain,
        /// Corners of the face's parameter extent.
        low: Point2,
        high: Point2,
        meridian: Option<Meridian>,
    },
}

/// The meridian plane a revolution's profile is drawn in.
struct Meridian {
    radial: Vector3<f64>,
    tangential: Vector3<f64>,
    profile: Option<Sinusoid>,
}

impl FaceExtremes {
    fn of<P: Payload>(face: &Face<'_, P>) -> Result<Self, MeasureError> {
        let surface = face.surface().clone();
        match &surface {
            Surface::Plane(_) | Surface::Cylinder(_) | Surface::Ruled(_) => {
                return Ok(Self::Developable(surface));
            }
            Surface::Nurbs(_) => {
                return Err(MeasureError::FaceBoundingBoxNotImplemented {
                    face: face.key(),
                    surface: "NURBS surface",
                });
            }
            Surface::Cone(_) | Surface::Sphere(_) | Surface::Torus(_) | Surface::Revolution(_) => {}
        }
        let trim = FaceTrimDomain::new(face, LINEAR_TOLERANCE).map_err(|source| {
            MeasureError::UnreadableBoundary {
                face: face.key(),
                source,
            }
        })?;
        let (low, high) = trim.domain_bounds();
        let meridian = match &surface {
            Surface::Revolution(revolution) => Some(
                Meridian::of(revolution.curve(), &revolution.axis, low.x, high.x).ok_or(
                    MeasureError::FaceBoundingBoxNotImplemented {
                        face: face.key(),
                        surface: "surface of revolution whose profile is not a line, circle \
                                  or ellipse in a plane through its axis",
                    },
                )?,
            ),
            _ => None,
        };
        Ok(Self::Curved {
            surface,
            trim,
            low,
            high,
            meridian,
        })
    }

    fn extend_candidates(&self, direction: Vector3<f64>, out: &mut Vec<Point3>) {
        let Self::Curved {
            surface,
            trim,
            low,
            high,
            meridian,
        } = self
        else {
            return;
        };
        // A row the surface collapses to a point, or a circle of equal height,
        // is represented at this face's own `u` so that its trim can place it.
        let u_center = (low.x + high.x) / 2.0;
        let mut candidates: Vec<Point2> = Vec::new();
        match surface {
            Surface::Cone(cone) => {
                if let Some(apex) = cone.apex_parameter() {
                    candidates.push(Point2::new(u_center, apex));
                }
            }
            Surface::Sphere(sphere) => {
                let local = components_in(sphere.frame(), direction);
                for normal in [local, -local] {
                    let u = azimuth(normal, u_center);
                    candidates.push(Point2::new(u, normal.z.clamp(-1.0, 1.0).asin()));
                }
            }
            Surface::Torus(torus) => {
                let local = components_in(torus.frame(), direction);
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
            Surface::Revolution(revolution) => {
                let meridian = meridian.as_ref().expect("a revolution reads its meridian");
                let axis = *revolution.axis.direction;
                let along = direction.dot(&axis);
                let (p, q) = (
                    direction.dot(&meridian.radial),
                    direction.dot(&meridian.tangential),
                );
                let reach = p.hypot(q);
                // At angle `v` the meridian's radial direction is
                // `cos v radial + sin v tangential`; the height peaks at
                // `atan2(q, p)` and bottoms out half a turn on.
                let peak = if reach == 0.0 {
                    (low.y + high.y) / 2.0
                } else {
                    q.atan2(p)
                };
                let u_range = Interval::new(low.x, high.x);
                for (v, sign) in [(peak, 1.0), (peak + PI, -1.0)] {
                    let in_meridian = axis * along + meridian.radial * (sign * reach);
                    candidates.extend(
                        critical_parameters(meridian.profile.as_ref(), u_range, in_meridian)
                            .map(|u| Point2::new(u, v)),
                    );
                }
            }
            Surface::Plane(_) | Surface::Cylinder(_) | Surface::Ruled(_) | Surface::Nurbs(_) => {
                unreachable!("developable and NURBS faces have no curved extremes")
            }
        }
        out.extend(
            candidates
                .into_iter()
                .filter(|uv| !matches!(trim.classify(*uv), TrimLocation::Outside { .. }))
                .map(|uv| surface.point_at(uv.x, uv.y)),
        );
    }

    fn feature_directions(&self, directions: &mut Directions) {
        let surface = match self {
            Self::Developable(surface) | Self::Curved { surface, .. } => surface,
        };
        match surface {
            Surface::Plane(plane) => directions.add(*plane.frame.z_dir),
            Surface::Cylinder(cylinder) => directions.add(*cylinder.frame.z_dir),
            Surface::Cone(cone) => directions.add(*cone.frame().z_dir),
            Surface::Torus(torus) => directions.add(*torus.frame().z_dir),
            Surface::Ruled(ruled) => directions.add(ruled.direction()),
            Surface::Revolution(revolution) => directions.add(*revolution.axis.direction),
            Surface::Sphere(_) | Surface::Nurbs(_) => {}
        }
    }
}

impl Meridian {
    /// The meridian plane of `profile` over `[start, end]`, when `profile` is a
    /// line, circle or ellipse lying in a plane through `axis`.
    fn of(profile: &Curve, axis: &crate::geometry::Axis3, start: f64, end: f64) -> Option<Self> {
        let samples = [0.0, 0.5, 1.0].map(|fraction| {
            let point = profile.point_at(NativeParam::new(start + fraction * (end - start)));
            point - axis.project(point)
        });
        let radial = samples
            .into_iter()
            .max_by(|a, b| a.norm().total_cmp(&b.norm()))?
            .try_normalize(LINEAR_TOLERANCE)?;
        let tangential = axis.direction.cross(&radial);
        let in_plane =
            |point: Point3| (point - axis.origin).dot(&tangential).abs() <= LINEAR_TOLERANCE;
        let parallel_to_plane =
            |direction: Vector3<f64>| direction.dot(&tangential).abs() <= ANGULAR_TOLERANCE;
        let square_to_plane =
            |direction: Vector3<f64>| direction.cross(&tangential).norm() <= ANGULAR_TOLERANCE;
        let planar = match profile {
            Curve::Line(line) => {
                in_plane(line.axis.origin) && parallel_to_plane(*line.axis.direction)
            }
            Curve::Circle(circle) => {
                in_plane(circle.plane().origin()) && square_to_plane(*circle.plane().normal())
            }
            Curve::Ellipse(ellipse) => {
                in_plane(ellipse.frame().origin) && square_to_plane(*ellipse.frame().z_dir)
            }
            Curve::Helix(_) | Curve::Nurbs(_) => false,
        };
        planar.then(|| Self {
            radial,
            tangential,
            profile: Sinusoid::of(profile).ok().flatten(),
        })
    }
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

/// Unit directions in first-seen order, each line through the origin once.
#[derive(Default)]
struct Directions(Vec<Vector3<f64>>);

impl Directions {
    fn add(&mut self, direction: Vector3<f64>) {
        let Some(unit) = direction.try_normalize(LINEAR_TOLERANCE) else {
            return;
        };
        if !self.0.iter().any(|seen| parallel(seen, &unit)) {
            self.0.push(signed(unit));
        }
    }
}

fn parallel(a: &Vector3<f64>, b: &Vector3<f64>) -> bool {
    a.cross(b).norm() <= ANGULAR_TOLERANCE
}

/// `direction` or its opposite, whichever has its largest component positive.
fn signed(direction: Vector3<f64>) -> Vector3<f64> {
    if direction[direction.iamax()] < 0.0 {
        -direction
    } else {
        direction
    }
}

/// The frame with `normal` as its `z` axis and `other`, made square to it, as
/// its `x` axis; `None` when the two are parallel.
fn frame_from(normal: Vector3<f64>, other: Vector3<f64>) -> Option<Frame> {
    let x = (other - normal * other.dot(&normal)).try_normalize(LINEAR_TOLERANCE)?;
    Some(Frame::from_xz(Point3::origin(), x, normal))
}

/// Whether two frames run along the same three lines, in any order and sense.
fn same_axes(a: &Frame, b: &Frame) -> bool {
    axes(a)
        .iter()
        .all(|axis| axes(b).iter().any(|other| parallel(axis, other)))
}

/// What makes one box smaller than another: its volume, then the area of its
/// faces, then the length of its edges, each compared at a resolution scaled
/// by the shape's size so that rounding cannot reorder two equal boxes. The
/// later measures decide between flat or straight boxes, whose volume is zero
/// in every frame that lies along them.
struct Size([f64; 3]);

impl Size {
    fn of(bbox: &BBox, scale: f64) -> Self {
        let [x, y, z] = [bbox.x_size(), bbox.y_size(), bbox.z_size()];
        let resolution = 1e-12;
        Self([
            x * y * z / (resolution * scale.powi(3)),
            (x * y + y * z + z * x) / (resolution * scale.powi(2)),
            (x + y + z) / (resolution * scale),
        ])
    }

    fn smaller_than(&self, other: &Self) -> bool {
        for (mine, theirs) in self.0.iter().zip(other.0) {
            if *mine < theirs - 1.0 {
                return true;
            }
            if *mine > theirs + 1.0 {
                return false;
            }
        }
        false
    }
}

/// `bbox`'s frame recentred on the box, its axes ordered from the longest
/// extent to the shortest, each pointing along its largest positive world
/// component, right-handed.
fn canonical_frame(bbox: &BBox) -> Frame {
    let frame = bbox.frame();
    let size = bbox.size();
    let resolution = 1e-12 * bbox.diagonal_length();
    let mut order = [0, 1, 2];
    // A stable insertion sort that only swaps extents that differ by more than
    // rounding, so a cube keeps the candidate's own order.
    for i in 1..3 {
        let mut j = i;
        while j > 0 && size[order[j]] > size[order[j - 1]] + resolution {
            order.swap(j, j - 1);
            j -= 1;
        }
    }
    let frame_axes = axes(frame);
    let x = signed(frame_axes[order[0]]);
    let y = signed(frame_axes[order[1]]);
    Frame::from_xy(bbox.center(), x, y)
}

/// The box of `edge` in `frame`.
pub(crate) fn edge_bounding_box<P: Payload>(
    edge: &EdgeCore<'_, P>,
    frame: &Frame,
) -> Result<BBox, MeasureError> {
    Extremes::of_edge(edge)?.box_in(frame)
}

/// The box of `edges` in `frame`.
pub(crate) fn edges_bounding_box<'a, P: Payload + 'a>(
    edges: impl IntoIterator<Item = Edge<'a, P>>,
    frame: &Frame,
) -> Result<BBox, MeasureError> {
    Extremes::of_edges(edges)?.box_in(frame)
}

/// The box of `faces` in `frame`, their edges included.
pub(crate) fn faces_bounding_box<'a, P: Payload + 'a>(
    faces: impl IntoIterator<Item = Face<'a, P>>,
    frame: &Frame,
) -> Result<BBox, MeasureError> {
    Extremes::of_faces(faces)?.box_in(frame)
}
