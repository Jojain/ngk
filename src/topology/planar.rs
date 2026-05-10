use std::ops::Deref;

use nalgebra::Vector3;

use crate::geometry::{Curve, LINEAR_TOLERANCE, Plane, Point3, Surface};
use crate::topology::face::Face;

use super::closed::Closed;
use super::edge::Edge;
use super::gmap::{Dart, GMap, MergeTopology};
use super::payload::Payload;
use super::profile::Profile;

/// Marker for topology views that can carry a verified support plane.
pub trait PlanarGeometry {}

impl<'a, P: Payload> PlanarGeometry for Edge<'a, P> {}
impl<'a, P: Payload> PlanarGeometry for Profile<'a, P> {}
impl<'a, P: Payload> PlanarGeometry for Closed<Profile<'a, P>> {}
impl<'a, P: Payload> PlanarGeometry for Face<'a, P> {}

pub const DEFAULT_PLANAR_TOLERANCE: f64 = LINEAR_TOLERANCE;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PlanarityError {
    MissingVertexPoint {
        dart: Dart,
    },
    TooFewDistinctPoints {
        count: usize,
    },
    NonPlanarPoint {
        dart: Dart,
        distance: f64,
        tolerance: f64,
    },
    NonPlanarCurve {
        dart: Dart,
        distance: f64,
        tolerance: f64,
    },
    NonPlanarSurface,
}

pub trait PlanarityCheck: PlanarGeometry {
    fn support_plane(&self, tolerance: f64) -> Result<Plane, PlanarityError>;
}

/// Wrapper that statically carries "the inner value lies on this plane".
///
/// The invariant is trusted at construction time. If the underlying topology or
/// geometry mutates afterwards, the wrapper does not re-check planarity.
pub struct Planar<T: PlanarGeometry> {
    inner: T,
    plane: Plane,
}

impl<T: PlanarGeometry + Clone> Clone for Planar<T> {
    fn clone(&self) -> Self {
        Self {
            inner: self.inner.clone(),
            plane: self.plane.clone(),
        }
    }
}

impl<T: PlanarityCheck> Planar<T> {
    pub fn new(inner: T) -> Result<Self, PlanarityError> {
        Self::new_with_tolerance(inner, DEFAULT_PLANAR_TOLERANCE)
    }

    pub fn new_with_tolerance(inner: T, tolerance: f64) -> Result<Self, PlanarityError> {
        let plane = inner.support_plane(tolerance)?;
        Ok(Self { inner, plane })
    }
}

impl<T: PlanarGeometry> Planar<T> {
    pub fn new_unchecked(inner: T, plane: Plane) -> Self {
        Self { inner, plane }
    }

    pub fn into_inner(self) -> T {
        self.inner
    }

    pub fn inner(&self) -> &T {
        &self.inner
    }

    pub fn plane(&self) -> &Plane {
        &self.plane
    }

    pub fn into_parts(self) -> (T, Plane) {
        (self.inner, self.plane)
    }
}

impl<P: Payload> PlanarityCheck for Edge<'_, P> {
    fn support_plane(&self, tolerance: f64) -> Result<Plane, PlanarityError> {
        let points = edge_planarity_points(self)?;
        let plane = fit_plane(&points, tolerance)?;
        check_edge_curve(self, &plane, tolerance)?;
        Ok(plane)
    }
}

impl<P: Payload> PlanarityCheck for Profile<'_, P> {
    fn support_plane(&self, tolerance: f64) -> Result<Plane, PlanarityError> {
        let points = profile_planarity_points(self)?;
        let plane = fit_plane(&points, tolerance)?;
        check_points_on_plane(&points, &plane, tolerance)?;
        for edge in self.edges() {
            check_edge_curve(&edge, &plane, tolerance)?;
        }
        Ok(plane)
    }
}

impl<P: Payload> PlanarityCheck for Closed<Profile<'_, P>> {
    fn support_plane(&self, tolerance: f64) -> Result<Plane, PlanarityError> {
        self.inner().support_plane(tolerance)
    }
}

impl<P: Payload> PlanarityCheck for Face<'_, P> {
    fn support_plane(&self, tolerance: f64) -> Result<Plane, PlanarityError> {
        let Surface::Plane(plane) = self.surface() else {
            return Err(PlanarityError::NonPlanarSurface);
        };

        let loops = std::iter::once(self.outer_loop())
            .chain(self.inner_loops())
            .collect::<Vec<_>>();
        for loop_ in loops {
            let points = profile_points(loop_.inner())?;
            check_points_on_plane(&points, plane, tolerance)?;
            for edge in loop_.edges() {
                check_edge_curve(&edge, plane, tolerance)?;
            }
        }
        Ok(plane.clone())
    }
}

#[derive(Clone, Copy)]
struct PointOnDart {
    dart: Dart,
    point: Point3,
}

fn edge_points<P: Payload>(edge: &Edge<'_, P>) -> Result<Vec<PointOnDart>, PlanarityError> {
    [edge.start(), edge.end()]
        .into_iter()
        .map(|vertex| {
            let point = *vertex
                .point()
                .ok_or(PlanarityError::MissingVertexPoint { dart: vertex.dart })?;
            Ok(PointOnDart {
                dart: vertex.dart,
                point,
            })
        })
        .collect()
}

fn profile_points<P: Payload>(
    profile: &Profile<'_, P>,
) -> Result<Vec<PointOnDart>, PlanarityError> {
    profile
        .vertices()
        .into_iter()
        .map(|vertex| {
            let point = *vertex
                .point()
                .ok_or(PlanarityError::MissingVertexPoint { dart: vertex.dart })?;
            Ok(PointOnDart {
                dart: vertex.dart,
                point,
            })
        })
        .collect()
}

fn fit_plane(points: &[PointOnDart], tolerance: f64) -> Result<Plane, PlanarityError> {
    let Some(origin) = points.first() else {
        return Err(PlanarityError::TooFewDistinctPoints { count: 0 });
    };
    let Some(second) = points
        .iter()
        .skip(1)
        .find(|candidate| (candidate.point - origin.point).norm() > tolerance)
    else {
        return Err(PlanarityError::TooFewDistinctPoints { count: 1 });
    };
    let x_dir = second.point - origin.point;
    let Some(third) = points.iter().skip(2).find(|candidate| {
        let y_dir = candidate.point - origin.point;
        x_dir.cross(&y_dir).norm() > tolerance
    }) else {
        return Ok(Plane::from_xy(
            origin.point,
            x_dir,
            fallback_plane_y_dir(x_dir),
        ));
    };
    let y_dir = third.point - origin.point;
    let plane = Plane::from_xy(origin.point, x_dir, y_dir);
    check_points_on_plane(points, &plane, tolerance)?;
    Ok(plane)
}

fn edge_planarity_points<P: Payload>(
    edge: &Edge<'_, P>,
) -> Result<Vec<PointOnDart>, PlanarityError> {
    let mut points = edge_points(edge)?;
    if let Some(curve) = edge.curve() {
        let (t0, t1) = match (edge.start().point(), edge.end().point()) {
            (Some(start), Some(end)) => curve.parameters_between(*start, *end),
            _ => return Ok(points),
        };
        points.extend(
            sampled_curve_points(curve, t0, t1)
                .into_iter()
                .map(|point| PointOnDart {
                    dart: edge.dart,
                    point,
                }),
        );
    }
    Ok(points)
}

fn profile_planarity_points<P: Payload>(
    profile: &Profile<'_, P>,
) -> Result<Vec<PointOnDart>, PlanarityError> {
    let mut points = profile_points(profile)?;
    for edge in profile.edges() {
        points.extend(edge_planarity_points(&edge)?);
    }
    Ok(points)
}

fn check_points_on_plane(
    points: &[PointOnDart],
    plane: &Plane,
    tolerance: f64,
) -> Result<(), PlanarityError> {
    for point in points {
        let distance = plane_distance(plane, point.point);
        if distance > tolerance {
            return Err(PlanarityError::NonPlanarPoint {
                dart: point.dart,
                distance,
                tolerance,
            });
        }
    }
    Ok(())
}

fn check_edge_curve<P: Payload>(
    edge: &Edge<'_, P>,
    plane: &Plane,
    tolerance: f64,
) -> Result<(), PlanarityError> {
    let Some(curve) = edge.curve() else {
        return Ok(());
    };

    let (t0, t1) = match (edge.start().point(), edge.end().point()) {
        (Some(start), Some(end)) => curve.parameters_between(*start, *end),
        _ => return Ok(()),
    };

    for point in sampled_curve_points(curve, t0, t1) {
        let distance = plane_distance(plane, point);
        if distance > tolerance {
            return Err(PlanarityError::NonPlanarCurve {
                dart: edge.dart,
                distance,
                tolerance,
            });
        }
    }
    Ok(())
}

fn sampled_curve_points(curve: &Curve, t0: f64, t1: f64) -> Vec<Point3> {
    match curve {
        Curve::Line(_) => vec![curve.point_at(t0), curve.point_at(t1)],
        Curve::Circle(_) | Curve::Nurbs(_) => {
            let segments = 32usize;
            (0..=segments)
                .map(|i| {
                    let t = t0 + (t1 - t0) * (i as f64 / segments as f64);
                    curve.point_at(t)
                })
                .collect()
        }
    }
}

fn plane_distance(plane: &Plane, point: Point3) -> f64 {
    (point - plane.origin()).dot(&plane.normal()).abs()
}

fn fallback_plane_y_dir(x_dir: Vector3<f64>) -> Vector3<f64> {
    let axis = if x_dir.cross(&Vector3::z()).norm_squared() > LINEAR_TOLERANCE * LINEAR_TOLERANCE {
        Vector3::z()
    } else {
        Vector3::y()
    };
    x_dir.cross(&axis)
}

impl<T: PlanarGeometry> Deref for Planar<T> {
    type Target = T;

    fn deref(&self) -> &T {
        &self.inner
    }
}

impl<P, T> MergeTopology<P> for Planar<T>
where
    P: Payload,
    T: PlanarGeometry + MergeTopology<P>,
{
    fn source_map(&self) -> &GMap<P> {
        self.inner.source_map()
    }

    fn merge_darts(&self) -> Vec<Dart> {
        self.inner.merge_darts()
    }

    fn handle_dart(&self) -> Dart {
        self.inner.handle_dart()
    }
}

pub type PlanarLoop<'a, P> = Planar<Closed<Profile<'a, P>>>;
pub type PlanarProfile<'a, P> = Planar<Profile<'a, P>>;
