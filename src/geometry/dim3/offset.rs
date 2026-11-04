//! Offsets: the support every point of which lies a signed distance from
//! another one, measured along its normal.
//!
//! **An offset keeps its source's parameterization.** The offset of `S` by `d`
//! is `S(u, v) + d·N(u, v)`, evaluated at the *same* `(u, v)`, and every form
//! this module returns honours that: a pcurve drawn on a face's surface draws
//! the matching boundary on its offset unchanged. That is what lets a face be
//! offset without re-deriving its boundary, and it is why an analytic offset
//! is built by moving or resizing the source rather than by fitting it.
//!
//! - **Exact** where the offset is the same kind of support: a plane moves, a
//!   cylinder, sphere or torus changes radius, a cone moves and changes its
//!   reference radius; a line moves and a circle changes radius. An extrusion
//!   or a revolution whose profile lies in the plane its normals sweep offsets
//!   its profile in that plane, which is exact whenever the profile's own
//!   offset is.
//! - **Fitted** otherwise: a cubic interpolation of offset samples taken at
//!   the source's own native parameters, refined until it stays within
//!   [`OFFSET_FIT_TOLERANCE`] between the samples.
//!
//! **An offset that folds is refused, not trimmed.** Moving a support farther
//! than its radius of curvature on the concave side turns it inside out: a
//! circle collapses to a point and then grows back reversed, a free-form
//! curve grows a cusp and a swallowtail. Trimming that away is a topological
//! decision about which part survives, which belongs to the caller that knows
//! the entity, so the geometry reports where it folds and stops.
//!
//! Sign conventions:
//!
//! - A surface moves along [`Surface::normal_at`]: a positive distance grows a
//!   sphere, a negative one shrinks it.
//! - A planar curve moves along `T × n`, the tangent crossed with the normal
//!   of its plane: a positive distance moves a loop running counter-clockwise
//!   about `n` outward, and moves a line to its right when seen from `n`.

use super::curves::{Circle, Curve};
use super::nurbs::{ControlNet, ControlPolygon, Degree};
use super::nurbs::{HPoint, InterpolationSystem, KnotVector, NurbsCurve, NurbsSurface};
use super::surfaces::{Cone, Cylinder, RuledSurface, Sphere, Surface, SurfaceOfRevolution, Torus};
use super::utils::Point3;
use crate::geometry::interval::Interval;
use crate::geometry::nurbs::error::NurbsError;
use crate::geometry::parameter::NativeParam;
use crate::geometry::transform::Rigid;
use crate::geometry::{ANGULAR_TOLERANCE, LINEAR_TOLERANCE, Plane};
use nalgebra::{UnitVector3, Vector3};

/// How far a fitted offset may stray from the exact offset between the
/// samples it interpolates.
///
/// The same order as a fitted intersection curve's, since an offset support
/// meets the rest of a model through exactly those curves.
pub const OFFSET_FIT_TOLERANCE: f64 = 3.0e-5;

/// Samples a fitted curve offset starts with, per span of the source.
const INITIAL_CURVE_SAMPLES: usize = 17;
/// Samples past which a curve fit gives up.
const MAX_CURVE_SAMPLES: usize = 2049;
/// Samples per direction a fitted surface offset starts with.
const INITIAL_SURFACE_SAMPLES: usize = 9;
/// Samples per direction past which a surface fit gives up.
const MAX_SURFACE_SAMPLES: usize = 129;
/// Degree of every fitted offset, in each direction.
const FIT_DEGREE: usize = 3;

/// Why a support has no offset at the distance asked for.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum OffsetError {
    /// An analytic support shrinks to nothing or past it: a circle, cylinder
    /// or sphere whose radius the distance consumes.
    #[error("offset by {distance} collapses a support of radius {radius}")]
    Collapsed { distance: f64, radius: f64 },
    /// The distance exceeds the support's radius of curvature on the side it
    /// moves toward, so the offset turns inside out near these parameters.
    #[error("offset by {distance} folds the support near parameters ({u}, {v})")]
    Folded { distance: f64, u: f64, v: f64 },
    /// A curve offset in a plane must lie in a plane normal to it; a line must
    /// at least run perpendicular to it.
    #[error("the curve does not lie in a plane normal to the offset direction")]
    NotPlanar,
    /// A fitted offset needs a finite parameter window, and the support's
    /// domain is unbounded.
    #[error("a fitted offset needs a finite window; the support is unbounded")]
    Unbounded,
    /// The fit did not get within tolerance before running out of samples.
    #[error(
        "the fitted offset strays {deviation} from the exact one with {samples} samples, \
         more than {tolerance}"
    )]
    NotConverged {
        deviation: f64,
        tolerance: f64,
        samples: usize,
    },
    #[error(transparent)]
    Nurbs(#[from] NurbsError),
}

impl Curve {
    /// The curve moved `distance` along `T × normal` at every point.
    ///
    /// The curve must lie in a plane normal to `normal` (a line need only run
    /// perpendicular to it). A line and a circle offset exactly; anything
    /// else is fitted over its domain, which must then be finite. See the
    /// [module documentation](self) for sign and parameterization.
    pub fn offset_in_plane(
        &self,
        normal: UnitVector3<f64>,
        distance: f64,
    ) -> Result<Curve, OffsetError> {
        let domain = self.domain();
        self.offset_in_plane_over(normal, distance, domain)
    }

    /// [`Self::offset_in_plane`], fitting over `window` where the offset is
    /// not exact. An exact offset ignores the window.
    pub fn offset_in_plane_over(
        &self,
        normal: UnitVector3<f64>,
        distance: f64,
        window: Interval,
    ) -> Result<Curve, OffsetError> {
        if distance == 0.0 {
            return Ok(self.clone());
        }
        match self {
            Curve::Line(line) => {
                let side = line.direction().cross(&normal);
                if (side.norm() - 1.0).abs() > ANGULAR_TOLERANCE.sqrt() {
                    return Err(OffsetError::NotPlanar);
                }
                Ok(self.moved(&Rigid::translation(side * distance)))
            }
            Curve::Circle(circle) => {
                let alignment = circle.plane().normal().dot(&normal);
                if (alignment.abs() - 1.0).abs() > ANGULAR_TOLERANCE.sqrt() {
                    return Err(OffsetError::NotPlanar);
                }
                // Counter-clockwise about `normal`, `T × normal` points away
                // from the centre; clockwise, toward it.
                let radius = circle.radius() + alignment.signum() * distance;
                if radius <= LINEAR_TOLERANCE {
                    return Err(OffsetError::Collapsed {
                        distance,
                        radius: circle.radius(),
                    });
                }
                Ok(Curve::Circle(Circle::new(circle.plane().clone(), radius)))
            }
            Curve::Ellipse(ellipse) => {
                let alignment = ellipse.frame().z_dir.dot(&normal);
                if (alignment.abs() - 1.0).abs() > ANGULAR_TOLERANCE.sqrt() {
                    return Err(OffsetError::NotPlanar);
                }
                fit_planar_curve_offset(self, normal, distance, window)
            }
            Curve::Helix(_) => Err(OffsetError::NotPlanar),
            Curve::Nurbs(nurbs) => {
                let points = nurbs.control_points();
                let origin = points.as_slice()[0].to_cartesian();
                let scale = points
                    .iter()
                    .map(|point| (point.to_cartesian() - origin).norm())
                    .fold(1.0, f64::max);
                let planar = points.iter().all(|point| {
                    (point.to_cartesian() - origin).dot(&normal).abs() <= LINEAR_TOLERANCE * scale
                });
                if !planar {
                    return Err(OffsetError::NotPlanar);
                }
                fit_planar_curve_offset(self, normal, distance, window)
            }
        }
    }
}

impl Surface {
    /// The surface moved `distance` along its normal at every point.
    ///
    /// Exact for planes, cylinders, spheres, cones and tori, and for
    /// extrusions and revolutions whose profile offsets exactly; anything else
    /// is fitted over the surface's domain, which must then be finite. See the
    /// [module documentation](self) for sign and parameterization.
    pub fn offset(&self, distance: f64) -> Result<Surface, OffsetError> {
        let (u, v) = self.domain();
        self.offset_over(distance, u, v)
    }

    /// [`Self::offset`], fitting over the window `u × v` where the offset is
    /// not exact. An exact offset ignores the window.
    pub fn offset_over(
        &self,
        distance: f64,
        u: Interval,
        v: Interval,
    ) -> Result<Surface, OffsetError> {
        if distance == 0.0 {
            return Ok(self.clone());
        }
        match self {
            Surface::Plane(plane) => Ok(Surface::Plane(Plane::from_frame(
                plane
                    .frame
                    .moved(&Rigid::translation(*plane.normal() * distance)),
            ))),
            Surface::Cylinder(cylinder) => {
                let radius = grown_radius(cylinder.radius, distance)?;
                Ok(Surface::Cylinder(Cylinder {
                    frame: cylinder.frame.clone(),
                    radius,
                }))
            }
            Surface::Sphere(sphere) => {
                let radius = grown_radius(sphere.radius(), distance)?;
                Ok(Surface::Sphere(Sphere::new(sphere.frame().clone(), radius)))
            }
            Surface::Torus(torus) => {
                let minor = grown_radius(torus.minor_radius(), distance)?;
                Ok(Surface::Torus(Torus::new(
                    torus.frame().clone(),
                    torus.major_radius(),
                    minor,
                )))
            }
            Surface::Cone(cone) => Ok(Surface::Cone(offset_cone(cone, distance))),
            Surface::Ruled(ruled) => match offset_extrusion(ruled, distance, u)? {
                Some(offset) => Ok(offset),
                None => fit_surface_offset(self, distance, u, v),
            },
            Surface::Revolution(revolution) => match offset_revolution(revolution, distance, u)? {
                Some(offset) => Ok(offset),
                None => fit_surface_offset(self, distance, u, v),
            },
            Surface::Nurbs(_) => fit_surface_offset(self, distance, u, v),
        }
    }
}

/// A radius grown by `distance`, refusing one the distance consumes.
fn grown_radius(radius: f64, distance: f64) -> Result<f64, OffsetError> {
    let grown = radius + distance;
    if grown <= LINEAR_TOLERANCE {
        return Err(OffsetError::Collapsed { distance, radius });
    }
    Ok(grown)
}

/// A cone's offset on the nappe it is built on, where its radius is
/// non-negative.
///
/// The normal there leans back from the axis by the half angle, so every point
/// moves out by `d·cos a` and down the axis by `d·sin a`. That is the same cone
/// with its reference circle moved: the frame slides `-d·sin a` along the axis
/// and the reference radius grows by `d·cos a`, and `v` still names the same
/// generator height. Past the apex the offset cones overlap, which is the
/// apex's own business and not refused here.
fn offset_cone(cone: &Cone, distance: f64) -> Cone {
    let angle = cone.half_angle();
    let frame = cone.frame();
    let moved = frame.moved(&Rigid::translation(
        *frame.z_dir * (-distance * angle.sin()),
    ));
    Cone::new(
        moved,
        cone.reference_radius() + distance * angle.cos(),
        angle,
    )
}

/// The offset of an extrusion whose profile lies in a plane normal to its
/// direction, or `None` when it does not.
///
/// The extrusion's normal is `C' × d`, which is the profile's own in-plane
/// offset direction about `d`; so the offset is the profile's offset in that
/// plane, extruded the same way.
fn offset_extrusion(
    ruled: &RuledSurface,
    distance: f64,
    window: Interval,
) -> Result<Option<Surface>, OffsetError> {
    let Some(direction) = UnitVector3::try_new(ruled.direction(), LINEAR_TOLERANCE) else {
        return Ok(None);
    };
    match ruled
        .curve()
        .offset_in_plane_over(direction, distance, window)
    {
        Ok(profile) => Ok(Some(Surface::Ruled(RuledSurface::new(
            profile,
            ruled.direction(),
        )))),
        Err(OffsetError::NotPlanar | OffsetError::Unbounded) => Ok(None),
        Err(error) => Err(error),
    }
}

/// The offset of a revolution whose profile lies in a plane through its axis,
/// or `None` when it does not.
///
/// The revolution's normal stays in each meridian plane, so the offset is the
/// profile offset within its own meridian plane, revolved. Which side that
/// is depends on how the profile runs relative to the sweep, and is read off
/// the surface normal once.
fn offset_revolution(
    revolution: &SurfaceOfRevolution,
    distance: f64,
    window: Interval,
) -> Result<Option<Surface>, OffsetError> {
    let profile = revolution.curve();
    // Any profile point reads the side; an unbounded profile has no middle,
    // but a line's every point is as good as another.
    let probe = window.or_extent(1.0);
    let middle = NativeParam::new(0.5 * (probe.start.value() + probe.end.value()));
    let point = profile.point_at(middle);
    let tangent = profile.derivative_at(middle, 1);
    let radial = point - revolution.axis.project(point);
    let Some(meridian) =
        UnitVector3::try_new(revolution.axis.direction.cross(&radial), LINEAR_TOLERANCE)
    else {
        return Ok(None);
    };
    let in_plane = tangent.cross(&meridian);
    let normal = revolution.normal_at(middle.value(), 0.0);
    let side = in_plane.dot(&normal).signum();
    match profile.offset_in_plane_over(meridian, side * distance, window) {
        Ok(offset) => Ok(Some(Surface::Revolution(SurfaceOfRevolution::new(
            offset,
            revolution.axis,
        )))),
        Err(OffsetError::NotPlanar | OffsetError::Unbounded) => Ok(None),
        Err(error) => Err(error),
    }
}

/// Fits the in-plane offset of `curve` over `window` at its native parameters.
fn fit_planar_curve_offset(
    curve: &Curve,
    normal: UnitVector3<f64>,
    distance: f64,
    window: Interval,
) -> Result<Curve, OffsetError> {
    if !window.is_finite() {
        return Err(OffsetError::Unbounded);
    }
    let exact = |t: f64| -> Result<Point3, OffsetError> {
        let parameter = NativeParam::new(t);
        let first = curve.derivative_at(parameter, 1);
        let second = curve.derivative_at(parameter, 2);
        let speed = first.norm();
        if speed <= LINEAR_TOLERANCE {
            return Err(OffsetError::Folded {
                distance,
                u: t,
                v: 0.0,
            });
        }
        // Moving along `T × n` stretches the curve by `1 + d·κ`, with `κ` the
        // curvature signed positive where the curve turns counter-clockwise
        // about `n`. At zero the offset has a cusp; below, it runs backwards.
        let curvature = first.cross(&second).dot(&normal) / speed.powi(3);
        if 1.0 + distance * curvature <= ANGULAR_TOLERANCE.sqrt() {
            return Err(OffsetError::Folded {
                distance,
                u: t,
                v: 0.0,
            });
        }
        Ok(curve.point_at(parameter) + first.cross(&normal) * (distance / speed))
    };

    let mut count = INITIAL_CURVE_SAMPLES;
    let mut deviation = f64::INFINITY;
    while count <= MAX_CURVE_SAMPLES {
        let parameters = samples(window, count);
        let points = parameters
            .iter()
            .map(|&t| exact(t).map(|point| HPoint::from_cartesian(point, 1.0)))
            .collect::<Result<Vec<_>, _>>()?;
        let degree = Degree::new(FIT_DEGREE)?;
        let knots = KnotVector::averaged(&parameters, degree)?;
        let control = InterpolationSystem::new(&parameters, degree, &knots)?.solve(&points)?;
        let fitted = NurbsCurve::new(degree, control, knots)?;
        deviation = 0.0;
        for pair in parameters.windows(2) {
            let middle = 0.5 * (pair[0] + pair[1]);
            deviation = f64::max(deviation, (fitted.point_at(middle) - exact(middle)?).norm());
        }
        if deviation <= OFFSET_FIT_TOLERANCE {
            return Ok(Curve::Nurbs(fitted));
        }
        count = 2 * count - 1;
    }
    Err(OffsetError::NotConverged {
        deviation,
        tolerance: OFFSET_FIT_TOLERANCE,
        samples: count.div_ceil(2),
    })
}

/// Fits the offset of `surface` over `u × v` at its native parameters.
fn fit_surface_offset(
    surface: &Surface,
    distance: f64,
    u: Interval,
    v: Interval,
) -> Result<Surface, OffsetError> {
    if !u.is_finite() || !v.is_finite() {
        return Err(OffsetError::Unbounded);
    }
    let step = 1.0e-6 * f64::max(u.length(), v.length());
    let exact = |s: f64, t: f64| -> Result<Point3, OffsetError> {
        let (point, du, dv) = surface.point_and_derivative_at(s, t);
        let normal = surface.normal_at(s, t);
        // The offset's tangent plane is spanned by `Su + d·Nu` and
        // `Sv + d·Nv`. It faces the source's way until the distance reaches a
        // principal radius of curvature, and the other way past it.
        let (nu, nv) = normal_derivatives(surface, s, t, step);
        let offset_normal = (du + nu * distance).cross(&(dv + nv * distance));
        if offset_normal.dot(&du.cross(&dv)) <= 0.0 {
            return Err(OffsetError::Folded {
                distance,
                u: s,
                v: t,
            });
        }
        Ok(point + *normal * distance)
    };

    let degree = Degree::new(FIT_DEGREE)?;
    let mut count = INITIAL_SURFACE_SAMPLES;
    let mut deviation = f64::INFINITY;
    while count <= MAX_SURFACE_SAMPLES {
        let us = samples(u, count);
        let vs = samples(v, count);
        let mut grid = Vec::with_capacity(count * count);
        for &t in &vs {
            for &s in &us {
                grid.push(HPoint::from_cartesian(exact(s, t)?, 1.0));
            }
        }
        let fitted = interpolate_grid(&grid, &us, &vs, degree)?;
        deviation = 0.0;
        for pair_v in vs.windows(2) {
            let t = 0.5 * (pair_v[0] + pair_v[1]);
            for pair_u in us.windows(2) {
                let s = 0.5 * (pair_u[0] + pair_u[1]);
                deviation = f64::max(deviation, (fitted.point_at(s, t) - exact(s, t)?).norm());
            }
        }
        if deviation <= OFFSET_FIT_TOLERANCE {
            return Ok(Surface::Nurbs(fitted));
        }
        count = 2 * count - 1;
    }
    Err(OffsetError::NotConverged {
        deviation,
        tolerance: OFFSET_FIT_TOLERANCE,
        samples: count.div_ceil(2),
    })
}

/// The normal's partial derivatives, by central differences.
///
/// Only their sign against the tangent plane is read, so a difference is
/// accurate enough and spares every surface type a second-derivative form.
fn normal_derivatives(
    surface: &Surface,
    u: f64,
    v: f64,
    step: f64,
) -> (Vector3<f64>, Vector3<f64>) {
    let nu = (*surface.normal_at(u + step, v) - *surface.normal_at(u - step, v)) / (2.0 * step);
    let nv = (*surface.normal_at(u, v + step) - *surface.normal_at(u, v - step)) / (2.0 * step);
    (nu, nv)
}

/// The global tensor-product interpolation of a `us × vs` grid (u fastest):
/// every row along `u` first, then every column of the resulting control
/// points along `v`.
fn interpolate_grid(
    grid: &[HPoint],
    us: &[f64],
    vs: &[f64],
    degree: Degree,
) -> Result<NurbsSurface, NurbsError> {
    let knots_u = KnotVector::averaged(us, degree)?;
    let knots_v = KnotVector::averaged(vs, degree)?;
    let rows = InterpolationSystem::new(us, degree, &knots_u)?;
    let columns = InterpolationSystem::new(vs, degree, &knots_v)?;
    let (nu, nv) = (us.len(), vs.len());

    let mut intermediate = vec![HPoint::from_cartesian(Point3::origin(), 1.0); nu * nv];
    for row in 0..nv {
        let solved: ControlPolygon = rows.solve(&grid[row * nu..(row + 1) * nu])?;
        for (column, point) in solved.iter().enumerate() {
            intermediate[row * nu + column] = *point;
        }
    }
    let mut net = intermediate.clone();
    for column in 0..nu {
        let points = (0..nv)
            .map(|row| intermediate[row * nu + column])
            .collect::<Vec<_>>();
        let solved = columns.solve(&points)?;
        for (row, point) in solved.iter().enumerate() {
            net[row * nu + column] = *point;
        }
    }
    NurbsSurface::new(
        degree,
        degree,
        ControlNet::new(net, nu, nv)?,
        knots_u,
        knots_v,
    )
}

/// `count` evenly spaced native parameters spanning `window`, both ends
/// included.
fn samples(window: Interval, count: usize) -> Vec<f64> {
    let (start, end) = (window.start.value(), window.end.value());
    (0..count)
        .map(|index| start + (end - start) * index as f64 / (count - 1) as f64)
        .collect()
}
