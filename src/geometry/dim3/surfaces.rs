use super::curves::{Curve, Periodicity, circle_nurbs_control_points, circle_nurbs_knots};
use super::frame::Frame;
use super::intersections::{
    IntersectionError, IntersectionOptions, SurfaceSurfaceIntersections, intersect_surfaces,
    intersect_surfaces_with_options,
};
use super::nurbs::{ControlNet, Degree, HPoint, KnotVector, NurbsSurface};
use super::utils::{IntoUnit, Point3};
use crate::geometry::LINEAR_TOLERANCE;
use crate::geometry::axis::Axis3;
use crate::geometry::nurbs::error::NurbsError;
use crate::geometry::{Interval, Point2};
use nalgebra::{Matrix2, Rotation3, UnitVector3, Vector2, Vector3};
use serde::{Deserialize, Serialize};

#[derive(Clone, Serialize, Deserialize)]
pub enum Surface {
    Plane(Plane),
    Cylinder(Cylinder),
    Ruled(RuledSurface),
    Revolution(SurfaceOfRevolution),
    Nurbs(NurbsSurface),
}

/// Periodicity of a surface's parameter-space directions.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SurfacePeriodicity {
    /// Neither parameter direction is periodic.
    None,
    /// Only the `u` parameter direction is periodic.
    UPeriodic(f64),
    /// Only the `v` parameter direction is periodic.
    VPeriodic(f64),
    /// Both parameter directions are periodic.
    UVPeriodic(f64, f64),
}

impl Surface {
    /// Returns the periodicity of the surface's parameter-space directions.
    pub fn periodicity(&self) -> SurfacePeriodicity {
        match self {
            Surface::Plane(_) | Surface::Nurbs(_) => SurfacePeriodicity::None,
            Surface::Cylinder(_) => SurfacePeriodicity::UPeriodic(std::f64::consts::TAU),
            Surface::Ruled(surface) => match surface.curve.periodicity() {
                Periodicity::None => SurfacePeriodicity::None,
                Periodicity::Periodic(period) => SurfacePeriodicity::UPeriodic(period),
            },
            Surface::Revolution(_) => SurfacePeriodicity::VPeriodic(std::f64::consts::TAU),
        }
    }

    pub fn to_nurbs(&self) -> Result<NurbsSurface, NurbsError> {
        match self {
            Surface::Plane(plane) => plane.to_nurbs(),
            Surface::Cylinder(cylinder) => cylinder.to_nurbs(),
            Surface::Ruled(surface) => surface.to_nurbs(),
            Surface::Revolution(surface) => surface.to_nurbs(),
            Surface::Nurbs(surface) => Ok(surface.clone()),
        }
    }

    /// Converts to NURBS realized over the requested parameter box.
    ///
    /// An unbounded analytic surface spans the box exactly, so callers holding a
    /// trim domain no longer silently lose the part of it outside the default
    /// unit patch. Surfaces already carrying their own finite parameterization
    /// ignore the box and return their full extent.
    pub fn to_nurbs_over(&self, u: Interval, v: Interval) -> Result<NurbsSurface, NurbsError> {
        match self {
            Surface::Plane(plane) => plane.to_nurbs_over(u, v),
            surface => surface.to_nurbs(),
        }
    }

    pub fn point_at(&self, u: f64, v: f64) -> Point3 {
        match self {
            Surface::Plane(p) => p.point_at(u, v),
            Surface::Cylinder(c) => c.point_at(u, v),
            Surface::Ruled(s) => s.point_at(u, v),
            Surface::Revolution(s) => s.point_at(u, v),
            Surface::Nurbs(s) => s.point_at(u, v),
        }
    }

    pub fn closest_parameter(&self, point: Point3) -> Result<Point2, NurbsError> {
        match self {
            Surface::Plane(plane) => Ok(plane.parameter_at(point)),
            Surface::Cylinder(cylinder) => Ok(cylinder.closest_parameter(point)),
            Surface::Ruled(surface) => Ok(surface.closest_parameter(point)),
            Surface::Nurbs(surface) => Ok(surface.closest_parameter(point)),
            surface => Ok(surface.to_nurbs()?.closest_parameter(point)),
        }
    }

    pub fn normal_at(&self, u: f64, v: f64) -> UnitVector3<f64> {
        match self {
            Surface::Plane(p) => p.normal(),
            Surface::Cylinder(c) => c.normal_at(u, v),
            Surface::Ruled(s) => s.normal_at(u, v),
            Surface::Revolution(s) => s.normal_at(u, v),
            Surface::Nurbs(s) => s.normal_at(u, v),
        }
    }

    pub fn intersect_surface(
        &self,
        other: &Surface,
    ) -> Result<SurfaceSurfaceIntersections, IntersectionError> {
        intersect_surfaces(self, other)
    }

    /// Intersects this surface with another using the supplied solver options.
    pub fn intersect_surface_with_options(
        &self,
        other: &Surface,
        options: IntersectionOptions,
    ) -> Result<SurfaceSurfaceIntersections, IntersectionError> {
        intersect_surfaces_with_options(self, other, options)
    }

    /// Returns this surface rotated by `angle` radians around `axis`.
    ///
    /// The parameterisation is preserved, so pcurves expressed in this
    /// surface's parameter space stay valid on the rotated copy.
    pub fn rotated(&self, axis: Axis3, angle: f64) -> Result<Self, NurbsError> {
        let rotation = Rotation3::from_axis_angle(&axis.direction, angle);
        let rotate = |point: Point3| axis.origin + rotation * (point - axis.origin);
        match self {
            Surface::Plane(plane) => Ok(Surface::Plane(Plane::from_xy(
                rotate(plane.origin()),
                rotation * *plane.x_dir(),
                rotation * *plane.y_dir(),
            ))),
            Surface::Cylinder(cylinder) => Ok(Surface::Cylinder(Cylinder::new(
                rotate(cylinder.origin()),
                rotation * *cylinder.x_dir(),
                rotation * *cylinder.axis(),
                cylinder.radius,
            ))),
            Surface::Ruled(surface) => Ok(Surface::Ruled(RuledSurface::new(
                surface.curve.rotated(axis, angle)?,
                rotation * surface.direction,
            ))),
            Surface::Revolution(surface) => Ok(Surface::Revolution(SurfaceOfRevolution::new(
                surface.curve.rotated(axis, angle)?,
                Axis3::new(
                    rotate(surface.axis.origin),
                    rotation * *surface.axis.direction,
                ),
            ))),
            Surface::Nurbs(surface) => {
                let control_points = surface
                    .control_points()
                    .as_slice()
                    .iter()
                    .map(|point| {
                        HPoint::from_cartesian(rotate(point.to_cartesian()), point.weight())
                    })
                    .collect();
                let control_points = ControlNet::new(
                    control_points,
                    surface.control_points().nu(),
                    surface.control_points().nv(),
                )?;
                Ok(Surface::Nurbs(NurbsSurface::new(
                    surface.degree_u(),
                    surface.degree_v(),
                    control_points,
                    surface.knots_u().clone(),
                    surface.knots_v().clone(),
                )?))
            }
        }
    }

    pub fn translated(&self, direction: Vector3<f64>) -> Result<Self, NurbsError> {
        match self {
            Surface::Plane(plane) => Ok(Surface::Plane(Plane::from_xy(
                plane.origin() + direction,
                plane.x_dir(),
                plane.y_dir(),
            ))),
            Surface::Cylinder(cylinder) => Ok(Surface::Cylinder(Cylinder::new(
                cylinder.origin() + direction,
                cylinder.x_dir(),
                cylinder.axis(),
                cylinder.radius,
            ))),
            Surface::Ruled(surface) => Ok(Surface::Ruled(RuledSurface::new(
                surface.curve.translated(direction)?,
                surface.direction,
            ))),
            Surface::Revolution(surface) => Ok(Surface::Revolution(SurfaceOfRevolution::new(
                surface.curve.translated(direction)?,
                surface.axis,
            ))),
            Surface::Nurbs(surface) => {
                let control_points = surface
                    .control_points()
                    .as_slice()
                    .iter()
                    .map(|point| {
                        HPoint::from_cartesian(point.to_cartesian() + direction, point.weight())
                    })
                    .collect();
                let control_points = ControlNet::new(
                    control_points,
                    surface.control_points().nu(),
                    surface.control_points().nv(),
                )?;
                Ok(Surface::Nurbs(NurbsSurface::new(
                    surface.degree_u(),
                    surface.degree_v(),
                    control_points,
                    surface.knots_u().clone(),
                    surface.knots_v().clone(),
                )?))
            }
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Plane {
    pub frame: Frame,
}

impl Plane {
    pub fn new(origin: Point3, x_dir: impl IntoUnit<3>, normal: impl IntoUnit<3>) -> Self {
        Self {
            frame: Frame::from_xz(origin, x_dir, normal),
        }
    }
    pub fn from_xy(origin: Point3, x_dir: impl IntoUnit<3>, y_dir: impl IntoUnit<3>) -> Self {
        Self {
            frame: Frame::from_xy(origin, x_dir, y_dir),
        }
    }

    pub fn xy() -> Self {
        Self::from_xy(Point3::origin(), Vector3::x(), Vector3::y())
    }
    pub fn xz() -> Self {
        Self::from_xy(Point3::origin(), Vector3::x(), Vector3::z())
    }
    pub fn yz() -> Self {
        Self::from_xy(Point3::origin(), Vector3::y(), Vector3::z())
    }

    pub fn point_at(&self, u: f64, v: f64) -> Point3 {
        self.frame.origin + u * *self.frame.x_dir + v * *self.frame.y_dir
    }

    pub fn parameter_at(&self, point: Point3) -> Point2 {
        let offset = point - self.origin();
        Point2::new(offset.dot(&self.x_dir()), offset.dot(&self.y_dir()))
    }

    pub fn origin(&self) -> Point3 {
        self.frame.origin
    }

    pub fn x_dir(&self) -> UnitVector3<f64> {
        self.frame.x_dir
    }

    pub fn y_dir(&self) -> UnitVector3<f64> {
        self.frame.y_dir
    }

    pub fn normal(&self) -> UnitVector3<f64> {
        self.frame.z_dir
    }

    pub fn to_nurbs(&self) -> Result<NurbsSurface, NurbsError> {
        self.to_nurbs_over(Interval::new(0.0, 1.0), Interval::new(0.0, 1.0))
    }

    /// Realizes this unbounded plane as the NURBS patch spanning `u` x `v`.
    ///
    /// The patch keeps the plane's own parameterization, so a point at plane
    /// parameters `(u, v)` inside the box has the same parameters on the patch.
    pub fn to_nurbs_over(&self, u: Interval, v: Interval) -> Result<NurbsSurface, NurbsError> {
        let origin = self.origin();
        let x = *self.x_dir();
        let y = *self.y_dir();
        let corner = |su: f64, sv: f64| HPoint::from_cartesian(origin + x * su + y * sv, 1.0);
        NurbsSurface::new(
            Degree::new(1)?,
            Degree::new(1)?,
            ControlNet::new(
                vec![
                    corner(u.start, v.start),
                    corner(u.end, v.start),
                    corner(u.start, v.end),
                    corner(u.end, v.end),
                ],
                2,
                2,
            )?,
            linear_knots(u)?,
            linear_knots(v)?,
        )
    }
}

#[derive(Clone, Serialize, Deserialize)]
pub struct Cylinder {
    pub frame: Frame,
    pub radius: f64,
}

impl Cylinder {
    pub fn new(
        origin: Point3,
        x_dir: impl IntoUnit<3>,
        axis: impl IntoUnit<3>,
        radius: f64,
    ) -> Self {
        Self {
            frame: Frame::from_xz(origin, x_dir, axis),
            radius,
        }
    }

    pub fn point_at(&self, u: f64, v: f64) -> Point3 {
        let rot = Rotation3::from_axis_angle(&self.axis(), u);
        let radial_dir = rot * self.x_dir();

        self.origin() + self.radius * *radial_dir + v * *self.axis()
    }

    pub fn normal_at(&self, u: f64, _v: f64) -> UnitVector3<f64> {
        let origin = self.origin();
        let projected = self.point_at(u, 0.0);
        (projected - origin).normalized()
    }

    pub fn closest_parameter(&self, point: Point3) -> Point2 {
        let offset = point - self.origin();
        let v = offset.dot(&self.axis());
        let radial = offset - *self.axis() * v;
        let y_dir = self.axis().cross(&self.x_dir());
        let mut u = radial.dot(&y_dir).atan2(radial.dot(&self.x_dir()));
        if u < 0.0 {
            u += std::f64::consts::TAU;
        }
        Point2::new(u, v)
    }

    pub fn origin(&self) -> Point3 {
        self.frame.origin
    }

    pub fn x_dir(&self) -> UnitVector3<f64> {
        self.frame.x_dir
    }

    pub fn axis(&self) -> UnitVector3<f64> {
        self.frame.z_dir
    }

    pub fn to_nurbs(&self) -> Result<NurbsSurface, NurbsError> {
        let circle = super::curves::Circle::new(
            Plane::new(self.origin(), self.x_dir(), self.axis()),
            self.radius,
        );
        let (circle_points, circle_weights) =
            circle_nurbs_control_points(circle.plane(), circle.radius());
        let points = circle_points
            .iter()
            .zip(circle_weights.iter().copied())
            .map(|(point, weight)| HPoint::from_cartesian(point.to_cartesian(), weight))
            .chain(
                circle_points
                    .iter()
                    .zip(circle_weights.iter().copied())
                    .map(|(point, weight)| {
                        HPoint::from_cartesian(point.to_cartesian() + *self.axis(), weight)
                    }),
            )
            .collect();

        NurbsSurface::new(
            Degree::new(2)?,
            Degree::new(1)?,
            ControlNet::new(points, 9, 2)?,
            circle_nurbs_knots()?,
            unit_linear_knots()?,
        )
    }
}

#[derive(Clone, Serialize, Deserialize)]
pub struct RuledSurface {
    curve: Curve,
    direction: Vector3<f64>,
}

impl RuledSurface {
    pub fn new(curve: Curve, direction: Vector3<f64>) -> Self {
        Self { curve, direction }
    }

    pub fn curve(&self) -> &Curve {
        &self.curve
    }

    pub fn direction(&self) -> Vector3<f64> {
        self.direction
    }

    pub fn point_at(&self, u: f64, v: f64) -> Point3 {
        self.curve.point_at(u) + self.direction * v
    }

    /// Returns the least-squares source parameters of a point on the ruled surface.
    pub fn closest_parameter(&self, point: Point3) -> Point2 {
        let direction_squared = self.direction.norm_squared();
        let mut u = self.curve.param_at(point);
        let mut v = if direction_squared > LINEAR_TOLERANCE * LINEAR_TOLERANCE {
            (point - self.curve.point_at(u)).dot(&self.direction) / direction_squared
        } else {
            0.0
        };
        for _ in 0..16 {
            let residual = self.point_at(u, v) - point;
            let du = self.curve.derivative_at(u, 1);
            let jacobian = Matrix2::new(
                du.dot(&du),
                du.dot(&self.direction),
                du.dot(&self.direction),
                direction_squared,
            );
            let rhs = Vector2::new(-du.dot(&residual), -self.direction.dot(&residual));
            let Some(delta) = jacobian.lu().solve(&rhs) else {
                break;
            };
            u += delta.x;
            v += delta.y;
            if delta.norm() <= 1.0e-12 {
                break;
            }
        }
        Point2::new(u, v)
    }

    pub fn normal_at(&self, u: f64, _v: f64) -> UnitVector3<f64> {
        let du = self.curve.derivative_at(u, 1);
        let n = du.cross(&self.direction);
        match UnitVector3::try_new(n, LINEAR_TOLERANCE) {
            Some(n) => n,
            None => Vector3::z_axis(),
        }
    }

    pub fn to_nurbs(&self) -> Result<NurbsSurface, NurbsError> {
        let curve = self.curve.to_nurbs()?;
        let points = curve
            .control_points()
            .iter()
            .map(|point| HPoint::from_cartesian(point.to_cartesian(), point.weight()))
            .chain(curve.control_points().iter().map(|point| {
                HPoint::from_cartesian(point.to_cartesian() + self.direction, point.weight())
            }))
            .collect();

        NurbsSurface::new(
            curve.degree(),
            Degree::new(1)?,
            ControlNet::new(points, curve.control_points().len(), 2)?,
            curve.knots().clone(),
            unit_linear_knots()?,
        )
    }
}

#[derive(Clone, Serialize, Deserialize)]
pub struct SurfaceOfRevolution {
    curve: Curve,
    pub axis: Axis3,
}

impl SurfaceOfRevolution {
    pub fn new(curve: Curve, axis: Axis3) -> Self {
        Self { curve, axis }
    }

    pub fn curve(&self) -> &Curve {
        &self.curve
    }

    pub fn origin(&self) -> Point3 {
        self.axis.origin
    }

    pub fn point_at(&self, u: f64, v: f64) -> Point3 {
        // u walks the profile curve, v is the angle [0, 2π]
        let p = self.curve.point_at(u);

        // Project p onto the axis, then get the radial component
        let proj = self.axis.project(p);
        let radial = p - proj;

        // Rotate the radial part by angle v around the axis
        let rot = Rotation3::from_axis_angle(&self.axis.direction, v);
        proj + (rot * radial)
    }

    /// Returns the unit surface normal, `dS/du x dS/dv`.
    ///
    /// Degenerates on the axis, where `dS/dv` vanishes and the surface has an
    /// apex rather than a tangent plane; the axis direction is returned there.
    pub fn normal_at(&self, u: f64, v: f64) -> UnitVector3<f64> {
        let du = self.partial_u(u, v);
        let dv = self.partial_v(u, v);
        let n = du.cross(&dv);
        UnitVector3::try_new(n, LINEAR_TOLERANCE).unwrap_or(self.axis.direction)
    }

    /// Analytic `dS/du`: the profile tangent carried around the axis by `v`.
    ///
    /// Rotating about the axis is a rigid motion independent of `u`, so it
    /// commutes with differentiation along the profile.
    fn partial_u(&self, u: f64, v: f64) -> Vector3<f64> {
        Rotation3::from_axis_angle(&self.axis.direction, v) * self.curve.derivative_at(u, 1)
    }

    /// Analytic `dS/dv`: the rotational velocity `axis x radius`.
    fn partial_v(&self, u: f64, v: f64) -> Vector3<f64> {
        let point = self.point_at(u, v);
        self.axis
            .direction
            .cross(&(point - self.axis.project(point)))
    }

    pub fn to_nurbs(&self) -> Result<NurbsSurface, NurbsError> {
        let curve = self.curve.to_nurbs()?;
        let mut points = Vec::with_capacity(curve.control_points().len() * 9);
        let angular_weights = [
            1.0,
            std::f64::consts::FRAC_1_SQRT_2,
            1.0,
            std::f64::consts::FRAC_1_SQRT_2,
            1.0,
            std::f64::consts::FRAC_1_SQRT_2,
            1.0,
            std::f64::consts::FRAC_1_SQRT_2,
            1.0,
        ];

        for (angle_index, angular_weight) in angular_weights.iter().copied().enumerate() {
            let angle = angle_index as f64 * std::f64::consts::FRAC_PI_4;
            let is_midpoint = angle_index % 2 == 1;

            let radial_scale = if is_midpoint {
                std::f64::consts::SQRT_2
            } else {
                1.0
            };
            let rotation = Rotation3::from_axis_angle(&self.axis.direction, angle);

            for point in curve.control_points().iter() {
                let p = point.to_cartesian();
                let proj = self.axis.project(p);
                let radial = p - proj;
                let revolved = proj + rotation * (radial * radial_scale);
                points.push(HPoint::from_cartesian(
                    revolved,
                    point.weight() * angular_weight,
                ));
            }
        }

        NurbsSurface::new(
            curve.degree(),
            Degree::new(2)?,
            ControlNet::new(points, curve.control_points().len(), 9)?,
            curve.knots().clone(),
            circle_nurbs_knots()?,
        )
    }
}

fn unit_linear_knots() -> Result<KnotVector, NurbsError> {
    linear_knots(Interval::new(0.0, 1.0))
}

/// Clamped degree-1 knots spanning `domain`.
fn linear_knots(domain: Interval) -> Result<KnotVector, NurbsError> {
    KnotVector::new(vec![domain.start, domain.start, domain.end, domain.end])
}
