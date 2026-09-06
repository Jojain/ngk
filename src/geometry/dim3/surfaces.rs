use super::bbox::BBox;
use super::curves::{Circle, Curve, Periodicity, circle_nurbs_control_points, circle_nurbs_knots};
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
use crate::geometry::traits::SurfaceGeometry;
use crate::geometry::{Interval, ParamMap, Point2, Reparam};
use nalgebra::{Matrix2, Rotation3, UnitVector3, Vector2, Vector3};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Surface {
    Plane(Plane),
    Cylinder(Cylinder),
    Sphere(Sphere),
    Cone(Cone),
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
    /// Returns the `(u, v)` parameter ranges over which the surface is defined.
    ///
    /// Unbounded directions report [`Interval::unbounded`]; a caller that needs
    /// a finite window clamps them with [`Interval::or_extent`].
    pub fn domain(&self) -> (Interval, Interval) {
        match self {
            Surface::Plane(surface) => SurfaceGeometry::domain(surface),
            Surface::Cylinder(surface) => SurfaceGeometry::domain(surface),
            Surface::Sphere(surface) => SurfaceGeometry::domain(surface),
            Surface::Cone(surface) => SurfaceGeometry::domain(surface),
            Surface::Ruled(surface) => SurfaceGeometry::domain(surface),
            Surface::Revolution(surface) => SurfaceGeometry::domain(surface),
            Surface::Nurbs(surface) => SurfaceGeometry::domain(surface),
        }
    }

    /// Returns the periodicity of the surface's parameter-space directions.
    pub fn periodicity(&self) -> SurfacePeriodicity {
        match self {
            Surface::Plane(surface) => SurfaceGeometry::periodicity(surface),
            Surface::Cylinder(surface) => SurfaceGeometry::periodicity(surface),
            Surface::Sphere(surface) => SurfaceGeometry::periodicity(surface),
            Surface::Cone(surface) => SurfaceGeometry::periodicity(surface),
            Surface::Ruled(surface) => SurfaceGeometry::periodicity(surface),
            Surface::Revolution(surface) => SurfaceGeometry::periodicity(surface),
            Surface::Nurbs(surface) => SurfaceGeometry::periodicity(surface),
        }
    }

    /// Returns whether the parameterization collapses at `(u, v)`.
    ///
    /// Tessellation and intersection seeding branch on this rather than
    /// emitting zero-area geometry at a pole or an apex.
    pub fn is_degenerate_at(&self, u: f64, v: f64) -> bool {
        match self {
            Surface::Plane(surface) => surface.is_degenerate_at(u, v),
            Surface::Cylinder(surface) => surface.is_degenerate_at(u, v),
            Surface::Sphere(surface) => surface.is_degenerate_at(u, v),
            Surface::Cone(surface) => surface.is_degenerate_at(u, v),
            Surface::Ruled(surface) => surface.is_degenerate_at(u, v),
            Surface::Revolution(surface) => surface.is_degenerate_at(u, v),
            Surface::Nurbs(surface) => surface.is_degenerate_at(u, v),
        }
    }

    pub fn to_nurbs(&self) -> Result<NurbsSurface, NurbsError> {
        match self {
            Surface::Plane(surface) => SurfaceGeometry::to_nurbs(surface),
            Surface::Cylinder(surface) => SurfaceGeometry::to_nurbs(surface),
            Surface::Sphere(surface) => SurfaceGeometry::to_nurbs(surface),
            Surface::Cone(surface) => SurfaceGeometry::to_nurbs(surface),
            Surface::Ruled(surface) => SurfaceGeometry::to_nurbs(surface),
            Surface::Revolution(surface) => SurfaceGeometry::to_nurbs(surface),
            Surface::Nurbs(surface) => SurfaceGeometry::to_nurbs(surface),
        }
    }

    /// Converts to NURBS realized over the requested parameter box.
    ///
    /// An unbounded analytic surface spans the box exactly, so callers holding
    /// a trim domain no longer silently lose the part of it outside the default
    /// patch. Surfaces already carrying their own finite parameterization
    /// ignore the box and return their full extent.
    ///
    /// The conversion reproduces the surface as a point set; it does not
    /// generally preserve the parameterization. See
    /// [`crate::geometry::traits`].
    pub fn to_nurbs_over(&self, u: Interval, v: Interval) -> Result<NurbsSurface, NurbsError> {
        match self {
            Surface::Plane(surface) => surface.to_nurbs_over(u, v),
            Surface::Cylinder(surface) => surface.to_nurbs_over(u, v),
            Surface::Sphere(surface) => surface.to_nurbs_over(u, v),
            Surface::Cone(surface) => surface.to_nurbs_over(u, v),
            Surface::Ruled(surface) => surface.to_nurbs_over(u, v),
            Surface::Revolution(surface) => surface.to_nurbs_over(u, v),
            Surface::Nurbs(surface) => surface.to_nurbs_over(u, v),
        }
    }

    /// Returns the analytic-to-NURBS parameter map for the requested patch.
    pub fn param_map_over(&self, u: Interval, v: Interval) -> ParamMap {
        match self {
            Surface::Plane(surface) => surface.param_map_over(u, v),
            Surface::Cylinder(surface) => surface.param_map_over(u, v),
            Surface::Sphere(surface) => surface.param_map_over(u, v),
            Surface::Cone(surface) => surface.param_map_over(u, v),
            Surface::Ruled(surface) => surface.param_map_over(u, v),
            Surface::Revolution(surface) => surface.param_map_over(u, v),
            Surface::Nurbs(surface) => surface.param_map_over(u, v),
        }
    }

    /// Returns a conservative finite box for the requested surface patch.
    pub fn bbox_over(&self, u: Interval, v: Interval) -> Option<BBox> {
        match self {
            Surface::Plane(surface) => surface.bbox_over(u, v),
            Surface::Cylinder(surface) => surface.bbox_over(u, v),
            Surface::Sphere(surface) => surface.bbox_over(u, v),
            Surface::Cone(surface) => surface.bbox_over(u, v),
            Surface::Ruled(surface) => surface.bbox_over(u, v),
            Surface::Revolution(surface) => surface.bbox_over(u, v),
            Surface::Nurbs(surface) => surface.bbox_over(u, v),
        }
    }

    pub fn point_at(&self, u: f64, v: f64) -> Point3 {
        match self {
            Surface::Plane(surface) => SurfaceGeometry::point_at(surface, u, v),
            Surface::Cylinder(surface) => SurfaceGeometry::point_at(surface, u, v),
            Surface::Sphere(surface) => SurfaceGeometry::point_at(surface, u, v),
            Surface::Cone(surface) => SurfaceGeometry::point_at(surface, u, v),
            Surface::Ruled(surface) => SurfaceGeometry::point_at(surface, u, v),
            Surface::Revolution(surface) => SurfaceGeometry::point_at(surface, u, v),
            Surface::Nurbs(surface) => SurfaceGeometry::point_at(surface, u, v),
        }
    }

    pub fn closest_parameter(&self, point: Point3) -> Result<Point2, NurbsError> {
        match self {
            Surface::Plane(surface) => surface.closest_parameter(point),
            Surface::Cylinder(surface) => SurfaceGeometry::closest_parameter(surface, point),
            Surface::Sphere(surface) => SurfaceGeometry::closest_parameter(surface, point),
            Surface::Cone(surface) => SurfaceGeometry::closest_parameter(surface, point),
            Surface::Ruled(surface) => SurfaceGeometry::closest_parameter(surface, point),
            Surface::Revolution(surface) => surface.closest_parameter(point),
            Surface::Nurbs(surface) => SurfaceGeometry::closest_parameter(surface, point),
        }
    }

    pub fn normal_at(&self, u: f64, v: f64) -> UnitVector3<f64> {
        match self {
            Surface::Plane(surface) => SurfaceGeometry::normal_at(surface, u, v),
            Surface::Cylinder(surface) => SurfaceGeometry::normal_at(surface, u, v),
            Surface::Sphere(surface) => SurfaceGeometry::normal_at(surface, u, v),
            Surface::Cone(surface) => SurfaceGeometry::normal_at(surface, u, v),
            Surface::Ruled(surface) => SurfaceGeometry::normal_at(surface, u, v),
            Surface::Revolution(surface) => SurfaceGeometry::normal_at(surface, u, v),
            Surface::Nurbs(surface) => SurfaceGeometry::normal_at(surface, u, v),
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
        Ok(match self {
            Surface::Plane(surface) => Surface::Plane(surface.rotated(axis, angle)?),
            Surface::Cylinder(surface) => Surface::Cylinder(surface.rotated(axis, angle)?),
            Surface::Sphere(surface) => Surface::Sphere(surface.rotated(axis, angle)?),
            Surface::Cone(surface) => Surface::Cone(surface.rotated(axis, angle)?),
            Surface::Ruled(surface) => Surface::Ruled(surface.rotated(axis, angle)?),
            Surface::Revolution(surface) => Surface::Revolution(surface.rotated(axis, angle)?),
            Surface::Nurbs(surface) => Surface::Nurbs(surface.rotated(axis, angle)?),
        })
    }

    pub fn translated(&self, direction: Vector3<f64>) -> Result<Self, NurbsError> {
        Ok(match self {
            Surface::Plane(surface) => Surface::Plane(surface.translated(direction)?),
            Surface::Cylinder(surface) => Surface::Cylinder(surface.translated(direction)?),
            Surface::Sphere(surface) => Surface::Sphere(surface.translated(direction)?),
            Surface::Cone(surface) => Surface::Cone(surface.translated(direction)?),
            Surface::Ruled(surface) => Surface::Ruled(surface.translated(direction)?),
            Surface::Revolution(surface) => Surface::Revolution(surface.translated(direction)?),
            Surface::Nurbs(surface) => Surface::Nurbs(surface.translated(direction)?),
        })
    }
}

/// Forwards to whichever variant the surface holds.
///
/// The inherent methods on [`Surface`] shadow these, so call sites keep working
/// without importing the trait; the impl exists so generic code can be written
/// once over any surface.
impl SurfaceGeometry for Surface {
    fn domain(&self) -> (Interval, Interval) {
        Surface::domain(self)
    }

    fn periodicity(&self) -> SurfacePeriodicity {
        Surface::periodicity(self)
    }

    fn point_at(&self, u: f64, v: f64) -> Point3 {
        Surface::point_at(self, u, v)
    }

    fn normal_at(&self, u: f64, v: f64) -> UnitVector3<f64> {
        Surface::normal_at(self, u, v)
    }

    fn is_degenerate_at(&self, u: f64, v: f64) -> bool {
        Surface::is_degenerate_at(self, u, v)
    }

    fn closest_parameter(&self, point: Point3) -> Result<Point2, NurbsError> {
        Surface::closest_parameter(self, point)
    }

    fn to_nurbs(&self) -> Result<NurbsSurface, NurbsError> {
        Surface::to_nurbs(self)
    }

    fn to_nurbs_over(&self, u: Interval, v: Interval) -> Result<NurbsSurface, NurbsError> {
        Surface::to_nurbs_over(self, u, v)
    }

    fn param_map_over(&self, u: Interval, v: Interval) -> ParamMap {
        Surface::param_map_over(self, u, v)
    }

    fn bbox_over(&self, u: Interval, v: Interval) -> Option<BBox> {
        Surface::bbox_over(self, u, v)
    }

    fn rotated(&self, axis: Axis3, angle: f64) -> Result<Self, NurbsError> {
        Surface::rotated(self, axis, angle)
    }

    fn translated(&self, direction: Vector3<f64>) -> Result<Self, NurbsError> {
        Surface::translated(self, direction)
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
    pub fn from_frame(frame: Frame) -> Self {
        Self { frame }
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

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
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

    /// Converts to NURBS realized over the requested parameter box.
    ///
    /// `point_at` moves `v` units along the axis, so the patch has to span the
    /// requested height rather than the unit interval [`Cylinder::to_nurbs`]
    /// uses; a taller face would otherwise lose everything above `v = 1`. The
    /// angular direction keeps the rational quadratic's projective
    /// parameterization, and only its knot range follows `u`.
    pub fn to_nurbs_over(&self, u: Interval, v: Interval) -> Result<NurbsSurface, NurbsError> {
        let circle = super::curves::Circle::new(
            Plane::new(self.origin(), self.x_dir(), self.axis()),
            self.radius,
        );
        let arc = if u.start.abs() <= LINEAR_TOLERANCE
            && (u.end - std::f64::consts::TAU).abs() <= LINEAR_TOLERANCE
        {
            circle.to_nurbs()?
        } else {
            circle.to_nurbs_between(u.start, u.end)?
        };

        let nu = arc.control_points().len();
        let mut points = Vec::with_capacity(2 * nu);
        for height in [v.start, v.end] {
            for point in arc.control_points().iter() {
                points.push(HPoint::from_cartesian(
                    point.to_cartesian() + height * *self.axis(),
                    point.weight(),
                ));
            }
        }

        NurbsSurface::new(
            arc.degree(),
            Degree::new(1)?,
            ControlNet::new(points, nu, 2)?,
            arc.knots().clone(),
            linear_knots(v)?,
        )
    }
}

/// A sphere parameterized by longitude `u` and latitude `v` in a local frame.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Sphere {
    frame: Frame,
    radius: f64,
}

impl Sphere {
    /// Creates a sphere centered at the frame origin.
    pub fn new(frame: Frame, radius: f64) -> Self {
        Self { frame, radius }
    }

    pub fn frame(&self) -> &Frame {
        &self.frame
    }

    pub fn radius(&self) -> f64 {
        self.radius
    }

    /// Evaluates the longitude/latitude parameterization.
    pub fn point_at(&self, u: f64, v: f64) -> Point3 {
        let radial = v.cos();
        self.frame.origin
            + *self.frame.x_dir * (self.radius * radial * u.cos())
            + *self.frame.y_dir * (self.radius * radial * u.sin())
            + *self.frame.z_dir * (self.radius * v.sin())
    }

    /// Returns the outward normal, including the meridian limit at each pole.
    pub fn normal_at(&self, u: f64, v: f64) -> UnitVector3<f64> {
        let direction = *self.frame.x_dir * (v.cos() * u.cos())
            + *self.frame.y_dir * (v.cos() * u.sin())
            + *self.frame.z_dir * v.sin();
        UnitVector3::new_normalize(direction)
    }

    /// Recovers longitude and latitude of the nearest radial projection.
    ///
    /// Longitude is undefined at the center and poles; those cases use zero.
    pub fn closest_parameter(&self, point: Point3) -> Point2 {
        let local = self.frame.coordinates_of(point);
        let radial = local.x.hypot(local.y);
        let u = if radial <= LINEAR_TOLERANCE {
            0.0
        } else {
            local.y.atan2(local.x).rem_euclid(std::f64::consts::TAU)
        };
        let v = if radial <= LINEAR_TOLERANCE && local.z.abs() <= LINEAR_TOLERANCE {
            0.0
        } else {
            local.z.atan2(radial)
        };
        Point2::new(u, v)
    }

    /// Converts the complete sphere to an exact rational biquadratic surface.
    pub fn to_nurbs(&self) -> Result<NurbsSurface, NurbsError> {
        self.to_nurbs_over(
            Interval::new(0.0, std::f64::consts::TAU),
            Interval::new(-std::f64::consts::FRAC_PI_2, std::f64::consts::FRAC_PI_2),
        )
    }

    /// Converts a longitude/latitude box to an exact rational NURBS patch.
    pub fn to_nurbs_over(&self, u: Interval, v: Interval) -> Result<NurbsSurface, NurbsError> {
        let unit_circle = Circle::new(Plane::xy(), 1.0).to_nurbs_between(u.start, u.end)?;
        let meridian = Circle::new(Plane::xz(), self.radius).to_nurbs_between(v.start, v.end)?;
        let nu = unit_circle.control_points().len();
        let nv = meridian.control_points().len();
        let mut points = Vec::with_capacity(nu * nv);

        for meridian_point in meridian.control_points().iter() {
            let meridian_cartesian = meridian_point.to_cartesian();
            let radial = meridian_cartesian.x;
            let height = meridian_cartesian.z;
            for longitude_point in unit_circle.control_points().iter() {
                let longitude = longitude_point.to_cartesian();
                let point = self.frame.origin
                    + *self.frame.x_dir * (radial * longitude.x)
                    + *self.frame.y_dir * (radial * longitude.y)
                    + *self.frame.z_dir * height;
                points.push(HPoint::from_cartesian(
                    point,
                    meridian_point.weight() * longitude_point.weight(),
                ));
            }
        }

        NurbsSurface::new(
            unit_circle.degree(),
            meridian.degree(),
            ControlNet::new(points, nu, nv)?,
            unit_circle.knots().clone(),
            meridian.knots().clone(),
        )
    }
}

/// A cone parameterized by longitude `u` and signed generatrix distance `v`.
///
/// The frame origin lies on the `v = 0` reference circle. Extending `v`
/// through the apex continues onto the opposite nappe.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Cone {
    frame: Frame,
    reference_radius: f64,
    half_angle: f64,
}

impl Cone {
    pub fn new(frame: Frame, reference_radius: f64, half_angle: f64) -> Self {
        Self {
            frame,
            reference_radius,
            half_angle,
        }
    }

    pub fn frame(&self) -> &Frame {
        &self.frame
    }

    pub fn reference_radius(&self) -> f64 {
        self.reference_radius
    }

    pub fn half_angle(&self) -> f64 {
        self.half_angle
    }

    pub fn radius_at(&self, v: f64) -> f64 {
        self.reference_radius + v * self.half_angle.sin()
    }

    pub fn apex_parameter(&self) -> Option<f64> {
        let slope = self.half_angle.sin();
        (slope.abs() > LINEAR_TOLERANCE).then_some(-self.reference_radius / slope)
    }

    pub fn point_at(&self, u: f64, v: f64) -> Point3 {
        let radius = self.radius_at(v);
        self.frame.origin
            + *self.frame.x_dir * (radius * u.cos())
            + *self.frame.y_dir * (radius * u.sin())
            + *self.frame.z_dir * (v * self.half_angle.cos())
    }

    /// Returns the oriented meridian-limit normal at the apex.
    pub fn normal_at(&self, u: f64, v: f64) -> UnitVector3<f64> {
        let direction = *self.frame.x_dir * (self.half_angle.cos() * u.cos())
            + *self.frame.y_dir * (self.half_angle.cos() * u.sin())
            - *self.frame.z_dir * self.half_angle.sin();
        if self.radius_at(v) < -LINEAR_TOLERANCE {
            UnitVector3::new_normalize(-direction)
        } else {
            UnitVector3::new_normalize(direction)
        }
    }

    /// Finds the nearest parameter on either nappe of the cone.
    pub fn closest_parameter(&self, point: Point3) -> Point2 {
        let local = self.frame.coordinates_of(point);
        let radial_distance = local.x.hypot(local.y);
        let observed_angle = if radial_distance <= LINEAR_TOLERANCE {
            0.0
        } else {
            local.y.atan2(local.x).rem_euclid(std::f64::consts::TAU)
        };
        let sin_angle = self.half_angle.sin();
        let cos_angle = self.half_angle.cos();
        let candidate = |signed_radius: f64| {
            let v = (signed_radius - self.reference_radius) * sin_angle + local.z * cos_angle;
            let error =
                (self.radius_at(v) - signed_radius).powi(2) + (v * cos_angle - local.z).powi(2);
            (v, error)
        };
        let positive = candidate(radial_distance);
        let negative = candidate(-radial_distance);
        let (mut u, v) = if negative.1 < positive.1 {
            (
                (observed_angle + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU),
                negative.0,
            )
        } else {
            (observed_angle, positive.0)
        };
        if self.radius_at(v).abs() <= LINEAR_TOLERANCE {
            u = 0.0;
        }
        Point2::new(u, v)
    }

    pub fn to_nurbs(&self) -> Result<NurbsSurface, NurbsError> {
        self.to_nurbs_over(
            Interval::new(0.0, std::f64::consts::TAU),
            Interval::new(0.0, 1.0),
        )
    }

    pub fn to_nurbs_over(&self, u: Interval, v: Interval) -> Result<NurbsSurface, NurbsError> {
        let arc = Circle::new(Plane::xy(), 1.0).to_nurbs_between(u.start, u.end)?;
        let nu = arc.control_points().len();
        let mut points = Vec::with_capacity(2 * nu);
        for parameter_v in [v.start, v.end] {
            let radius = self.radius_at(parameter_v);
            let height = parameter_v * self.half_angle.cos();
            for arc_point in arc.control_points().iter() {
                let longitude = arc_point.to_cartesian();
                let point = self.frame.origin
                    + *self.frame.x_dir * (radius * longitude.x)
                    + *self.frame.y_dir * (radius * longitude.y)
                    + *self.frame.z_dir * height;
                points.push(HPoint::from_cartesian(point, arc_point.weight()));
            }
        }
        NurbsSurface::new(
            arc.degree(),
            Degree::new(1)?,
            ControlNet::new(points, nu, 2)?,
            arc.knots().clone(),
            linear_knots(v)?,
        )
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
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

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
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

impl SurfaceGeometry for Plane {
    fn domain(&self) -> (Interval, Interval) {
        (Interval::unbounded(), Interval::unbounded())
    }

    fn periodicity(&self) -> SurfacePeriodicity {
        SurfacePeriodicity::None
    }

    fn point_at(&self, u: f64, v: f64) -> Point3 {
        Plane::point_at(self, u, v)
    }

    fn normal_at(&self, _u: f64, _v: f64) -> UnitVector3<f64> {
        self.normal()
    }

    fn is_degenerate_at(&self, _u: f64, _v: f64) -> bool {
        false
    }

    fn closest_parameter(&self, point: Point3) -> Result<Point2, NurbsError> {
        Ok(self.parameter_at(point))
    }

    fn to_nurbs(&self) -> Result<NurbsSurface, NurbsError> {
        Plane::to_nurbs(self)
    }

    fn to_nurbs_over(&self, u: Interval, v: Interval) -> Result<NurbsSurface, NurbsError> {
        Plane::to_nurbs_over(self, u, v)
    }

    fn param_map_over(&self, _u: Interval, _v: Interval) -> ParamMap {
        ParamMap::identity()
    }

    fn bbox_over(&self, u: Interval, v: Interval) -> Option<BBox> {
        if !u.is_finite() || !v.is_finite() {
            return None;
        }
        Some(BBox::from_points_in_frame(
            self.frame.clone(),
            [
                self.point_at(u.start, v.start),
                self.point_at(u.end, v.start),
                self.point_at(u.start, v.end),
                self.point_at(u.end, v.end),
            ],
        ))
    }

    fn rotated(&self, axis: Axis3, angle: f64) -> Result<Self, NurbsError> {
        let rotation = Rotation3::from_axis_angle(&axis.direction, angle);
        Ok(Plane::from_xy(
            axis.origin + rotation * (self.origin() - axis.origin),
            rotation * *self.x_dir(),
            rotation * *self.y_dir(),
        ))
    }

    fn translated(&self, direction: Vector3<f64>) -> Result<Self, NurbsError> {
        Ok(Plane::from_xy(
            self.origin() + direction,
            self.x_dir(),
            self.y_dir(),
        ))
    }
}

impl SurfaceGeometry for Cylinder {
    fn domain(&self) -> (Interval, Interval) {
        (
            Interval::new(0.0, std::f64::consts::TAU),
            Interval::unbounded(),
        )
    }

    fn periodicity(&self) -> SurfacePeriodicity {
        SurfacePeriodicity::UPeriodic(std::f64::consts::TAU)
    }

    fn point_at(&self, u: f64, v: f64) -> Point3 {
        Cylinder::point_at(self, u, v)
    }

    fn normal_at(&self, u: f64, v: f64) -> UnitVector3<f64> {
        Cylinder::normal_at(self, u, v)
    }

    fn is_degenerate_at(&self, _u: f64, _v: f64) -> bool {
        false
    }

    fn closest_parameter(&self, point: Point3) -> Result<Point2, NurbsError> {
        Ok(Cylinder::closest_parameter(self, point))
    }

    fn to_nurbs(&self) -> Result<NurbsSurface, NurbsError> {
        Cylinder::to_nurbs(self)
    }

    fn to_nurbs_over(&self, u: Interval, v: Interval) -> Result<NurbsSurface, NurbsError> {
        Cylinder::to_nurbs_over(self, u, v)
    }

    fn param_map_over(&self, u: Interval, _v: Interval) -> ParamMap {
        ParamMap {
            u: Reparam::conic_arc(u, u),
            v: Reparam::Identity,
        }
    }

    fn bbox_over(&self, u: Interval, v: Interval) -> Option<BBox> {
        if !u.is_finite() || !v.is_finite() {
            return None;
        }
        let ordered = u.ordered();
        let mut angles = vec![ordered.start, ordered.end];
        let first = (ordered.start / std::f64::consts::FRAC_PI_2).ceil() as i64;
        let last = (ordered.end / std::f64::consts::FRAC_PI_2).floor() as i64;
        angles.extend((first..=last).map(|index| index as f64 * std::f64::consts::FRAC_PI_2));
        Some(BBox::from_points_in_frame(
            self.frame.clone(),
            [v.start, v.end].into_iter().flat_map(|height| {
                angles
                    .iter()
                    .map(move |&angle| self.point_at(angle, height))
            }),
        ))
    }

    fn rotated(&self, axis: Axis3, angle: f64) -> Result<Self, NurbsError> {
        let rotation = Rotation3::from_axis_angle(&axis.direction, angle);
        Ok(Cylinder::new(
            axis.origin + rotation * (self.origin() - axis.origin),
            rotation * *self.x_dir(),
            rotation * *self.axis(),
            self.radius,
        ))
    }

    fn translated(&self, direction: Vector3<f64>) -> Result<Self, NurbsError> {
        Ok(Cylinder::new(
            self.origin() + direction,
            self.x_dir(),
            self.axis(),
            self.radius,
        ))
    }
}

impl SurfaceGeometry for Sphere {
    fn domain(&self) -> (Interval, Interval) {
        (
            Interval::new(0.0, std::f64::consts::TAU),
            Interval::new(-std::f64::consts::FRAC_PI_2, std::f64::consts::FRAC_PI_2),
        )
    }

    fn periodicity(&self) -> SurfacePeriodicity {
        SurfacePeriodicity::UPeriodic(std::f64::consts::TAU)
    }

    fn point_at(&self, u: f64, v: f64) -> Point3 {
        Sphere::point_at(self, u, v)
    }

    fn normal_at(&self, u: f64, v: f64) -> UnitVector3<f64> {
        Sphere::normal_at(self, u, v)
    }

    fn is_degenerate_at(&self, _u: f64, v: f64) -> bool {
        self.radius * v.cos().abs() <= LINEAR_TOLERANCE
    }

    fn closest_parameter(&self, point: Point3) -> Result<Point2, NurbsError> {
        Ok(Sphere::closest_parameter(self, point))
    }

    fn to_nurbs(&self) -> Result<NurbsSurface, NurbsError> {
        Sphere::to_nurbs(self)
    }

    fn to_nurbs_over(&self, u: Interval, v: Interval) -> Result<NurbsSurface, NurbsError> {
        Sphere::to_nurbs_over(self, u, v)
    }

    fn param_map_over(&self, u: Interval, v: Interval) -> ParamMap {
        ParamMap {
            u: Reparam::conic_arc(u, u),
            v: Reparam::conic_arc(v, v),
        }
    }

    fn bbox_over(&self, u: Interval, v: Interval) -> Option<BBox> {
        if !u.is_finite() || !v.is_finite() {
            return None;
        }
        let mut longitudes = angular_extrema(u, std::f64::consts::FRAC_PI_2);
        let mut latitudes = angular_extrema(v, std::f64::consts::FRAC_PI_2);
        longitudes.extend([u.start, u.end]);
        latitudes.extend([v.start, v.end]);
        Some(BBox::from_points_in_frame(
            self.frame.clone(),
            latitudes.into_iter().flat_map(|latitude| {
                longitudes
                    .iter()
                    .map(move |&longitude| self.point_at(longitude, latitude))
            }),
        ))
    }

    fn rotated(&self, axis: Axis3, angle: f64) -> Result<Self, NurbsError> {
        let rotation = Rotation3::from_axis_angle(&axis.direction, angle);
        Ok(Sphere::new(
            Frame::from_xy(
                axis.origin + rotation * (self.frame.origin - axis.origin),
                rotation * *self.frame.x_dir,
                rotation * *self.frame.y_dir,
            ),
            self.radius,
        ))
    }

    fn translated(&self, direction: Vector3<f64>) -> Result<Self, NurbsError> {
        Ok(Sphere::new(
            Frame::from_xy(
                self.frame.origin + direction,
                self.frame.x_dir,
                self.frame.y_dir,
            ),
            self.radius,
        ))
    }
}

impl SurfaceGeometry for Cone {
    fn domain(&self) -> (Interval, Interval) {
        (
            Interval::new(0.0, std::f64::consts::TAU),
            Interval::unbounded(),
        )
    }

    fn periodicity(&self) -> SurfacePeriodicity {
        SurfacePeriodicity::UPeriodic(std::f64::consts::TAU)
    }

    fn point_at(&self, u: f64, v: f64) -> Point3 {
        Cone::point_at(self, u, v)
    }

    fn normal_at(&self, u: f64, v: f64) -> UnitVector3<f64> {
        Cone::normal_at(self, u, v)
    }

    fn is_degenerate_at(&self, _u: f64, v: f64) -> bool {
        self.radius_at(v).abs() <= LINEAR_TOLERANCE
    }

    fn closest_parameter(&self, point: Point3) -> Result<Point2, NurbsError> {
        Ok(Cone::closest_parameter(self, point))
    }

    fn to_nurbs(&self) -> Result<NurbsSurface, NurbsError> {
        Cone::to_nurbs(self)
    }

    fn to_nurbs_over(&self, u: Interval, v: Interval) -> Result<NurbsSurface, NurbsError> {
        Cone::to_nurbs_over(self, u, v)
    }

    fn param_map_over(&self, u: Interval, _v: Interval) -> ParamMap {
        ParamMap {
            u: Reparam::conic_arc(u, u),
            v: Reparam::Identity,
        }
    }

    fn bbox_over(&self, u: Interval, v: Interval) -> Option<BBox> {
        if !u.is_finite() || !v.is_finite() {
            return None;
        }
        let mut angles = angular_extrema(u, std::f64::consts::FRAC_PI_2);
        angles.extend([u.start, u.end]);
        Some(BBox::from_points_in_frame(
            self.frame.clone(),
            [v.start, v.end].into_iter().flat_map(|parameter_v| {
                angles
                    .iter()
                    .map(move |&parameter_u| self.point_at(parameter_u, parameter_v))
            }),
        ))
    }

    fn rotated(&self, axis: Axis3, angle: f64) -> Result<Self, NurbsError> {
        let rotation = Rotation3::from_axis_angle(&axis.direction, angle);
        Ok(Cone::new(
            Frame::from_xy(
                axis.origin + rotation * (self.frame.origin - axis.origin),
                rotation * *self.frame.x_dir,
                rotation * *self.frame.y_dir,
            ),
            self.reference_radius,
            self.half_angle,
        ))
    }

    fn translated(&self, direction: Vector3<f64>) -> Result<Self, NurbsError> {
        Ok(Cone::new(
            Frame::from_xy(
                self.frame.origin + direction,
                self.frame.x_dir,
                self.frame.y_dir,
            ),
            self.reference_radius,
            self.half_angle,
        ))
    }
}

impl SurfaceGeometry for RuledSurface {
    fn domain(&self) -> (Interval, Interval) {
        (self.curve.domain(), Interval::new(0.0, 1.0))
    }

    fn periodicity(&self) -> SurfacePeriodicity {
        match self.curve.periodicity() {
            Periodicity::None => SurfacePeriodicity::None,
            Periodicity::Periodic(period) => SurfacePeriodicity::UPeriodic(period),
        }
    }

    fn point_at(&self, u: f64, v: f64) -> Point3 {
        RuledSurface::point_at(self, u, v)
    }

    fn normal_at(&self, u: f64, v: f64) -> UnitVector3<f64> {
        RuledSurface::normal_at(self, u, v)
    }

    fn is_degenerate_at(&self, _u: f64, _v: f64) -> bool {
        false
    }

    fn closest_parameter(&self, point: Point3) -> Result<Point2, NurbsError> {
        Ok(RuledSurface::closest_parameter(self, point))
    }

    fn to_nurbs(&self) -> Result<NurbsSurface, NurbsError> {
        RuledSurface::to_nurbs(self)
    }

    /// The ruling already carries a finite `v` parameterization over `[0, 1]`,
    /// so the requested box adds nothing here.
    fn to_nurbs_over(&self, _u: Interval, _v: Interval) -> Result<NurbsSurface, NurbsError> {
        RuledSurface::to_nurbs(self)
    }

    fn param_map_over(&self, _u: Interval, _v: Interval) -> ParamMap {
        ParamMap {
            u: self.curve.nurbs_param_map(),
            v: Reparam::Identity,
        }
    }

    fn bbox_over(&self, u: Interval, v: Interval) -> Option<BBox> {
        if !u.is_finite() || !v.is_finite() {
            return None;
        }
        let curve_bounds = self.curve.bbox_over(u)?;
        let corners = curve_bounds.corners()?;
        Some(BBox::from_points([v.start, v.end].into_iter().flat_map(
            |parameter| {
                corners
                    .iter()
                    .map(move |point| *point + self.direction * parameter)
            },
        )))
    }

    fn rotated(&self, axis: Axis3, angle: f64) -> Result<Self, NurbsError> {
        let rotation = Rotation3::from_axis_angle(&axis.direction, angle);
        Ok(RuledSurface::new(
            self.curve.rotated(axis, angle)?,
            rotation * self.direction,
        ))
    }

    fn translated(&self, direction: Vector3<f64>) -> Result<Self, NurbsError> {
        Ok(RuledSurface::new(
            self.curve.translated(direction)?,
            self.direction,
        ))
    }
}

impl SurfaceGeometry for SurfaceOfRevolution {
    fn domain(&self) -> (Interval, Interval) {
        (
            self.curve.domain(),
            Interval::new(0.0, std::f64::consts::TAU),
        )
    }

    fn periodicity(&self) -> SurfacePeriodicity {
        SurfacePeriodicity::VPeriodic(std::f64::consts::TAU)
    }

    fn point_at(&self, u: f64, v: f64) -> Point3 {
        SurfaceOfRevolution::point_at(self, u, v)
    }

    fn normal_at(&self, u: f64, v: f64) -> UnitVector3<f64> {
        SurfaceOfRevolution::normal_at(self, u, v)
    }

    /// The parameterization collapses wherever the profile curve meets the
    /// axis: every `v` maps to the same point there.
    fn is_degenerate_at(&self, u: f64, v: f64) -> bool {
        let point = self.point_at(u, v);
        (point - self.axis.project(point)).norm() <= LINEAR_TOLERANCE
    }

    fn closest_parameter(&self, point: Point3) -> Result<Point2, NurbsError> {
        Ok(self.to_nurbs()?.closest_parameter(point))
    }

    fn to_nurbs(&self) -> Result<NurbsSurface, NurbsError> {
        SurfaceOfRevolution::to_nurbs(self)
    }

    /// The revolution already spans a full turn in `v`, and its profile's own
    /// domain in `u`, so the requested box adds nothing here.
    fn to_nurbs_over(&self, _u: Interval, _v: Interval) -> Result<NurbsSurface, NurbsError> {
        SurfaceOfRevolution::to_nurbs(self)
    }

    fn param_map_over(&self, _u: Interval, _v: Interval) -> ParamMap {
        ParamMap {
            u: self.curve.nurbs_param_map(),
            v: Reparam::conic_arc(
                Interval::new(0.0, std::f64::consts::TAU),
                Interval::new(0.0, std::f64::consts::TAU),
            ),
        }
    }

    fn bbox_over(&self, _u: Interval, _v: Interval) -> Option<BBox> {
        positive_surface_control_bounds(&self.to_nurbs().ok()?)
    }

    fn rotated(&self, axis: Axis3, angle: f64) -> Result<Self, NurbsError> {
        let rotation = Rotation3::from_axis_angle(&axis.direction, angle);
        Ok(SurfaceOfRevolution::new(
            self.curve.rotated(axis, angle)?,
            Axis3::new(
                axis.origin + rotation * (self.axis.origin - axis.origin),
                rotation * *self.axis.direction,
            ),
        ))
    }

    fn translated(&self, direction: Vector3<f64>) -> Result<Self, NurbsError> {
        Ok(SurfaceOfRevolution::new(
            self.curve.translated(direction)?,
            self.axis,
        ))
    }
}

impl SurfaceGeometry for NurbsSurface {
    fn domain(&self) -> (Interval, Interval) {
        (self.domain_u(), self.domain_v())
    }

    fn periodicity(&self) -> SurfacePeriodicity {
        SurfacePeriodicity::None
    }

    fn point_at(&self, u: f64, v: f64) -> Point3 {
        NurbsSurface::point_at(self, u, v)
    }

    fn normal_at(&self, u: f64, v: f64) -> UnitVector3<f64> {
        NurbsSurface::normal_at(self, u, v)
    }

    fn is_degenerate_at(&self, _u: f64, _v: f64) -> bool {
        false
    }

    fn closest_parameter(&self, point: Point3) -> Result<Point2, NurbsError> {
        Ok(NurbsSurface::closest_parameter(self, point))
    }

    fn to_nurbs(&self) -> Result<NurbsSurface, NurbsError> {
        Ok(self.clone())
    }

    /// A NURBS patch carries its own finite parameterization, so the requested
    /// box adds nothing here.
    fn to_nurbs_over(&self, _u: Interval, _v: Interval) -> Result<NurbsSurface, NurbsError> {
        Ok(self.clone())
    }

    fn param_map_over(&self, _u: Interval, _v: Interval) -> ParamMap {
        ParamMap::identity()
    }

    fn bbox_over(&self, _u: Interval, _v: Interval) -> Option<BBox> {
        positive_surface_control_bounds(self)
    }

    fn rotated(&self, axis: Axis3, angle: f64) -> Result<Self, NurbsError> {
        let rotation = Rotation3::from_axis_angle(&axis.direction, angle);
        let points = self
            .control_points()
            .as_slice()
            .iter()
            .map(|point| {
                let rotated = axis.origin + rotation * (point.to_cartesian() - axis.origin);
                HPoint::from_cartesian(rotated, point.weight())
            })
            .collect();
        let control_points = ControlNet::new(
            points,
            self.control_points().nu(),
            self.control_points().nv(),
        )?;
        NurbsSurface::new(
            self.degree_u(),
            self.degree_v(),
            control_points,
            self.knots_u().clone(),
            self.knots_v().clone(),
        )
    }

    fn translated(&self, direction: Vector3<f64>) -> Result<Self, NurbsError> {
        let points = self
            .control_points()
            .as_slice()
            .iter()
            .map(|point| HPoint::from_cartesian(point.to_cartesian() + direction, point.weight()))
            .collect();
        let control_points = ControlNet::new(
            points,
            self.control_points().nu(),
            self.control_points().nv(),
        )?;
        NurbsSurface::new(
            self.degree_u(),
            self.degree_v(),
            control_points,
            self.knots_u().clone(),
            self.knots_v().clone(),
        )
    }
}

/// Bounds a rational surface by its control hull when the convex-hull
/// precondition (finite positive weights) is satisfied.
fn positive_surface_control_bounds(surface: &NurbsSurface) -> Option<BBox> {
    surface
        .control_points()
        .as_slice()
        .iter()
        .all(|point| point.weight().is_finite() && point.weight() > 0.0)
        .then(|| {
            BBox::from_points(
                surface
                    .control_points()
                    .as_slice()
                    .iter()
                    .map(|point| point.to_cartesian()),
            )
        })
}

/// Interior multiples of `step` inside an angular interval.
fn angular_extrema(interval: Interval, step: f64) -> Vec<f64> {
    let ordered = interval.ordered();
    let first = (ordered.start / step).ceil() as i64;
    let last = (ordered.end / step).floor() as i64;
    (first..=last).map(|index| index as f64 * step).collect()
}
