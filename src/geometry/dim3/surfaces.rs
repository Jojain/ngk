use super::curves::Curve;
use super::frame::Frame;
use super::nurbs::{ControlNet, HPoint, NurbsError, NurbsSurface};
use super::utils::{IntoUnit3, Point3};
use crate::geometry::LINEAR_TOLERANCE;
use nalgebra::{Rotation3, UnitVector3, Vector3};

#[derive(Clone)]
pub enum Surface {
    Plane(Plane),
    Cylinder(Cylinder),
    Ruled(RuledSurface),
    Revolution(SurfaceOfRevolution),
    Nurbs(NurbsSurface),
}

impl Surface {
    pub fn point_at(&self, u: f64, v: f64) -> Point3 {
        match self {
            Surface::Plane(p) => p.point_at(u, v),
            Surface::Cylinder(c) => c.point_at(u, v),
            Surface::Ruled(s) => s.point_at(u, v),
            Surface::Revolution(s) => s.point_at(u, v),
            Surface::Nurbs(s) => s.point_at(u, v),
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
                surface.origin + direction,
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

#[derive(Clone)]
pub struct Plane {
    pub frame: Frame,
}

impl Plane {
    pub fn new(origin: Point3, x_dir: impl IntoUnit3, normal: impl IntoUnit3) -> Self {
        Self {
            frame: Frame::from_xz(origin, x_dir, normal),
        }
    }
    pub fn from_xy(origin: Point3, x_dir: impl IntoUnit3, y_dir: impl IntoUnit3) -> Self {
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
}

#[derive(Clone)]
pub struct Cylinder {
    pub frame: Frame,
    pub radius: f64,
}

impl Cylinder {
    pub fn new(origin: Point3, x_dir: impl IntoUnit3, axis: impl IntoUnit3, radius: f64) -> Self {
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

    pub fn origin(&self) -> Point3 {
        self.frame.origin
    }

    pub fn x_dir(&self) -> UnitVector3<f64> {
        self.frame.x_dir
    }

    pub fn axis(&self) -> UnitVector3<f64> {
        self.frame.z_dir
    }
}

#[derive(Clone)]
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

    pub fn normal_at(&self, u: f64, _v: f64) -> UnitVector3<f64> {
        let du = finite_difference_curve_tangent(&self.curve, u);
        let n = du.cross(&self.direction);
        match UnitVector3::try_new(n, LINEAR_TOLERANCE) {
            Some(n) => n,
            None => Vector3::z_axis(),
        }
    }
}

#[derive(Clone)]
pub struct SurfaceOfRevolution {
    curve: Curve,
    origin: Point3,
    axis: UnitVector3<f64>,
}

impl SurfaceOfRevolution {
    pub fn new(curve: Curve, origin: Point3, axis: impl IntoUnit3) -> Self {
        Self {
            curve,
            origin,
            axis: axis.normalized(),
        }
    }

    pub fn curve(&self) -> &Curve {
        &self.curve
    }

    pub fn origin(&self) -> Point3 {
        self.origin
    }

    pub fn axis(&self) -> UnitVector3<f64> {
        self.axis
    }

    pub fn point_at(&self, u: f64, v: f64) -> Point3 {
        // u walks the profile curve, v is the angle [0, 2π]
        let p = self.curve.point_at(u);

        // Project p onto the axis, then get the radial component
        let op = p - self.origin;
        let axial = op.dot(&self.axis) * *self.axis;
        let radial = op - axial;

        // Rotate the radial part by angle v around the axis
        let rot = Rotation3::from_axis_angle(&self.axis, v);
        self.origin + axial + rot * radial
    }

    pub fn normal_at(&self, u: f64, v: f64) -> UnitVector3<f64> {
        let du = self.partial_u(u, v);
        let dv = self.partial_v(u, v);
        let n = du.cross(&dv);
        UnitVector3::try_new(n, LINEAR_TOLERANCE).unwrap_or(Vector3::z_axis())
    }

    fn partial_u(&self, u: f64, v: f64) -> Vector3<f64> {
        let h = 1e-6;
        self.point_at(u + h, v) - self.point_at(u - h, v)
    }

    fn partial_v(&self, u: f64, v: f64) -> Vector3<f64> {
        // Analytical: dS/dv = rot(v) * radial_perp
        // But finite difference is consistent with your existing style
        let h = 1e-6;
        self.point_at(u, v + h) - self.point_at(u, v - h)
    }
}

fn finite_difference_curve_tangent(curve: &Curve, u: f64) -> Vector3<f64> {
    let h = 1.0e-6;
    let before = curve.point_at(u - h);
    let after = curve.point_at(u + h);
    after - before
}
