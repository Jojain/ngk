use super::frame::Frame;
use super::nurbs::NurbsSurface;
use super::utils::{IntoUnit3, Point3};
use nalgebra::{Rotation3, UnitVector3};

#[derive(Clone)]
pub enum Surface {
    Plane(Plane),
    Cylinder(Cylinder),
    Nurbs(NurbsSurface),
}

impl Surface {
    pub fn point_at(&self, u: f64, v: f64) -> Point3 {
        match self {
            Surface::Plane(p) => p.point_at(u, v),
            Surface::Cylinder(c) => c.point_at(u, v),
            Surface::Nurbs(s) => s.point_at(u, v),
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
