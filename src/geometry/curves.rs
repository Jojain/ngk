use std::f64::consts::TAU;

use super::nurbs::NurbsCurve;
use super::surfaces::Plane;
use super::utils::{IntoUnit3, Point3};
use nalgebra::{Rotation3, UnitVector3};

pub enum Periodicity{
    None,
    Periodic(f64),
}

#[derive(Clone)]
pub enum Curve {
    Line(Line),
    Circle(Circle),
    Nurbs(NurbsCurve),
}

impl Curve {
    pub fn periodicity(&self) -> Periodicity {
        match self {
            Curve::Line(l) => Periodicity::None,
            Curve::Circle(c) => Periodicity::Periodic(TAU),
            Curve::Nurbs(n) => unimplemented!(),
        }
    }
    pub fn point_at(&self, t: f64) -> Point3 {
        match self {
            Curve::Line(l) => l.point_at(t),
            Curve::Circle(c) => c.point_at(t),
            Curve::Nurbs(n) => n.point_at(t),
        }
    }
    pub fn param_at(&self, point: Point3) -> f64 {
        match self {
            Curve::Line(l) => l.param_at(point),
            Curve::Circle(c) => c.param_at(point),
            Curve::Nurbs(n) => unimplemented!(),
        }
    }
    pub fn length(&self, t0: f64, t1: f64) -> f64 {
        match self {
            Curve::Line(l) => l.length(t0, t1),
            Curve::Circle(c) => c.length(t0, t1),
            Curve::Nurbs(n) => unimplemented!(),
        }
    }
}

#[derive(Clone)]
pub struct Line {
    start: Point3,
    end: Point3,
}

impl Line {
    pub fn new(start: Point3, end: Point3) -> Self {
        Self { start, end }
    }
    pub fn from_direction(start: Point3, direction: impl IntoUnit3) -> Self {
        let direction = direction.normalized();
        Self {
            start,
            end: start + *direction,
        }
    }

    pub fn direction(&self) -> UnitVector3<f64> {
        (self.end - self.start).normalized()
    }

    pub fn point_at(&self, t: f64) -> Point3 {
        self.start + (self.end - self.start) * t
    }
    pub fn param_at(&self, point: Point3) -> f64 {
        (point - self.start).dot(&self.direction()) / self.direction().norm_squared()
    }
    pub fn length(&self, t0: f64, t1: f64) -> f64 {
        (t1 - t0).abs()
    }
}

#[derive(Clone)]
pub struct Circle {
    plane: Plane,
    radius: f64,
}

impl Circle {
    pub fn new(plane: Plane, radius: f64) -> Self {
        Self { plane, radius }
    }

    pub fn point_at(&self, t: f64) -> Point3 {
        let rot = Rotation3::from_axis_angle(&self.plane.normal, t);
        let vec = rot * self.plane.x_dir;
        self.plane.origin + self.radius * *vec
    }
    pub fn param_at(&self, point: Point3) -> f64 {
        let t =
            (point - self.plane.origin).dot(&self.plane.normal) / self.plane.normal.norm_squared();
        t.clamp(0.0, 1.0)
    }
    pub fn length(&self, t0: f64, t1: f64) -> f64 {
        if t0 == t1 {
            return TAU;
        } else {
            (t1 - t0).abs()
        }
    }
}
