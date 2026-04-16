use super::nurbs::NurbsCurve;
use super::surfaces::Plane;
use super::utils::{IntoUnit3, Point3};
use nalgebra::Rotation3;

#[derive(Clone)]
pub enum Curve {
    Line(Line),
    Circle(Circle),
    Nurbs(NurbsCurve),
}

impl Curve {
    pub fn point_at(&self, t: f64) -> Point3 {
        match self {
            Curve::Line(l) => l.point_at(t),
            Curve::Circle(c) => c.point_at(t),
            Curve::Nurbs(n) => n.point_at(t),
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

    pub fn point_at(&self, t: f64) -> Point3 {
        self.start + (self.end - self.start) * t
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
}
