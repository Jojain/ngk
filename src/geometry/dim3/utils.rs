use nalgebra::Point2 as NPoint2;
use nalgebra::Point3 as NPoint3;
use nalgebra::{UnitVector2, UnitVector3, Vector2, Vector3};

pub type Point2 = NPoint2<f64>;
pub type Point3 = NPoint3<f64>;

pub trait PointCoincidence<Rhs = Self> {
    fn coincides(&self, other: Rhs, tol: f64) -> bool;
}

impl PointCoincidence<Point2> for Point2 {
    fn coincides(&self, other: Point2, tol: f64) -> bool {
        (self - other).norm_squared() <= tol * tol
    }
}

impl PointCoincidence<&Point2> for Point2 {
    fn coincides(&self, other: &Point2, tol: f64) -> bool {
        self.coincides(*other, tol)
    }
}

impl PointCoincidence<Point3> for Point3 {
    fn coincides(&self, other: Point3, tol: f64) -> bool {
        (self - other).norm_squared() <= tol * tol
    }
}

impl PointCoincidence<&Point3> for Point3 {
    fn coincides(&self, other: &Point3, tol: f64) -> bool {
        self.coincides(*other, tol)
    }
}

pub trait IntoUnit2 {
    fn normalized(self) -> UnitVector2<f64>;
}

impl IntoUnit2 for Vector2<f64> {
    fn normalized(self) -> UnitVector2<f64> {
        UnitVector2::new_normalize(self)
    }
}

impl IntoUnit2 for UnitVector2<f64> {
    fn normalized(self) -> UnitVector2<f64> {
        self
    }
}

pub trait IntoUnit3 {
    fn normalized(self) -> UnitVector3<f64>;
}

impl IntoUnit3 for Vector3<f64> {
    fn normalized(self) -> UnitVector3<f64> {
        UnitVector3::new_normalize(self)
    }
}

impl IntoUnit3 for UnitVector3<f64> {
    fn normalized(self) -> UnitVector3<f64> {
        self
    }
}

#[cfg(test)]
mod tests {
    use super::{Point3, PointCoincidence};
    use crate::geometry::LINEAR_TOLERANCE;

    #[test]
    fn point_coincides_uses_tolerance() {
        let a = Point3::new(1.0, 2.0, 3.0);
        let b = Point3::new(1.0 + 1.0e-10, 2.0, 3.0);

        assert!(a.coincides(b, LINEAR_TOLERANCE));
        assert!(!a.coincides(b, 1.0e-12));
    }
}
