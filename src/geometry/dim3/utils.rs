use nalgebra::Point2 as NPoint2;
use nalgebra::Point3 as NPoint3;
use nalgebra::{UnitVector2, UnitVector3, Vector2, Vector3};

pub type Point2 = NPoint2<f64>;
pub type Point3 = NPoint3<f64>;

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
