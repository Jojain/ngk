use crate::solver::numerical::Jet;
use std::ops::{Add, Sub};

/// A plane vector whose coordinates are [`Jet`]s, so it carries its
/// derivatives with respect to the unknowns.
#[derive(Clone, Debug)]
pub(super) struct JetVector2 {
    pub x: Jet,
    pub y: Jet,
}
impl JetVector2 {
    pub fn dot(self, b: Self) -> Jet {
        self.x * b.x + self.y * b.y
    }
    pub fn cross(self, b: Self) -> Jet {
        self.x * b.y - self.y * b.x
    }
    pub fn norm(self) -> Jet {
        self.x.hypot(self.y)
    }
    pub fn unit(self) -> Self {
        let norm = self.clone().norm();
        Self {
            x: self.x / norm.clone(),
            y: self.y / norm,
        }
    }
    pub fn scale(self, s: Jet) -> Self {
        Self {
            x: self.x * s.clone(),
            y: self.y * s,
        }
    }
}
impl Add for JetVector2 {
    type Output = Self;
    fn add(self, b: Self) -> Self {
        Self {
            x: self.x + b.x,
            y: self.y + b.y,
        }
    }
}
impl Sub for JetVector2 {
    type Output = Self;
    fn sub(self, b: Self) -> Self {
        Self {
            x: self.x - b.x,
            y: self.y - b.y,
        }
    }
}
