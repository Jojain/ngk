use crate::solver::numerical::Jet;
use nalgebra::{DVector, Vector3};
use std::ops::{Add, Sub};

/// A space vector whose coordinates are [`Jet`]s, so it carries its derivatives with
/// respect to the unknowns.
#[derive(Clone, Debug)]
pub(super) struct JetVector3 {
    pub x: Jet,
    pub y: Jet,
    pub z: Jet,
}

impl JetVector3 {
    pub fn constant(v: Vector3<f64>) -> Self {
        Self {
            x: Jet::constant(v.x),
            y: Jet::constant(v.y),
            z: Jet::constant(v.z),
        }
    }
    /// The three unknowns starting at `column`.
    pub fn variables(x: &DVector<f64>, column: usize) -> Self {
        Self {
            x: Jet::variable(x[column], column),
            y: Jet::variable(x[column + 1], column + 1),
            z: Jet::variable(x[column + 2], column + 2),
        }
    }
    pub fn dot(&self, b: &Self) -> Jet {
        self.x.clone() * b.x.clone() + self.y.clone() * b.y.clone() + self.z.clone() * b.z.clone()
    }
    pub fn cross(&self, b: &Self) -> Self {
        Self {
            x: self.y.clone() * b.z.clone() - self.z.clone() * b.y.clone(),
            y: self.z.clone() * b.x.clone() - self.x.clone() * b.z.clone(),
            z: self.x.clone() * b.y.clone() - self.y.clone() * b.x.clone(),
        }
    }
    pub fn scale(&self, s: &Jet) -> Self {
        Self {
            x: self.x.clone() * s.clone(),
            y: self.y.clone() * s.clone(),
            z: self.z.clone() * s.clone(),
        }
    }
    pub fn components(self) -> [Jet; 3] {
        [self.x, self.y, self.z]
    }
}

impl Add for JetVector3 {
    type Output = Self;
    fn add(self, b: Self) -> Self {
        Self {
            x: self.x + b.x,
            y: self.y + b.y,
            z: self.z + b.z,
        }
    }
}

impl Sub for JetVector3 {
    type Output = Self;
    fn sub(self, b: Self) -> Self {
        Self {
            x: self.x - b.x,
            y: self.y - b.y,
            z: self.z - b.z,
        }
    }
}

/// Below this squared angle the rotation's coefficients use their Taylor series, which
/// stay smooth at zero where the closed forms divide by the angle.
const SMALL_ANGLE_SQUARED: f64 = 1e-8;

/// `v` turned by the rotation vector `omega`: about its direction, by its length.
///
/// Rodrigues' formula, `v + a (ω × v) + b ω × (ω × v)` with `a = sin θ / θ` and
/// `b = (1 - cos θ) / θ²`.
pub(super) fn rotate(omega: &JetVector3, v: &JetVector3) -> JetVector3 {
    let squared = omega.dot(omega);
    let (a, b) = if squared.value < SMALL_ANGLE_SQUARED {
        (
            -(squared.clone() * (1. / 6.)) + 1.,
            -(squared * (1. / 24.)) + 0.5,
        )
    } else {
        let angle = squared.clone().sqrt();
        let a = angle.clone().sin() / angle.clone();
        let b = (-angle.cos() + 1.) / squared;
        (a, b)
    };
    let turned = omega.cross(v);
    let twice = omega.cross(&turned);
    v.clone() + turned.scale(&a) + twice.scale(&b)
}
