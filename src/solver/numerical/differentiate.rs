use smallvec::SmallVec;
use std::ops::{Add, Div, Mul, Neg, Sub};

type Gradient = SmallVec<[(usize, f64); 12]>;

/// Sparse forward differentiation of a local geometric expression.
#[derive(Clone, Debug)]
pub(super) struct Jet {
    pub value: f64,
    pub gradient: Gradient,
}
impl Jet {
    pub fn constant(value: f64) -> Self {
        Self {
            value,
            gradient: Gradient::new(),
        }
    }
    pub fn variable(value: f64, index: usize) -> Self {
        Self {
            value,
            gradient: smallvec::smallvec![(index, 1.)],
        }
    }
    pub fn sin(self) -> Self {
        let (s, c) = self.value.sin_cos();
        Self {
            value: s,
            gradient: combine(&self.gradient, c, &[], 0.),
        }
    }
    pub fn cos(self) -> Self {
        let (s, c) = self.value.sin_cos();
        Self {
            value: c,
            gradient: combine(&self.gradient, -s, &[], 0.),
        }
    }
    /// Differentiates an oriented angle on its local branch.
    pub fn atan2(self, x: Self) -> Self {
        let denominator = self.value * self.value + x.value * x.value;
        Self {
            value: self.value.atan2(x.value),
            gradient: combine(
                &self.gradient,
                x.value / denominator,
                &x.gradient,
                -self.value / denominator,
            ),
        }
    }
    /// Chooses the positive x direction at coincident points, where norm is nonsmooth.
    pub fn hypot(self, y: Self) -> Self {
        let value = self.value.hypot(y.value);
        let (a, b) = if value == 0. {
            (1., 0.)
        } else {
            (self.value / value, y.value / value)
        };
        Self {
            value,
            gradient: combine(&self.gradient, a, &y.gradient, b),
        }
    }
    pub fn wrapped(mut self) -> Self {
        self.value = self.value.sin().atan2(self.value.cos());
        self
    }
}
/// Merges sorted local derivatives without allocating a global dense gradient.
fn combine(a: &[(usize, f64)], sa: f64, b: &[(usize, f64)], sb: f64) -> Gradient {
    let mut output = Gradient::new();
    let (mut i, mut j) = (0, 0);
    while i < a.len() || j < b.len() {
        let (index, value) = if j == b.len() || (i < a.len() && a[i].0 < b[j].0) {
            let value = (a[i].0, a[i].1 * sa);
            i += 1;
            value
        } else if i == a.len() || b[j].0 < a[i].0 {
            let value = (b[j].0, b[j].1 * sb);
            j += 1;
            value
        } else {
            let value = (a[i].0, a[i].1 * sa + b[j].1 * sb);
            i += 1;
            j += 1;
            value
        };
        if value != 0. {
            output.push((index, value));
        }
    }
    output
}
impl Add for Jet {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self {
            value: self.value + rhs.value,
            gradient: combine(&self.gradient, 1., &rhs.gradient, 1.),
        }
    }
}
impl Sub for Jet {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self {
            value: self.value - rhs.value,
            gradient: combine(&self.gradient, 1., &rhs.gradient, -1.),
        }
    }
}
impl Mul for Jet {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        Self {
            value: self.value * rhs.value,
            gradient: combine(&self.gradient, rhs.value, &rhs.gradient, self.value),
        }
    }
}
impl Div for Jet {
    type Output = Self;
    fn div(self, rhs: Self) -> Self {
        Self {
            value: self.value / rhs.value,
            gradient: combine(
                &self.gradient,
                1. / rhs.value,
                &rhs.gradient,
                -self.value / (rhs.value * rhs.value),
            ),
        }
    }
}
impl Neg for Jet {
    type Output = Self;
    fn neg(self) -> Self {
        self * -1.
    }
}
impl Add<f64> for Jet {
    type Output = Self;
    fn add(mut self, rhs: f64) -> Self {
        self.value += rhs;
        self
    }
}
impl Sub<f64> for Jet {
    type Output = Self;
    fn sub(mut self, rhs: f64) -> Self {
        self.value -= rhs;
        self
    }
}
impl Mul<f64> for Jet {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self {
        Self {
            value: self.value * rhs,
            gradient: combine(&self.gradient, rhs, &[], 0.),
        }
    }
}
impl Div<f64> for Jet {
    type Output = Self;
    fn div(self, rhs: f64) -> Self {
        self * (1. / rhs)
    }
}

#[derive(Clone, Debug)]
pub(super) struct V2 {
    pub x: Jet,
    pub y: Jet,
}
impl V2 {
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
impl Add for V2 {
    type Output = Self;
    fn add(self, b: Self) -> Self {
        Self {
            x: self.x + b.x,
            y: self.y + b.y,
        }
    }
}
impl Sub for V2 {
    type Output = Self;
    fn sub(self, b: Self) -> Self {
        Self {
            x: self.x - b.x,
            y: self.y - b.y,
        }
    }
}
