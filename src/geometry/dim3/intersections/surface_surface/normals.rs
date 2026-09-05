//! Conservative normal-direction cones for rational Bézier patches.
//!
//! A patch that is nowhere parallel to another patch cannot meet it in a closed
//! loop (Sederberg and Meyers, *Loop detection in surface patch intersections*),
//! which is what lets a subdivision search stop without hunting for interior
//! loops. Making that argument requires a sound outer bound on the directions a
//! patch normal can take, and that is what this module computes.
//!
//! For a rational patch `S = P / w` with positive weights, writing `P` for the
//! weighted control polynomial,
//!
//! ```text
//! S_u x S_v = (1 / w^3) * (w (P_u x P_v) - w_v (P_u x P) - w_u (P x P_v))
//! ```
//!
//! so the bracketed vector polynomial `N` is everywhere parallel to the normal.
//! `N` is a tensor-product Bernstein polynomial whose values are convex
//! combinations of its own coefficients, so the cone those coefficients span
//! contains every normal direction of the patch.

use nalgebra::{UnitVector3, Vector3};

use crate::geometry::BezierSurface;

/// A tensor-product Bernstein polynomial in Bernstein (control) form.
///
/// `nu` and `nv` are coefficient counts, so the bidegree is `(nu - 1, nv - 1)`.
struct BernsteinNet<T> {
    coefficients: Vec<T>,
    nu: usize,
    nv: usize,
}

impl<T: Copy> BernsteinNet<T> {
    fn get(&self, u: usize, v: usize) -> T {
        self.coefficients[v * self.nu + u]
    }

    fn degree_u(&self) -> usize {
        self.nu - 1
    }

    fn degree_v(&self) -> usize {
        self.nv - 1
    }
}

impl BernsteinNet<Vector3<f64>> {
    /// Adds two nets of equal bidegree coefficientwise.
    fn add(&self, other: &Self) -> Self {
        Self {
            coefficients: self
                .coefficients
                .iter()
                .zip(&other.coefficients)
                .map(|(left, right)| left + right)
                .collect(),
            nu: self.nu,
            nv: self.nv,
        }
    }

    /// Scales every coefficient.
    fn scaled(&self, factor: f64) -> Self {
        Self {
            coefficients: self
                .coefficients
                .iter()
                .map(|value| value * factor)
                .collect(),
            nu: self.nu,
            nv: self.nv,
        }
    }
}

/// Differentiates a Bernstein net in u, lowering its u degree by one.
fn derivative_u<T>(net: &BernsteinNet<T>) -> BernsteinNet<T>
where
    T: Copy + std::ops::Sub<Output = T> + std::ops::Mul<f64, Output = T>,
{
    let degree = net.degree_u() as f64;
    let mut coefficients = Vec::with_capacity((net.nu - 1) * net.nv);
    for v in 0..net.nv {
        for u in 0..net.nu - 1 {
            coefficients.push((net.get(u + 1, v) - net.get(u, v)) * degree);
        }
    }
    BernsteinNet {
        coefficients,
        nu: net.nu - 1,
        nv: net.nv,
    }
}

/// Differentiates a Bernstein net in v, lowering its v degree by one.
fn derivative_v<T>(net: &BernsteinNet<T>) -> BernsteinNet<T>
where
    T: Copy + std::ops::Sub<Output = T> + std::ops::Mul<f64, Output = T>,
{
    let degree = net.degree_v() as f64;
    let mut coefficients = Vec::with_capacity(net.nu * (net.nv - 1));
    for v in 0..net.nv - 1 {
        for u in 0..net.nu {
            coefficients.push((net.get(u, v + 1) - net.get(u, v)) * degree);
        }
    }
    BernsteinNet {
        coefficients,
        nu: net.nu,
        nv: net.nv - 1,
    }
}

/// Multiplies two Bernstein nets under a bilinear coefficient operation.
///
/// The Bernstein product rule `B_i^m B_j^n = C(m,i) C(n,j) / C(m+n,i+j) *
/// B_{i+j}^{m+n}` applies independently in each parameter, so the result is
/// exact rather than a resampling.
fn product<A, B, C>(
    left: &BernsteinNet<A>,
    right: &BernsteinNet<B>,
    combine: impl Fn(A, B) -> C,
) -> BernsteinNet<C>
where
    A: Copy,
    B: Copy,
    C: Copy + Default + std::ops::Add<Output = C> + std::ops::Mul<f64, Output = C>,
{
    let (p, q) = (left.degree_u(), left.degree_v());
    let (r, s) = (right.degree_u(), right.degree_v());
    let (nu, nv) = (p + r + 1, q + s + 1);
    let mut coefficients = vec![C::default(); nu * nv];
    for k in 0..nu {
        for l in 0..nv {
            let mut total = C::default();
            for i in k.saturating_sub(r)..=k.min(p) {
                for m in l.saturating_sub(s)..=l.min(q) {
                    let weight = binomial(p, i) * binomial(r, k - i) / binomial(p + r, k)
                        * binomial(q, m)
                        * binomial(s, l - m)
                        / binomial(q + s, l);
                    total = total + combine(left.get(i, m), right.get(k - i, l - m)) * weight;
                }
            }
            coefficients[l * nu + k] = total;
        }
    }
    BernsteinNet {
        coefficients,
        nu,
        nv,
    }
}

/// Returns `C(n, k)` exactly for the small degrees Bézier patches use.
fn binomial(n: usize, k: usize) -> f64 {
    if k > n {
        return 0.0;
    }
    let mut value = 1.0;
    for step in 0..k.min(n - k) {
        value = value * (n - step) as f64 / (step + 1) as f64;
    }
    value
}

/// A cone of directions containing every normal direction of one patch.
#[derive(Debug, Clone, Copy)]
pub(super) struct NormalCone {
    axis: UnitVector3<f64>,
    half_angle: f64,
}

impl NormalCone {
    /// Bounds the normal directions of a positive-weight rational Bézier patch.
    ///
    /// Returns `None` when the bound would not be a proper cone: a coefficient
    /// that is negligible against the largest one leaves the direction of the
    /// normal field unconstrained there, and a half angle at or beyond a right
    /// angle no longer describes a convex cone.
    pub(super) fn from_patch(patch: &BezierSurface, tolerance: f64) -> Option<Self> {
        let normals = normal_field(patch);
        let longest = normals
            .coefficients
            .iter()
            .map(|value| value.norm())
            .fold(0.0_f64, f64::max);
        if longest <= tolerance {
            return None;
        }
        let mut directions = Vec::with_capacity(normals.coefficients.len());
        for coefficient in &normals.coefficients {
            let norm = coefficient.norm();
            if norm <= longest * tolerance {
                return None;
            }
            directions.push(coefficient / norm);
        }
        let axis = directions
            .iter()
            .fold(Vector3::zeros(), |sum, direction| sum + direction);
        let axis = UnitVector3::try_new(axis, tolerance)?;
        let half_angle = directions
            .iter()
            .map(|direction| direction.dot(&axis).clamp(-1.0, 1.0).acos())
            .fold(0.0_f64, f64::max);
        (half_angle < std::f64::consts::FRAC_PI_2).then_some(Self { axis, half_angle })
    }

    /// Returns the cone's half angle, which measures how loose the bound is.
    pub(super) fn width(self) -> f64 {
        self.half_angle
    }

    /// Returns whether no direction, or its opposite, is shared with `other`.
    ///
    /// The opposite is included so the answer does not depend on which way each
    /// patch happens to be parameterized: two patches whose normals are
    /// antiparallel are as tangent as two whose normals agree.
    pub(super) fn is_disjoint_from(self, other: Self, angular_tolerance: f64) -> bool {
        let between = self.axis.dot(&other.axis).clamp(-1.0, 1.0).acos();
        let separation = between.min(std::f64::consts::PI - between);
        separation > self.half_angle + other.half_angle + angular_tolerance
    }
}

/// Builds the Bernstein net of the patch's unnormalized normal field.
fn normal_field(patch: &BezierSurface) -> BernsteinNet<Vector3<f64>> {
    let net = patch.control_points();
    let points = BernsteinNet {
        coefficients: net
            .as_slice()
            .iter()
            .map(|point| point.weighted_xyz())
            .collect(),
        nu: net.nu(),
        nv: net.nv(),
    };
    let weights = BernsteinNet {
        coefficients: net.as_slice().iter().map(|point| point.weight()).collect(),
        nu: net.nu(),
        nv: net.nv(),
    };
    let points_u = derivative_u(&points);
    let points_v = derivative_v(&points);
    let weights_u = derivative_u(&weights);
    let weights_v = derivative_v(&weights);

    let cross_uv = product(&points_u, &points_v, |a: Vector3<f64>, b: Vector3<f64>| {
        a.cross(&b)
    });
    let cross_up = product(&points_u, &points, |a: Vector3<f64>, b: Vector3<f64>| {
        a.cross(&b)
    });
    let cross_pv = product(&points, &points_v, |a: Vector3<f64>, b: Vector3<f64>| {
        a.cross(&b)
    });

    let first = product(&weights, &cross_uv, |w: f64, n: Vector3<f64>| n * w);
    let second = product(&weights_v, &cross_up, |w: f64, n: Vector3<f64>| n * w);
    let third = product(&weights_u, &cross_pv, |w: f64, n: Vector3<f64>| n * w);
    first.add(&second.scaled(-1.0)).add(&third.scaled(-1.0))
}
