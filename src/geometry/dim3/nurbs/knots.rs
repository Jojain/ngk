use super::degree::Degree;
use super::error::NurbsError;

/// A non-decreasing knot vector.
#[derive(Debug, Clone, PartialEq)]
pub struct KnotVector(Vec<f64>);

impl KnotVector {
    pub fn new(knots: Vec<f64>) -> Result<Self, NurbsError> {
        if knots.is_empty() {
            return Err(NurbsError::EmptyKnotVector);
        }
        if knots.windows(2).any(|w| w[0] > w[1]) {
            return Err(NurbsError::UnsortedKnots);
        }
        Ok(Self(knots))
    }

    /// Clamped uniform knot vector for `num_control_points` control points of
    /// the given `degree`. Length is `n + p + 1 + 1 = num_control_points + p + 1`.
    /// Domain is `[0, 1]`.
    pub fn uniform_clamped(num_control_points: usize, degree: Degree) -> Self {
        let p = degree.get();
        let n = num_control_points.saturating_sub(1);
        let m = n + p + 1;
        let mut knots = Vec::with_capacity(m + 1);
        for _ in 0..=p {
            knots.push(0.0);
        }
        let interior = m.saturating_sub(2 * p + 1);
        for i in 1..=interior {
            knots.push(i as f64 / (interior as f64 + 1.0));
        }
        for _ in 0..=p {
            knots.push(1.0);
        }
        Self(knots)
    }

    pub fn as_slice(&self) -> &[f64] {
        &self.0
    }

    pub fn len(&self) -> usize {
        self.0.len()
    }

    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    pub fn get(&self, i: usize) -> f64 {
        self.0[i]
    }

    /// Parametric domain `[U[p], U[m-p]]` where `m = len - 1`.
    pub fn domain(&self, degree: Degree) -> (f64, f64) {
        let p = degree.get();
        let m = self.0.len() - 1;
        (self.0[p], self.0[m - p])
    }

    /// Piegl & Tiller A2.1 — find the knot span index for parameter `u`.
    /// `n = num_control_points - 1`, `p = degree`.
    pub fn find_span(&self, n: usize, degree: Degree, u: f64) -> usize {
        let p = degree.get();
        if u >= self.0[n + 1] {
            return n;
        }
        if u <= self.0[p] {
            return p;
        }
        let mut low = p;
        let mut high = n + 1;
        let mut mid = (low + high) / 2;
        while u < self.0[mid] || u >= self.0[mid + 1] {
            if u < self.0[mid] {
                high = mid;
            } else {
                low = mid;
            }
            mid = (low + high) / 2;
        }
        mid
    }

    /// Number of times `u` appears as a knot.
    pub fn multiplicity(&self, u: f64) -> usize {
        self.0.iter().filter(|&&k| k == u).count()
    }

    pub fn is_clamped(&self, degree: Degree) -> bool {
        let p = degree.get();
        if self.0.len() < 2 * (p + 1) {
            return false;
        }
        let first = self.0[0];
        let last = *self.0.last().unwrap();
        self.0.iter().take(p + 1).all(|&k| k == first)
            && self.0.iter().rev().take(p + 1).all(|&k| k == last)
    }

    pub(crate) fn insert(&mut self, position: usize, value: f64) {
        self.0.insert(position, value);
    }
}
