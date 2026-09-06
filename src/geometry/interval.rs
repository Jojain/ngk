use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Interval {
    pub start: f64,
    pub end: f64,
}

impl Interval {
    pub fn new(start: f64, end: f64) -> Self {
        Self { start, end }
    }

    pub fn ordered(self) -> Self {
        if self.start <= self.end {
            self
        } else {
            Self::new(self.end, self.start)
        }
    }

    pub fn length(self) -> f64 {
        (self.end - self.start).abs()
    }

    pub fn contains(self, value: f64, tolerance: f64) -> bool {
        let ordered = self.ordered();
        value >= ordered.start - tolerance && value <= ordered.end + tolerance
    }

    pub fn is_degenerate(self, tolerance: f64) -> bool {
        self.length() <= tolerance
    }

    pub fn intersects(self, other: Self, tolerance: f64) -> bool {
        self.intersection(other, tolerance).is_some()
    }

    pub fn intersection(self, other: Self, tolerance: f64) -> Option<Self> {
        let a = self.ordered();
        let b = other.ordered();
        let start = a.start.max(b.start);
        let end = a.end.min(b.end);
        if start <= end {
            return Some(Self::new(start, end));
        }
        if start - end <= tolerance {
            let midpoint = 0.5 * (start + end);
            return Some(Self::new(midpoint, midpoint));
        }
        None
    }

    /// An interval covering the whole real line.
    ///
    /// Unbounded supports — a line, a plane, the height of a cylinder — report
    /// this from their `domain()`, so a caller that needs a finite window has
    /// to clamp it rather than assume the endpoints are usable numbers.
    pub fn unbounded() -> Self {
        Self::new(f64::NEG_INFINITY, f64::INFINITY)
    }

    /// Returns whether both endpoints are finite.
    pub fn is_finite(self) -> bool {
        self.start.is_finite() && self.end.is_finite()
    }

    /// Returns this interval with each infinite endpoint replaced by `±extent`.
    ///
    /// Debug views and tessellation need a finite window over an unbounded
    /// support. Finite endpoints are left alone, so a bounded domain keeps its
    /// real extent even when it is wider than `extent`.
    pub fn or_extent(self, extent: f64) -> Self {
        Self::new(
            if self.start.is_finite() {
                self.start
            } else {
                -extent
            },
            if self.end.is_finite() {
                self.end
            } else {
                extent
            },
        )
    }
}
