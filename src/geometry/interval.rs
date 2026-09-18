//! A directed range of parameters, in a stated parameter space.

use serde::{Deserialize, Serialize};

use crate::geometry::parameter::{Fraction, Native, Param};

/// A directed range of parameters in the space `S`.
///
/// Directed, not ordered: `start` may exceed `end`, and that is the same
/// geometry traversed backward rather than a normalization bug. Direction is
/// load-bearing — the minor and major arcs between two points on a circle share
/// both endpoints, so only the span tells them apart.
///
/// The space defaults to [`Native`], because most intervals in the kernel are a
/// support's own parameters. A range of traversal fractions is
/// `Interval<Normalized>`, and the two do not convert implicitly: crossing
/// between them takes [`at`](Self::at) or [`fraction_of`](Self::fraction_of),
/// each of which names the span the fraction is a fraction of.
#[derive(Debug, Serialize, Deserialize)]
#[serde(bound = "")]
pub struct Interval<S = Native> {
    pub start: Param<S>,
    pub end: Param<S>,
}

// Written out rather than derived, for the reason given on [`Param`]: a derive
// would bound each impl on the marker, and a marker is not a value.
impl<S> Clone for Interval<S> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<S> Copy for Interval<S> {}

impl<S> PartialEq for Interval<S> {
    fn eq(&self, other: &Self) -> bool {
        self.start == other.start && self.end == other.end
    }
}

impl<S> Interval<S> {
    pub fn new(start: impl Into<Param<S>>, end: impl Into<Param<S>>) -> Self {
        Self {
            start: start.into(),
            end: end.into(),
        }
    }

    /// Returns the same parameter span traversed in the opposite direction.
    pub fn reversed(self) -> Self {
        Self::new(self.end, self.start)
    }

    /// Returns the signed change in parameter from start to end.
    pub fn delta(self) -> f64 {
        self.end - self.start
    }

    /// Returns the parameter halfway along the span.
    pub fn midpoint(self) -> Param<S> {
        self.start + 0.5 * self.delta()
    }

    /// Maps a normalized traversal fraction onto this directed parameter span.
    ///
    /// Fraction `0` is [`start`](Self::start) and `1` is [`end`](Self::end),
    /// whichever way the span runs. This is one of the two conversions between
    /// parameter spaces, and the span it is asked of is the reference the
    /// fraction is a fraction of.
    pub fn at(self, fraction: Fraction) -> Param<S> {
        self.start + fraction.value() * self.delta()
    }

    /// Returns where `parameter` falls along this span, as a fraction of it.
    ///
    /// The inverse of [`at`](Self::at). A degenerate span has no direction to
    /// measure along, so every parameter on it reports fraction `0`.
    pub fn fraction_of(self, parameter: Param<S>) -> Fraction {
        let delta = self.delta();
        if delta == 0.0 {
            Fraction::START
        } else {
            Fraction::new((parameter - self.start) / delta)
        }
    }

    pub fn ordered(self) -> Self {
        if self.start <= self.end {
            self
        } else {
            Self::new(self.end, self.start)
        }
    }

    pub fn length(self) -> f64 {
        self.delta().abs()
    }

    pub fn contains(self, value: Param<S>, tolerance: f64) -> bool {
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
            let midpoint = Self::new(start, end).midpoint();
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
                Param::new(-extent)
            },
            if self.end.is_finite() {
                self.end
            } else {
                Param::new(extent)
            },
        )
    }
}

impl Interval<crate::geometry::parameter::Normalized> {
    /// The whole of a span, start to end.
    pub const UNIT: Self = Self {
        start: Fraction::START,
        end: Fraction::END,
    };
}
