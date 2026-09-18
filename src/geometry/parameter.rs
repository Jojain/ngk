//! Which parameter space a number lives in.
//!
//! Every parameter in the kernel used to be an `f64`, and `f64` was asked to
//! mean several incompatible things at once: a support's own parameter, a
//! normalized fraction of some span, a NURBS knot value. The prose told them
//! apart; nothing else did, and a fraction spent where a native parameter was
//! meant compiled and produced the wrong shape.
//!
//! [`Param<S>`] is that number with its space attached. `S` is a marker — it
//! holds no data and costs nothing at runtime — and the whole point is that
//! [`Param<Native>`] and [`Param<Normalized>`] are different types.
//!
//! # The two spaces
//!
//! **[`Native`]** is whatever parameterization the support itself uses: a
//! line's affine parameter, a conic's angle in radians, a NURBS curve's own
//! knot domain. Nothing is renormalized on the way in, so these numbers mean
//! what the geometry means by them.
//!
//! **[`Normalized`]** is a traversal fraction of *a named span*: `0` is that
//! span's start and `1` is its end. The span is the reference, and a fraction
//! without one is meaningless — which is why only
//! [`TrimmedCurve`](crate::geometry::TrimmedCurve) and
//! [`TrimmedCurve2`](crate::geometry::TrimmedCurve2) produce or consume one. A
//! bare [`Curve`](crate::geometry::Curve) has no fractions, because it has no
//! span to be a fraction of.
//!
//! The bridge between them is
//! [`Interval::at`](crate::geometry::Interval::at) and its inverse
//! [`Interval::fraction_of`](crate::geometry::Interval::fraction_of). Those are
//! the only two conversions, and each takes the reference span as its receiver,
//! so "a fraction of what" always has an answer at the call site.
//!
//! # What this does not promise
//!
//! A [`Fraction`] is **not** confined to `[0, 1]`. A solver hit just past the
//! end of a span, a clipped overlap, a point projected beyond an edge — each
//! produces one outside the unit interval, and that is information a bounded
//! type would either refuse or silently clamp away. The marker states where `0`
//! and `1` are; it does not state that the value is between them. Ask
//! [`Param::is_inside_unit`] where that matters.
//!
//! # Curves carry the brand; surfaces do not
//!
//! A brand earns its keep where two parameter spaces meet over one object. A
//! curve has both: its support's own parameter, and a fraction of whichever
//! span is meant. A surface has one. There is no trimmed surface and no surface
//! fraction, so a surface's `u` and `v` have nothing to be confused with, and
//! they stay plain `f64` — as does the [`Point2`](crate::geometry::Point2) that
//! is a point of its parameter space. A surface's `domain()` is still an
//! [`Interval<Native>`](crate::geometry::Interval), because that is what its
//! parameters are; reading an endpoint out to evaluate takes
//! [`Param::value`].
//!
//! Within a surface, `u` is not distinguished from `v` either: both are its own
//! parameterization, and telling them apart would fork every axis-generic
//! helper over [`Axis2`](crate::geometry::Axis2) in two. A swapped `u`/`v`
//! produces visibly wrong geometry; the confusion these types exist for is the
//! silent one.

use std::fmt;
use std::marker::PhantomData;
use std::ops::{Add, AddAssign, Neg, Sub, SubAssign};

use serde::{Deserialize, Serialize};

/// A support's own parameterization.
///
/// A line's affine parameter, a conic's angle in radians, a NURBS curve's knot
/// domain. The units differ per support.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Native;

/// Normalized traversal of a named span: `0` is its start, `1` is its end.
///
/// Meaningful only against the span it is a fraction of. Values outside
/// `[0, 1]` name points beyond that span's ends and are not errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Normalized;

/// A parameter in the space `S`.
///
/// Wraps one `f64` and carries no other data. Arithmetic is deliberately
/// narrow: two parameters of the same space subtract to a plain distance in
/// that space, and a parameter shifts by such a distance. Adding two parameters
/// is not offered, because the sum of two positions is not a position — use
/// [`Interval::midpoint`](crate::geometry::Interval::midpoint) when a midpoint
/// is what is wanted.
#[repr(transparent)]
#[derive(Serialize, Deserialize)]
#[serde(transparent, bound = "")]
pub struct Param<S = Native> {
    value: f64,
    #[serde(skip)]
    space: PhantomData<S>,
}

/// A parameter in a support's own parameterization.
pub type NativeParam = Param<Native>;

/// A normalized traversal fraction of a named span.
pub type Fraction = Param<Normalized>;

impl<S> Param<S> {
    /// Reads `value` as a parameter in this space.
    pub const fn new(value: f64) -> Self {
        Self {
            value,
            space: PhantomData,
        }
    }

    /// Returns the underlying number, discarding the space.
    pub const fn value(self) -> f64 {
        self.value
    }

    /// Returns whether the parameter is a usable number.
    ///
    /// An unbounded domain reports infinite endpoints rather than refusing to
    /// have a domain, so a caller that needs a finite window asks here.
    pub fn is_finite(self) -> bool {
        self.value.is_finite()
    }

    /// Returns the lesser of two parameters.
    pub fn min(self, other: Self) -> Self {
        Self::new(self.value.min(other.value))
    }

    /// Returns the greater of two parameters.
    pub fn max(self, other: Self) -> Self {
        Self::new(self.value.max(other.value))
    }

    /// Returns this parameter confined to `[low, high]`.
    pub fn clamp(self, low: Self, high: Self) -> Self {
        Self::new(self.value.clamp(low.value, high.value))
    }

    /// Orders two parameters totally, including across `NaN`.
    pub fn total_cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.value.total_cmp(&other.value)
    }

    /// Returns this parameter shifted by whole multiples of `period` onto the
    /// branch nearest `target`.
    ///
    /// A periodic support reports its parameter on one fixed branch — a circle
    /// uses `atan2`, so `(-pi, pi]` — which need not be the branch a span
    /// covers.
    pub fn on_branch_near(self, target: Self, period: f64) -> Self {
        self + ((target.value - self.value) / period).round() * period
    }
}

impl Fraction {
    /// The start of any span.
    pub const START: Self = Self::new(0.0);

    /// The end of any span.
    pub const END: Self = Self::new(1.0);

    /// Returns whether this fraction falls within the span it measures.
    ///
    /// A fraction outside `[0, 1]` is a legitimate value naming a point beyond
    /// the span's ends; this is the question to ask when that matters.
    pub fn is_inside_unit(self) -> bool {
        (0.0..=1.0).contains(&self.value)
    }

    /// Returns this fraction confined to the span it measures.
    pub fn clamped_to_unit(self) -> Self {
        self.clamp(Self::START, Self::END)
    }
}

impl<S> From<f64> for Param<S> {
    fn from(value: f64) -> Self {
        Self::new(value)
    }
}

impl<S> From<Param<S>> for f64 {
    fn from(parameter: Param<S>) -> Self {
        parameter.value
    }
}

/// Two parameters of one space differ by a distance in that space.
impl<S> Sub for Param<S> {
    type Output = f64;

    fn sub(self, other: Self) -> f64 {
        self.value - other.value
    }
}

impl<S> Add<f64> for Param<S> {
    type Output = Self;

    fn add(self, delta: f64) -> Self {
        Self::new(self.value + delta)
    }
}

impl<S> Sub<f64> for Param<S> {
    type Output = Self;

    fn sub(self, delta: f64) -> Self {
        Self::new(self.value - delta)
    }
}

impl<S> AddAssign<f64> for Param<S> {
    fn add_assign(&mut self, delta: f64) {
        self.value += delta;
    }
}

impl<S> SubAssign<f64> for Param<S> {
    fn sub_assign(&mut self, delta: f64) {
        self.value -= delta;
    }
}

impl<S> Neg for Param<S> {
    type Output = Self;

    fn neg(self) -> Self {
        Self::new(-self.value)
    }
}

// Every one of these is written out rather than derived. A derive would bound
// the impl on the *marker* — `S: Copy`, `S: PartialEq` — which is both wrong
// and load-bearing: inside code generic over `S` those bounds do not hold, and
// a parameter would stop being `Copy` exactly where `Interval<S>` needs it. A
// marker carries no data, so none of these depend on it.
impl<S> Clone for Param<S> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<S> Copy for Param<S> {}

// Two parameters compare when they live in the same space, and a parameter
// never compares against a bare `f64` — that comparison is the mixing these
// types exist to prevent.
impl<S> PartialEq for Param<S> {
    fn eq(&self, other: &Self) -> bool {
        self.value == other.value
    }
}

impl<S> PartialOrd for Param<S> {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        self.value.partial_cmp(&other.value)
    }
}

impl<S> fmt::Debug for Param<S> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Debug::fmt(&self.value, f)
    }
}

impl<S> fmt::Display for Param<S> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Display::fmt(&self.value, f)
    }
}
