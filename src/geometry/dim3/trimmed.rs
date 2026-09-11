//! A curve paired with the portion of it that is meant.
//!
//! A [`Curve`] is a support: it is never cut down to the cell or section
//! resting on it, so a line runs to infinity and a circle closes. Saying *which
//! part* is meant therefore takes a second value, an [`Interval`] of the
//! support's own native parameters. [`TrimmedCurve`] is that pair, kept
//! together so the two halves cannot drift apart.
//!
//! Nothing here trims anything: the support is carried untouched, which is what
//! keeps an arc an exact [`Curve::Circle`] instead of degrading it to NURBS.
//! The name describes what the value *denotes*, not what it does to the curve.
//! Use [`TrimmedCurve::to_curve`] for the cut-down copy when one is genuinely
//! needed, such as deriving a pcurve.

use serde::{Deserialize, Serialize};

use super::curves::{Circle, Curve, Ellipse, Periodicity};
use super::frame::Frame;
use super::surfaces::Plane;
use super::utils::{Point3, PointCoincidence};
use crate::geometry::Interval;
use crate::geometry::nurbs::error::NurbsError;

/// A support curve together with the native parameter span that is meant.
///
/// Traversal is normalized: fraction `0` is [`start`](Self::start), fraction
/// `1` is [`end`](Self::end), whichever way the span runs. A reversed span
/// (`interval.start > interval.end`) traverses the same geometry backward, and
/// is a distinct value from its forward twin rather than a normalization bug.
///
/// Direction is load-bearing and cannot be recovered from the endpoints alone:
/// the minor and major arcs between two points on a circle share both ends, so
/// only the span distinguishes them. That is why the span is carried rather
/// than derived — except on an edge, where the bounding vertices plus the
/// edge's own orientation do determine it, and
/// [`Edge::trimmed_curve`](crate::topology::edge::Edge::trimmed_curve) derives
/// it there.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TrimmedCurve {
    curve: Curve,
    interval: Interval,
}

impl TrimmedCurve {
    /// Pairs a support with an explicit span of its native parameters.
    pub fn new(curve: Curve, interval: Interval) -> Self {
        Self { curve, interval }
    }

    /// Pairs a support with the span running forward from `start` to `end`.
    ///
    /// Only the shorter way round is expressible on a periodic support: the
    /// span always advances in increasing parameter, so of the two arcs joining
    /// two points this selects the one reached by going forward from `start`.
    /// A section that needs the other one has to state its span with
    /// [`new`](Self::new).
    pub fn between(curve: Curve, start: Point3, end: Point3) -> Self {
        let interval = curve.interval_between(start, end);
        Self { curve, interval }
    }

    /// Returns the straight segment from `start` to `end`.
    ///
    /// The support is the infinite line through both points, and the span is
    /// its `[0, 1]` window.
    pub fn segment(start: Point3, end: Point3) -> Self {
        Self::new(Curve::line(start, end), Interval::new(0.0, 1.0))
    }

    /// Returns the arc that starts along the plane's `x_dir` and sweeps `sweep`
    /// radians around its normal.
    ///
    /// The support is the whole circle; `sweep` is the span over it, so its
    /// sign chooses the direction and its magnitude may exceed a full turn.
    /// Naming an arc between two known points is done by placing the plane's
    /// `x_dir` on the first and stating the sweep, rather than by giving the far
    /// endpoint: the two arcs joining those points share both ends.
    pub fn arc(plane: Plane, radius: f64, sweep: f64) -> Self {
        Self::new(
            Curve::Circle(Circle::new(plane, radius)),
            Interval::new(0.0, sweep),
        )
    }

    /// Returns the elliptical arc that starts on the frame's X axis and sweeps
    /// `sweep` radians of eccentric angle.
    ///
    /// As with [`arc`](Self::arc), the support is the whole ellipse and the
    /// sweep is the span over it.
    pub fn ellipse_arc(frame: Frame, major_radius: f64, minor_radius: f64, sweep: f64) -> Self {
        Self::new(
            Curve::Ellipse(Ellipse::new(frame, major_radius, minor_radius)),
            Interval::new(0.0, sweep),
        )
    }

    /// Returns the span covering the whole of a support's own domain.
    ///
    /// Meaningful only for a support that was built to *be* the section — an
    /// interpolated NURBS curve, say — where its domain already is the span
    /// that is meant. An unbounded support has no such domain, and the span
    /// comes back unbounded with it.
    pub fn whole(curve: Curve) -> Self {
        let interval = curve.domain();
        Self::new(curve, interval)
    }

    /// Returns the untrimmed support.
    pub fn curve(&self) -> &Curve {
        &self.curve
    }

    /// Returns the native parameter span that is meant.
    pub fn interval(&self) -> Interval {
        self.interval
    }

    /// Returns the support, discarding the span.
    pub fn into_curve(self) -> Curve {
        self.curve
    }

    /// Evaluates at a normalized traversal fraction of the span.
    pub fn point_at(&self, fraction: f64) -> Point3 {
        self.curve.point_at(self.interval.at(fraction))
    }

    /// Returns the first point of the span.
    pub fn start(&self) -> Point3 {
        self.point_at(0.0)
    }

    /// Returns the last point of the span.
    pub fn end(&self) -> Point3 {
        self.point_at(1.0)
    }

    /// Returns the derivative with respect to the normalized fraction.
    pub fn derivative_at(&self, fraction: f64, order: usize) -> nalgebra::Vector3<f64> {
        let derivative = self.curve.derivative_at(self.interval.at(fraction), order);
        match order {
            1 => derivative * self.interval.delta(),
            _ => derivative,
        }
    }

    /// Locates `point` on the branch this span lives on, in native parameters.
    ///
    /// A periodic support reports its parameter on one fixed branch — a circle
    /// uses `atan2`, so `(-pi, pi]` — which need not be the branch this span
    /// covers. The raw parameter is shifted by whole periods onto the branch
    /// nearest the span, so one crossing the branch cut still measures against
    /// its own extent rather than the complementary one.
    pub fn native_parameter_at(&self, point: Point3) -> f64 {
        let raw = self.curve.param_at(point);
        let Periodicity::Periodic(period) = self.curve.periodicity() else {
            return raw;
        };
        let middle = 0.5 * (self.interval.start + self.interval.end);
        raw + ((middle - raw) / period).round() * period
    }

    /// Returns where `point` falls along the span, as a normalized fraction.
    pub fn parameter_at(&self, point: Point3) -> f64 {
        (self.native_parameter_at(point) - self.interval.start) / self.interval.delta()
    }

    /// Whether `point` lies on this span, and not merely on the support.
    ///
    /// This is the question a support cannot answer for itself: an edge's line
    /// meets things far outside the edge, and those hits are not contacts with
    /// it. `tolerance` is a distance, converted to parameters at the span's own
    /// speed so it means the same thing on a long span as on a short one.
    pub fn contains(&self, point: Point3, tolerance: f64) -> bool {
        self.curve
            .point_at(self.curve.param_at(point))
            .coincides(point, tolerance)
            && self.interval.contains(
                self.native_parameter_at(point),
                self.parameter_slack(tolerance),
            )
    }

    /// Converts a distance tolerance into one in this support's parameters.
    ///
    /// Native parameters are not arc length — a circle's are radians — so a
    /// fixed parameter slack would admit a point a whole radius away on a small
    /// arc and reject one well inside a long one. Dividing by the speed at the
    /// span's midpoint spends the tolerance in the unit it was given in.
    pub fn parameter_slack(&self, tolerance: f64) -> f64 {
        let speed = self
            .curve
            .derivative_at(0.5 * (self.interval.start + self.interval.end), 1)
            .norm();
        if speed > crate::geometry::LINEAR_TOLERANCE {
            tolerance / speed
        } else {
            tolerance
        }
    }

    /// Returns the same geometry traversed in the opposite direction.
    pub fn reversed(&self) -> Self {
        Self::new(self.curve.clone(), self.interval.reversed())
    }

    /// Narrows to a sub-span, given as normalized fractions of this one.
    ///
    /// The support is carried over untouched, so repeated narrowing never
    /// accumulates conversion error.
    pub fn sub(&self, fractions: Interval) -> Self {
        Self::new(
            self.curve.clone(),
            Interval::new(
                self.interval.at(fractions.start),
                self.interval.at(fractions.end),
            ),
        )
    }

    /// Returns the arc length of the span.
    pub fn length(&self) -> f64 {
        self.curve.length(self.interval.start, self.interval.end)
    }

    /// Returns the span as a standalone curve parameterized over `[0, 1]`.
    ///
    /// This is the cut-down copy: exact, but no longer the analytic support —
    /// an arc comes back as the NURBS that represents it. Use it only where a
    /// curve that *is* the section is required, such as fitting a pcurve to it;
    /// anywhere the support's identity matters, keep the [`TrimmedCurve`].
    pub fn to_curve(&self) -> Result<Curve, NurbsError> {
        self.curve.trimmed_native(self.interval)
    }

    /// Returns the span's bounding box.
    pub fn bbox(&self) -> Option<crate::geometry::BBox> {
        self.curve.bbox_over(self.interval)
    }
}
