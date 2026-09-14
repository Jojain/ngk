//! A 2D support curve paired with the portion of it that is meant.
//!
//! This is the parameter-space twin of
//! [`TrimmedCurve`](crate::geometry::TrimmedCurve), and it exists for the same
//! reason: a [`Curve2`] is a support, never cut down to the pcurve resting on
//! it, so a [`Line2`](super::curves::Line2) runs to infinity and a
//! [`Circle2`](super::curves::Circle2) closes. Saying *which part* is meant
//! takes a second value, an [`Interval`] of the support's own native
//! parameters. [`TrimmedCurve2`] is that pair, kept together so the two halves
//! cannot drift apart.
//!
//! Nothing here trims anything: the support is carried untouched, which is what
//! keeps a pcurve arc an exact `Circle2` instead of degrading it to NURBS. Use
//! [`TrimmedCurve2::to_curve`] for the cut-down copy when one is genuinely
//! needed.
//!
//! Unlike a 3D edge, a pcurve has no bounding vertices to derive its span from
//! — a face stores no 2D vertex positions — so every pcurve carries its own
//! [`TrimmedCurve2`].

use nalgebra::Vector2;
use serde::{Deserialize, Serialize};

use super::curves::{Circle2, Curve2, Ellipse2};
use super::intersections::{
    CurveCurveIntersections2, CurveIntersectionError, CurveIntersectionOptions, intersect_curves,
    intersect_curves_with_options,
};
use super::nurbs::NurbsCurve2;
use super::utils::Point2;
use crate::geometry::dim3::curves::Periodicity;
use crate::geometry::nurbs::error::NurbsError;
use crate::geometry::{Interval, LINEAR_TOLERANCE};

/// A 2D support together with the native parameter span that is meant.
///
/// Traversal is normalized: fraction `0` is [`start`](Self::start), fraction
/// `1` is [`end`](Self::end), whichever way the span runs. A reversed span
/// (`interval.start > interval.end`) traverses the same geometry backward, and
/// is a distinct value from its forward twin rather than a normalization bug.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TrimmedCurve2 {
    curve: Curve2,
    interval: Interval,
}

impl TrimmedCurve2 {
    /// Pairs a support with an explicit span of its native parameters.
    pub fn new(curve: Curve2, interval: Interval) -> Self {
        Self { curve, interval }
    }

    /// Pairs a support with the span running forward from `start` to `end`.
    ///
    /// Only the shorter way round is expressible on a periodic support: the
    /// span always advances in increasing parameter, so of the two arcs joining
    /// two points this selects the one reached by going forward from `start`.
    /// A section that needs the other one states its span with
    /// [`new`](Self::new).
    pub fn between(curve: Curve2, start: Point2, end: Point2) -> Self {
        let interval = curve.interval_between(start, end);
        Self { curve, interval }
    }

    /// Returns the straight segment from `start` to `end`.
    ///
    /// The support is the infinite line through both points, and the span is
    /// its `[0, 1]` window — the overwhelmingly common pcurve shape.
    pub fn segment(start: Point2, end: Point2) -> Self {
        Self::new(Curve2::line(start, end), Interval::new(0.0, 1.0))
    }

    /// Returns the arc that starts along `x_dir` and sweeps `sweep` radians.
    ///
    /// The support is the whole circle; `sweep` is the span over it, so its
    /// sign chooses the direction and its magnitude may exceed a full turn.
    /// Naming an arc between two known points is `arc(center, first - center,
    /// radius, sweep)`, which is why the sweep is stated rather than the far
    /// endpoint: the two arcs joining those points share both ends.
    pub fn arc(center: Point2, x_dir: Vector2<f64>, radius: f64, sweep: f64) -> Self {
        Self::new(
            Curve2::Circle(Circle2::new(center, x_dir, radius)),
            Interval::new(0.0, sweep),
        )
    }

    /// Returns the elliptical arc that starts on the major axis and sweeps
    /// `sweep` radians of eccentric angle.
    ///
    /// As with [`arc`](Self::arc), the support is the whole ellipse and the
    /// sweep is the span over it.
    pub fn ellipse_arc(
        center: Point2,
        x_dir: Vector2<f64>,
        major_radius: f64,
        minor_radius: f64,
        sweep: f64,
    ) -> Self {
        Self::new(
            Curve2::Ellipse(Ellipse2::new(center, x_dir, major_radius, minor_radius)),
            Interval::new(0.0, sweep),
        )
    }

    /// Returns the span covering the whole of a support's own domain.
    ///
    /// Meaningful only for a support that was built to *be* the section — an
    /// interpolated NURBS pcurve, say — where its domain already is the span
    /// that is meant. An unbounded support has no such domain, and the span
    /// comes back unbounded with it.
    pub fn whole(curve: Curve2) -> Self {
        let domain = curve.domain();
        Self::new(curve, domain)
    }

    /// Returns the untrimmed support.
    pub fn curve(&self) -> &Curve2 {
        &self.curve
    }

    /// Returns the native parameter span that is meant.
    pub fn interval(&self) -> Interval {
        self.interval
    }

    /// Returns the support, discarding the span.
    pub fn into_curve(self) -> Curve2 {
        self.curve
    }

    /// Evaluates at a normalized traversal fraction of the span.
    pub fn point_at(&self, fraction: f64) -> Point2 {
        self.curve.point_at(self.interval.at(fraction))
    }

    /// Returns the first point of the span.
    pub fn start(&self) -> Point2 {
        self.point_at(0.0)
    }

    /// Returns the last point of the span.
    pub fn end(&self) -> Point2 {
        self.point_at(1.0)
    }

    /// Returns the derivative with respect to the normalized fraction.
    pub fn derivative_at(&self, fraction: f64, order: usize) -> Vector2<f64> {
        let derivative = self.curve.derivative_at(self.interval.at(fraction), order);
        match order {
            1 => derivative * self.interval.delta(),
            _ => derivative,
        }
    }

    /// Returns whether the span closes on itself.
    pub fn is_closed(&self) -> bool {
        (self.end() - self.start()).norm() <= LINEAR_TOLERANCE
    }

    /// Locates `point` on the branch this span lives on, in native parameters.
    ///
    /// A periodic support reports its parameter on one fixed branch — a circle
    /// uses `atan2`, so `(-pi, pi]` — which need not be the branch this span
    /// covers. The raw parameter is shifted by whole periods onto the branch
    /// nearest the span, so one crossing the branch cut still measures against
    /// its own extent rather than the complementary one.
    pub fn native_parameter_at(&self, point: Point2) -> f64 {
        let raw = self.curve.param_at(point);
        let Periodicity::Periodic(period) = self.curve.periodicity() else {
            return raw;
        };
        let middle = 0.5 * (self.interval.start + self.interval.end);
        raw + ((middle - raw) / period).round() * period
    }

    /// Returns where `point` falls along the span, as a normalized fraction.
    pub fn parameter_at(&self, point: Point2) -> f64 {
        (self.native_parameter_at(point) - self.interval.start) / self.interval.delta()
    }

    /// Returns the fraction of `point`, or `None` when it is not on the span.
    pub fn try_parameter_at(&self, point: Point2, tolerance: f64) -> Option<f64> {
        self.contains(point, tolerance)
            .then(|| self.parameter_at(point).clamp(0.0, 1.0))
    }

    /// Whether `point` lies on this span, and not merely on the support.
    ///
    /// This is the question a support cannot answer for itself: a pcurve's line
    /// extends far outside the boundary edge it describes, and points out there
    /// are not on the pcurve. `tolerance` is a distance, converted to
    /// parameters at the span's own speed so it means the same thing on a long
    /// span as on a short one.
    pub fn contains(&self, point: Point2, tolerance: f64) -> bool {
        (self.curve.project(point) - point).norm() <= tolerance
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
        if speed > LINEAR_TOLERANCE {
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

    /// Returns the two halves of the span either side of a normalized fraction.
    pub fn split_at(&self, fraction: f64) -> (Self, Self) {
        (
            self.sub(Interval::new(0.0, fraction)),
            self.sub(Interval::new(fraction, 1.0)),
        )
    }

    /// Returns the arc length of the span.
    pub fn length(&self) -> f64 {
        self.curve.length(self.interval.start, self.interval.end)
    }

    /// Returns the span translated by `offset`.
    ///
    /// Every support preserves its parameterization under translation, so the
    /// span is carried over unchanged.
    pub fn translated(&self, offset: Vector2<f64>) -> Result<Self, NurbsError> {
        Ok(Self::new(self.curve.translated(offset)?, self.interval))
    }

    /// Returns `segments + 1` points sampled uniformly in traversal fraction.
    pub fn sample(&self, segments: usize) -> Vec<Point2> {
        let segments = segments.max(1);
        (0..=segments)
            .map(|index| self.point_at(index as f64 / segments as f64))
            .collect()
    }

    /// Samples the span densely enough to stay within `tolerance` of it.
    ///
    /// Returned parameters are normalized traversal fractions of the span.
    pub fn adaptive_samples(&self, tolerance: f64, max_depth: usize) -> Vec<(f64, Point2)> {
        let segments = self.sample_count(tolerance, max_depth);
        (0..=segments)
            .map(|index| {
                let fraction = index as f64 / segments as f64;
                (fraction, self.point_at(fraction))
            })
            .collect()
    }

    /// Returns how many chords keep the sag under `tolerance`.
    ///
    /// A straight span needs one; a curved one needs enough that each chord
    /// subtends a small enough turn. The depth limit caps the count the way a
    /// recursive embedding would.
    fn sample_count(&self, tolerance: f64, max_depth: usize) -> usize {
        let depth_limit = 1usize.checked_shl(max_depth.min(20) as u32).unwrap_or(1);
        let radius = match &self.curve {
            Curve2::Line(_) => return 1,
            Curve2::Circle(circle) => circle.radius().abs(),
            Curve2::Ellipse(ellipse) => ellipse
                .major_radius()
                .abs()
                .max(ellipse.minor_radius().abs()),
            Curve2::Nurbs(_) => {
                return self
                    .to_nurbs()
                    .map(|nurbs| nurbs.adaptive_samples(tolerance, max_depth).len().max(2) - 1)
                    .unwrap_or(1)
                    .min(depth_limit);
            }
        };
        let ratio = (1.0 - tolerance / radius.max(tolerance)).clamp(-1.0, 1.0);
        let angle_step = (2.0 * ratio.acos()).max(1.0e-6);
        ((self.interval.delta().abs() / angle_step).ceil() as usize)
            .max(1)
            .min(depth_limit)
    }

    /// Returns the span as a standalone support parameterized over `[0, 1]`.
    ///
    /// This is the cut-down copy: exact, but no longer the analytic support —
    /// an arc comes back as the NURBS that represents it. Use it only where a
    /// curve that *is* the section is required; anywhere the support's identity
    /// matters, keep the [`TrimmedCurve2`].
    pub fn to_curve(&self) -> Result<Curve2, NurbsError> {
        self.curve.trimmed_native(self.interval)
    }

    /// Returns the span as an exact NURBS curve over `[0, 1]`.
    pub fn to_nurbs(&self) -> Result<NurbsCurve2, NurbsError> {
        self.to_curve()?.to_nurbs()
    }

    /// Intersects this span with another using default tolerances.
    ///
    /// Returned parameters and intervals are normalized traversal fractions of
    /// each span.
    pub fn intersect_curve(
        &self,
        other: &TrimmedCurve2,
    ) -> Result<CurveCurveIntersections2, CurveIntersectionError> {
        intersect_curves(self, other)
    }

    /// Intersects this span with another using explicit tolerances.
    pub fn intersect_curve_with_options(
        &self,
        other: &TrimmedCurve2,
        options: CurveIntersectionOptions,
    ) -> Result<CurveCurveIntersections2, CurveIntersectionError> {
        intersect_curves_with_options(self, other, options)
    }
}
