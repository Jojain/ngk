//! Explicit maps between analytic parameters and exact NURBS parameters.

use std::f64::consts::FRAC_PI_2;

use crate::geometry::{Interval, Point2};

/// A monotone map from one analytic parameter to its NURBS parameter.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Reparam {
    /// The analytic and NURBS parameters are identical.
    Identity,
    /// A piecewise rational-quadratic conic map.
    ///
    /// `source` is the caller-visible parameter interval and `angle` is the
    /// conic angle swept by that interval. `max_span` matches the largest arc
    /// used by the exact conic builder.
    ConicArc {
        source: Interval,
        angle: Interval,
        max_span: f64,
    },
}

impl Reparam {
    /// Creates the standard conic map used by circles and ellipses.
    pub fn conic_arc(source: Interval, angle: Interval) -> Self {
        Self::ConicArc {
            source,
            angle,
            max_span: FRAC_PI_2,
        }
    }

    /// Maps an analytic parameter to the exact NURBS parameter.
    pub fn map(self, parameter: f64) -> f64 {
        match self {
            Self::Identity => parameter,
            Self::ConicArc {
                source,
                angle,
                max_span,
            } => {
                let fraction = interval_fraction(source, parameter);
                let theta = angle.start + (angle.end - angle.start) * fraction;
                conic_parameter(angle, max_span, theta)
            }
        }
    }

    /// Maps a NURBS parameter back to the analytic parameter.
    pub fn inverse(self, parameter: f64) -> f64 {
        match self {
            Self::Identity => parameter,
            Self::ConicArc {
                source,
                angle,
                max_span,
            } => {
                let theta = conic_angle(angle, max_span, parameter);
                source.start + (source.end - source.start) * interval_fraction(angle, theta)
            }
        }
    }
}

/// The independent analytic-to-NURBS maps of a surface patch.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ParamMap {
    pub u: Reparam,
    pub v: Reparam,
}

impl ParamMap {
    /// A map for a parameterization preserved in both directions.
    pub const fn identity() -> Self {
        Self {
            u: Reparam::Identity,
            v: Reparam::Identity,
        }
    }

    /// Maps analytic UV coordinates into the exact NURBS patch domain.
    pub fn map(self, parameter: Point2) -> Point2 {
        Point2::new(self.u.map(parameter.x), self.v.map(parameter.y))
    }

    /// Maps exact NURBS patch coordinates back to analytic UV coordinates.
    pub fn inverse(self, parameter: Point2) -> Point2 {
        Point2::new(self.u.inverse(parameter.x), self.v.inverse(parameter.y))
    }
}

fn interval_fraction(interval: Interval, parameter: f64) -> f64 {
    let length = interval.end - interval.start;
    if length == 0.0 {
        0.0
    } else {
        (parameter - interval.start) / length
    }
}

fn conic_layout(angle: Interval, max_span: f64) -> (Interval, usize, f64, bool) {
    let reversed = angle.end < angle.start;
    let ordered = angle.ordered();
    let span_count = (ordered.length() / max_span).ceil().max(1.0) as usize;
    let span = ordered.length() / span_count as f64;
    (ordered, span_count, span, reversed)
}

fn conic_parameter(angle: Interval, max_span: f64, theta: f64) -> f64 {
    let (ordered, span_count, span, reversed) = conic_layout(angle, max_span);
    let relative = ((theta - ordered.start) / span).clamp(0.0, span_count as f64);
    let index = (relative.floor() as usize).min(span_count - 1);
    let start = ordered.start + index as f64 * span;
    let delta = (theta - start).clamp(0.0, span);
    let weight = (0.5 * span).cos();
    let tangent = (0.5 * delta).tan();
    let full_tangent = (0.5 * span).tan();
    let local = tangent / (weight * full_tangent + tangent * (1.0 - weight));
    let mapped = start + span * local;
    if reversed {
        ordered.start + ordered.end - mapped
    } else {
        mapped
    }
}

fn conic_angle(angle: Interval, max_span: f64, parameter: f64) -> f64 {
    let (ordered, span_count, span, reversed) = conic_layout(angle, max_span);
    let parameter = if reversed {
        ordered.start + ordered.end - parameter
    } else {
        parameter
    };
    let relative = ((parameter - ordered.start) / span).clamp(0.0, span_count as f64);
    let index = (relative.floor() as usize).min(span_count - 1);
    let start = ordered.start + index as f64 * span;
    let local = ((parameter - start) / span).clamp(0.0, 1.0);
    let weight = (0.5 * span).cos();
    let tangent = weight * local * (0.5 * span).tan() / (1.0 - local + weight * local);
    start + 2.0 * tangent.atan()
}
