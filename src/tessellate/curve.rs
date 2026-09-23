//! Curve sampling: any [`Curve`] from `t0` to `t1` → [`Polyline3`].
//!
//! An edge and every face it bounds must meet at the same points, or the edge
//! line sinks into the faces and the faces open up along it. So a curve span
//! is sampled at a count read off the span alone -- [`segments_for`] -- and at
//! uniform fractions of it, which is a set of points any caller holding the
//! same span arrives at, whichever way round it walks it.

use std::f64::consts::TAU;

use super::{CurveOpts, Polyline3};
use crate::geometry::parameter::{Fraction, NativeParam};
use crate::geometry::{Curve, TrimmedCurve};

/// Probes read along a span to measure how far its tangent turns.
const TURNING_PROBES: usize = 256;
/// Most segments one span is ever cut into.
const MAX_SEGMENTS: usize = 8192;

/// Sample `curve` uniformly between `t0` and `t1` into `opts.segments + 1`
/// points. The endpoints are always present.
pub fn tessellate_curve(curve: &Curve, t0: f64, t1: f64, opts: CurveOpts) -> Polyline3 {
    let segments = opts.segments.max(1);
    let mut points = Vec::with_capacity(segments + 1);
    for i in 0..=segments {
        let t = t0 + (t1 - t0) * (i as f64 / segments as f64);
        points.push(curve.point_at(NativeParam::new(t)));
    }
    Polyline3 { points }
}

/// Samples a span at [`segments_for`] uniform fractions.
pub fn tessellate_span(span: &TrimmedCurve, opts: CurveOpts) -> Polyline3 {
    let segments = segments_for(span, opts);
    Polyline3 {
        points: (0..=segments)
            .map(|index| span.point_at(Fraction::new(index as f64 / segments as f64)))
            .collect(),
    }
}

/// How many segments `span` is cut into: enough that its tangent turns
/// through at most `1 / opts.segments` of a whole turn along any one of them.
///
/// Read off the span's own geometry and nothing else, so an edge and each
/// face along it ask the same question and get the same answer. A whole
/// circle gets `opts.segments`, as a fixed count would give it; a line gets
/// one; a helix gets `opts.segments` for every turn it makes rather than for
/// all of them together.
pub fn segments_for(span: &TrimmedCurve, opts: CurveOpts) -> usize {
    let per_turn = opts.segments.max(1) as f64;
    let mut probes = TURNING_PROBES;
    loop {
        let (turning, widest) = turning(span, probes);
        // A probe spacing the tangent turns a large way across could be
        // skipping whole turns between probes; read it again, finer.
        if widest <= TAU / 16.0 || probes >= MAX_SEGMENTS {
            let segments = (turning / (TAU / per_turn) - 1.0e-9).ceil();
            return (segments.max(1.0) as usize).min(MAX_SEGMENTS);
        }
        probes *= 4;
    }
}

/// The total angle `span`'s tangent turns through over `probes` steps, and
/// the widest turn across any one step.
fn turning(span: &TrimmedCurve, probes: usize) -> (f64, f64) {
    let tangent = |index: usize| {
        let direction = span.derivative_at(Fraction::new(index as f64 / probes as f64), 1);
        let length = direction.norm();
        (length > 0.0).then(|| direction / length)
    };
    let (mut total, mut widest) = (0.0_f64, 0.0_f64);
    let mut previous = tangent(0);
    for index in 1..=probes {
        let current = tangent(index);
        if let (Some(from), Some(to)) = (previous, current) {
            let angle = from.dot(&to).clamp(-1.0, 1.0).acos();
            total += angle;
            widest = widest.max(angle);
        }
        previous = current.or(previous);
    }
    (total, widest)
}
