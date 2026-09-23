//! Routing a ring's cut through the holes that wind across it.
//!
//! [`SeamedFace`](super::seam::SeamedFace) cuts a ring face open along one
//! straight parameter line from one rim to the other. That is right while
//! nothing stands in the way, and wrong as soon as a hole does: the strip a
//! thread covers on its core winds round the wall several times, so a straight
//! cut runs through it once per turn. Written that way, the cut is an edge of
//! the face lying across a hole of the same face, and every reader resolves the
//! contradiction differently -- one keeps the wall whole and loses the thread,
//! another drops the wall.
//!
//! **The cut goes through the hole instead of across it.** A ring with one
//! hole cut along two arcs -- one rim to the hole, the hole to the other rim --
//! is a single disk, whatever the hole's shape and however often it winds. So
//! each stretch of cut is replaced by a *bridge* from the rim to the hole's
//! nearest corner, the hole's own edges round to its farthest corner, and a
//! bridge on to the other rim. The two stretches of cut walk the same bridges
//! in opposite directions and the hole's two halves between them, which is
//! what keeps each bridge one seam used twice. Several holes are threaded one
//! after the other in the order the cut meets them.
//!
//! Only the reading changes. The hole's edges are the same edges its other
//! faces name, so nothing is split and no other face learns of it.
//!
//! **A bridge runs along parameter lines**, because a seam has to be written as
//! a curve the support traces exactly: out along the transverse direction, round
//! along the periodic one at the height of the hole corner it serves, which is
//! as far as the hole reaches in that direction and so the one height where
//! running round meets the hole only at that corner. Where a bridge would still
//! meet the boundary anywhere else, the face is refused by name: a cut that
//! crosses the face's own boundary is the file this exists to stop writing.

use crate::geometry::{Axis2, LINEAR_TOLERANCE, Point2};
use crate::topology::attributes::LoopKind;
use crate::topology::face::Face;
use crate::topology::gmap::Dart;
use crate::topology::payload::Payload;
use crate::topology::unwrapped_face_domain::{UnwrappedFaceDomain, UnwrappedFaceDomainLoop};

use super::super::error::TopologyError;
use super::seam::{SeamedBound, SeamedEdge};

/// How finely a boundary is flattened to test a bridge against it, as a
/// fraction of the domain's diagonal.
const PROBE_CHORD: f64 = 1e-4;
const PROBE_DEPTH: usize = 12;

/// A hole the cut runs across, and where the bridges meet it.
struct Crossing<'a> {
    /// Index of the hole among the face's bounds.
    bound: usize,
    darts: &'a [Dart],
    /// The corner the bridge from the rim the cut leaves arrives at.
    entry: usize,
    /// The corner the bridge toward the other rim leaves from.
    exit: usize,
    corners: Vec<Point2>,
}

/// Replaces a ring's straight cut by one threaded through every hole it runs
/// across.
///
/// `bounds` is the seamed face as [`SeamedFace`](super::seam::SeamedFace)
/// derived it, outer first and then one bound per hole, in the order of
/// `domain`'s loops and of `hole_darts`. A face that is not a ring, or whose
/// cut crosses no hole, is left as it is.
pub(super) fn bridge_crossing_holes<P: Payload>(
    face: &Face<'_, P>,
    domain: &UnwrappedFaceDomain,
    hole_darts: &[Vec<Dart>],
    bounds: &mut Vec<SeamedBound>,
) -> Result<(), TopologyError> {
    let Some(axis) = ring_axis(face) else {
        return Ok(());
    };
    let across = axis.transverse();
    let (Some(period), None) = (domain.period(axis), domain.period(across)) else {
        return Ok(());
    };

    let stretches: Vec<usize> = bounds[0]
        .edges
        .iter()
        .enumerate()
        .filter(|(_, edge)| matches!(edge, SeamedEdge::Synthetic { .. }))
        .map(|(index, _)| index)
        .collect();
    let Some(&first) = stretches.first() else {
        return Ok(());
    };
    let SeamedEdge::Synthetic {
        from: leave,
        to: reach,
    } = bounds[0].edges[first]
    else {
        unreachable!("filtered to synthetic stretches");
    };
    let cut = axis.of(leave);

    let chord = PROBE_CHORD * domain.diagonal().max(1.0);
    let placed = domain.loops();
    let rising = across.of(reach) > across.of(leave);
    let mut crossings = Vec::new();
    for (hole, darts) in hole_darts.iter().enumerate() {
        let loop_ = &placed[1 + hole];
        if !runs_across(loop_, axis, cut, period, chord) {
            continue;
        }
        if bounds[1 + hole]
            .edges
            .iter()
            .any(|edge| matches!(edge, SeamedEdge::Synthetic { .. }))
        {
            return Err(refused(
                face,
                "a hole the cut runs across is itself cut open",
            ));
        }
        let corners: Vec<Point2> = loop_.curves().iter().map(|curve| curve.start()).collect();
        let height = |index: &usize| across.of(corners[*index]);
        let lowest = (0..corners.len()).min_by(|a, b| height(a).total_cmp(&height(b)));
        let highest = (0..corners.len()).max_by(|a, b| height(a).total_cmp(&height(b)));
        let (Some(lowest), Some(highest)) = (lowest, highest) else {
            return Err(refused(face, "a hole the cut runs across has no corner"));
        };
        let (entry, exit) = if rising {
            (lowest, highest)
        } else {
            (highest, lowest)
        };
        if entry == exit {
            return Err(refused(
                face,
                "a hole the cut runs across has one corner, so no bridge can reach it and leave it",
            ));
        }
        crossings.push(Crossing {
            bound: 1 + hole,
            darts,
            entry,
            exit,
            corners,
        });
    }
    if crossings.is_empty() {
        return Ok(());
    }

    // A ring's outer bound is its two rims and the two stretches of cut
    // between them, which are one seam walked both ways.
    let &[first, second] = stretches.as_slice() else {
        return Err(refused(
            face,
            "a ring whose cut runs across a hole is cut in more than two stretches",
        ));
    };
    let SeamedEdge::Synthetic { from: back, .. } = bounds[0].edges[second] else {
        unreachable!("filtered to synthetic stretches");
    };
    if (across.of(back) - across.of(reach)).abs() > LINEAR_TOLERANCE {
        return Err(refused(
            face,
            "a ring's two stretches of cut do not run between the same rims",
        ));
    }

    // The cut meets the holes in the order it climbs past their entries.
    crossings.sort_by(|a, b| {
        let (a, b) = (across.of(a.corners[a.entry]), across.of(b.corners[b.entry]));
        if rising {
            a.total_cmp(&b)
        } else {
            b.total_cmp(&a)
        }
    });

    let mut bridges = Vec::with_capacity(crossings.len() + 1);
    let mut at = (leave, false);
    for crossing in &crossings {
        let entry = crossing.corners[crossing.entry];
        bridges.push(bridge(at, (entry, true), axis, period));
        at = (crossing.corners[crossing.exit], true);
    }
    bridges.push(bridge(at, (reach, false), axis, period));

    for legs in &bridges {
        for &(from, to) in legs {
            if meets_boundary(placed, (from, to), legs, axis, period, chord) {
                return Err(refused(
                    face,
                    "no bridge along parameter lines reaches a hole the cut runs across \
                     without crossing the face's boundary",
                ));
            }
        }
    }

    // Out along the bridges and the holes' first halves; back along the same
    // bridges reversed and the holes' other halves.
    let mut outward = Vec::new();
    let mut inward = Vec::new();
    for (index, legs) in bridges.iter().enumerate() {
        outward.extend(
            legs.iter()
                .map(|&(from, to)| SeamedEdge::Synthetic { from, to }),
        );
        if let Some(crossing) = crossings.get(index) {
            outward.extend(half(crossing, crossing.entry, crossing.exit));
        }
    }
    for (index, legs) in bridges.iter().enumerate().rev() {
        inward.extend(
            legs.iter()
                .rev()
                .map(|&(from, to)| SeamedEdge::Synthetic { from: to, to: from }),
        );
        if index > 0 {
            let crossing = &crossings[index - 1];
            inward.extend(half(crossing, crossing.exit, crossing.entry));
        }
    }

    let mut edges = Vec::with_capacity(bounds[0].edges.len() + outward.len() + inward.len());
    for (index, edge) in bounds[0].edges.drain(..).enumerate() {
        if index == first {
            edges.append(&mut outward);
        } else if index == second {
            edges.append(&mut inward);
        } else {
            edges.push(edge);
        }
    }
    bounds[0].edges = edges;

    let mut threaded: Vec<usize> = crossings.iter().map(|crossing| crossing.bound).collect();
    threaded.sort_unstable_by(|a, b| b.cmp(a));
    for bound in threaded {
        bounds.remove(bound);
    }
    Ok(())
}

/// The axis a ring face wraps, when its outer boundary is a pair of wrapping
/// loops.
fn ring_axis<P: Payload>(face: &Face<'_, P>) -> Option<Axis2> {
    let mut axis = None;
    let mut wrapping = 0;
    for loop_ in face.loops() {
        match loop_.kind() {
            LoopKind::Wrapping { axis: wrapped } => {
                axis = Some(wrapped);
                wrapping += 1;
            }
            LoopKind::Inner => {}
            LoopKind::Outer | LoopKind::Capping { .. } => return None,
        }
    }
    (wrapping == 2).then_some(axis).flatten()
}

/// Whether a placed hole reaches across the parameter line the cut runs on.
fn runs_across(
    loop_: &UnwrappedFaceDomainLoop,
    axis: Axis2,
    cut: f64,
    period: f64,
    chord: f64,
) -> bool {
    let (low, high) = loop_
        .adaptive_polyline(chord, PROBE_DEPTH)
        .into_iter()
        .map(|point| axis.of(point))
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(low, high), value| {
            (low.min(value), high.max(value))
        });
    let turn = ((low - cut) / period).ceil();
    cut + turn * period < high
}

/// The stretches of a bridge from one point of the chain to the next.
///
/// Each end is a point with whether it is a hole corner. The bridge runs
/// round at the height of the hole corner it serves -- the entry it arrives at,
/// or the exit it leaves -- and along the transverse direction for the rest.
/// The far end is taken on the branch nearest the near one, so that running
/// round goes the short way.
fn bridge(
    (from, from_hole): (Point2, bool),
    (to, to_hole): (Point2, bool),
    axis: Axis2,
    period: f64,
) -> Vec<(Point2, Point2)> {
    let across = axis.transverse();
    let mut to = to;
    to[axis.index()] -= ((axis.of(to) - axis.of(from)) / period).round() * period;

    let height = match (from_hole, to_hole) {
        (true, _) => across.of(from),
        (false, _) => across.of(to),
    };
    let at = |point: Point2| {
        let mut point = point;
        point[across.index()] = height;
        point
    };
    [(from, at(from)), (at(from), at(to)), (at(to), to)]
        .into_iter()
        .filter(|(start, end)| (end - start).norm() > LINEAR_TOLERANCE)
        .collect()
}

/// One half of a threaded hole: its edges from corner `from` round to `to`.
fn half(crossing: &Crossing<'_>, from: usize, to: usize) -> Vec<SeamedEdge> {
    let count = crossing.darts.len();
    let mut edges = Vec::new();
    let mut index = from;
    while index != to {
        edges.push(SeamedEdge::Real {
            dart: crossing.darts[index],
        });
        index = (index + 1) % count;
    }
    edges
}

/// Whether one stretch of a bridge meets the face's boundary anywhere but at
/// the ends of the bridge it belongs to.
///
/// The test is made in the quotient: every placed boundary is compared at each
/// whole-period translate that could reach the stretch, since a hole winding
/// round the wall is placed over several periods.
fn meets_boundary(
    placed: &[UnwrappedFaceDomainLoop],
    (from, to): (Point2, Point2),
    legs: &[(Point2, Point2)],
    axis: Axis2,
    period: f64,
    chord: f64,
) -> bool {
    let (Some(&(start, _)), Some(&(_, end))) = (legs.first(), legs.last()) else {
        return false;
    };
    let tolerance = chord.max(LINEAR_TOLERANCE);
    let at_an_end = |point: Point2| {
        [start, end].iter().any(|end| {
            let mut gap = point - end;
            gap[axis.index()] -= (gap[axis.index()] / period).round() * period;
            gap.norm() <= tolerance
        })
    };

    // Each placed curve on its own: the gaps between them are the straight cut
    // being replaced, which is no boundary for a bridge to avoid.
    for curve in placed.iter().flat_map(UnwrappedFaceDomainLoop::curves) {
        let points: Vec<Point2> = curve
            .curve()
            .adaptive_samples(chord, PROBE_DEPTH)
            .into_iter()
            .map(|(_, point)| point + curve.offset())
            .collect();
        for pair in points.windows(2) {
            let (a, b) = (pair[0], pair[1]);
            let (low, high) = (axis.of(a).min(axis.of(b)), axis.of(a).max(axis.of(b)));
            let (reach_low, reach_high) = (
                axis.of(from).min(axis.of(to)),
                axis.of(from).max(axis.of(to)),
            );
            let first = ((reach_low - high) / period).floor() as i64;
            let last = ((reach_high - low) / period).ceil() as i64;
            for turn in first..=last {
                let shift = turn as f64 * period;
                let (mut a, mut b) = (a, b);
                a[axis.index()] += shift;
                b[axis.index()] += shift;
                if let Some(hit) = hit_on_line((from, to), (a, b))
                    && !at_an_end(hit)
                {
                    return true;
                }
            }
        }
    }
    false
}

/// Where a segment meets a stretch running along one parameter line, if it
/// does. Overlap along the line counts, and reports the overlap's nearer end
/// to `from`.
fn hit_on_line((from, to): (Point2, Point2), (a, b): (Point2, Point2)) -> Option<Point2> {
    // The stretch varies along exactly one axis; `fixed` is the other.
    let (varies, fixed) = if (to.x - from.x).abs() > (to.y - from.y).abs() {
        (0, 1)
    } else {
        (1, 0)
    };
    let line = from[fixed];
    let (low, high) = (from[varies].min(to[varies]), from[varies].max(to[varies]));
    let (da, db) = (a[fixed] - line, b[fixed] - line);

    if da.abs() <= LINEAR_TOLERANCE && db.abs() <= LINEAR_TOLERANCE {
        // Along the line: they meet where their ranges overlap.
        let (seg_low, seg_high) = (a[varies].min(b[varies]), a[varies].max(b[varies]));
        let (overlap_low, overlap_high) = (low.max(seg_low), high.min(seg_high));
        if overlap_low > overlap_high + LINEAR_TOLERANCE {
            return None;
        }
        let mut hit = from;
        hit[varies] = if from[varies] <= to[varies] {
            overlap_low
        } else {
            overlap_high
        };
        // An overlap that is more than a touch has a point away from the ends.
        if overlap_high - overlap_low > LINEAR_TOLERANCE {
            hit[varies] = 0.5 * (overlap_low + overlap_high);
        }
        return Some(hit);
    }
    if da * db > 0.0 {
        return None;
    }
    let fraction = da / (da - db);
    let value = a[varies] + fraction * (b[varies] - a[varies]);
    if value < low - LINEAR_TOLERANCE || value > high + LINEAR_TOLERANCE {
        return None;
    }
    let mut hit = from;
    hit[varies] = value;
    Some(hit)
}

fn refused<P: Payload>(face: &Face<'_, P>, detail: &str) -> TopologyError {
    TopologyError::UncuttableFace {
        face: face.key(),
        detail: detail.to_string(),
    }
}
