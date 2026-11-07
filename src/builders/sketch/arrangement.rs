use super::SketchError;
use super::drawn::Drawn;
use super::joints::Joints;
use super::path::Path;
use super::pieces::{Arrangement, Piece, cut_curves, rejoin_pieces};
use crate::geometry::dim2::utils::Point2;
use crate::solver::CurveId;
use std::f64::consts::{PI, TAU};

/// How close two tangents leaving one node may be and still be the same direction. Curves
/// leaving a joint tangentially agree only as well as the solver holds them.
const TANGENT_TIE: f64 = 1e-6;

/// One stretch of a region's boundary, in the direction the region walks it.
#[derive(Clone, Copy, Debug)]
pub(super) struct Stretch {
    pub curve: CurveId,
    pub path: Path,
}

/// A closed area of the plane the curves enclose: counter-clockwise outside, clockwise
/// round each hole, so the region lies left of every stretch.
#[derive(Clone, Debug)]
pub(super) struct Region {
    pub outer: Vec<Stretch>,
    pub holes: Vec<Vec<Stretch>>,
}

/// The closed areas the curves enclose. What bounds no area, an overshooting end or a
/// segment with one region on both sides, is left out.
pub(super) fn regions(curves: &[Drawn], joints: &Joints) -> Result<Vec<Region>, SketchError> {
    let mut arrangement = cut_curves(curves, joints)?;
    drop_what_bounds_nothing(&mut arrangement);
    rejoin_pieces(&mut arrangement);
    classify(&arrangement)
}

/// A piece walked one way or the other: `2 * index` along it, `2 * index + 1` against it.
type HalfPiece = usize;

fn walked(pieces: &[Piece], half: HalfPiece) -> Path {
    let path = pieces[half / 2].path;
    if half.is_multiple_of(2) {
        path
    } else {
        path.reversed()
    }
}

/// The walks that keep their region on their left, each turning as sharply left as it can
/// at every node.
fn walks(arrangement: &Arrangement) -> Vec<Vec<HalfPiece>> {
    let pieces = &arrangement.pieces;
    let leaving = leaving_each_node(arrangement);
    let next = |half: HalfPiece| -> HalfPiece {
        let Some((from, to)) = pieces[half / 2].nodes else {
            return half;
        };
        let arrival = if half.is_multiple_of(2) { to } else { from };
        let around = &leaving[arrival];
        let back = half ^ 1;
        let at = around
            .iter()
            .position(|candidate| *candidate == back)
            .expect("a piece's far end leaves the node it arrives at");
        around[(at + around.len() - 1) % around.len()]
    };
    let mut seen = vec![false; pieces.len() * 2];
    let mut walks = vec![];
    for first in 0..seen.len() {
        if seen[first] {
            continue;
        }
        let mut walk = vec![];
        let mut half = first;
        while !seen[half] {
            seen[half] = true;
            walk.push(half);
            half = next(half);
        }
        walks.push(walk);
    }
    walks
}

/// The half-pieces leaving each node, in counter-clockwise order.
fn leaving_each_node(arrangement: &Arrangement) -> Vec<Vec<HalfPiece>> {
    let mut leaving = vec![vec![]; arrangement.nodes.len()];
    for (index, piece) in arrangement.pieces.iter().enumerate() {
        if let Some((from, to)) = piece.nodes {
            leaving[from].push(2 * index);
            leaving[to].push(2 * index + 1);
        }
    }
    for around in &mut leaving {
        order_counterclockwise(around, &arrangement.pieces);
    }
    leaving
}

/// Sorts by tangent direction. Paths leaving along one tangent separate by curvature: the
/// more a path curves left, the further counter-clockwise it lies just after the node.
fn order_counterclockwise(around: &mut Vec<HalfPiece>, pieces: &[Piece]) {
    let mut keyed: Vec<(f64, f64, HalfPiece)> = around
        .iter()
        .map(|half| {
            let (angle, curvature) = walked(pieces, *half).leaving();
            // Directions either side of the cut in `atan2` are one direction.
            let angle = if angle > PI - TANGENT_TIE {
                angle - TAU
            } else {
                angle
            };
            (angle, curvature, *half)
        })
        .collect();
    keyed.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut start = 0;
    while start < keyed.len() {
        let mut end = start + 1;
        while end < keyed.len() && keyed[end].0 - keyed[end - 1].0 <= TANGENT_TIE {
            end += 1;
        }
        keyed[start..end].sort_by(|a, b| a.1.total_cmp(&b.1));
        start = end;
    }
    *around = keyed.into_iter().map(|(_, _, half)| half).collect();
}

/// Removes every piece one walk runs both ways: it has the same area on both sides.
fn drop_what_bounds_nothing(arrangement: &mut Arrangement) {
    loop {
        let mut walk_of = vec![0; arrangement.pieces.len() * 2];
        for (index, walk) in walks(arrangement).iter().enumerate() {
            for half in walk {
                walk_of[*half] = index;
            }
        }
        let before = arrangement.pieces.len();
        let mut index = 0;
        arrangement.pieces.retain(|_| {
            let keep = walk_of[2 * index] != walk_of[2 * index + 1];
            index += 1;
            keep
        });
        if arrangement.pieces.len() == before {
            return;
        }
    }
}

/// One closed walk with what classifying it needs.
struct Walk {
    halves: Vec<HalfPiece>,
    area: f64,
    /// Which connected group of pieces it belongs to.
    group: usize,
}

/// Walks with positive area are outer boundaries. The others are the outside of a
/// connected group of pieces, which is a hole of whichever region contains that group.
fn classify(arrangement: &Arrangement) -> Result<Vec<Region>, SketchError> {
    let pieces = &arrangement.pieces;
    let group_of = connected_groups(arrangement);
    let classified: Vec<Walk> = walks(arrangement)
        .into_iter()
        .map(|halves| Walk {
            area: halves
                .iter()
                .map(|half| walked(pieces, *half).signed_area())
                .sum(),
            group: group_of[halves[0] / 2],
            halves,
        })
        .collect();
    let (outers, others): (Vec<_>, Vec<_>) =
        classified.into_iter().partition(|walk| walk.area > 0.);

    let mut regions = Vec::with_capacity(outers.len());
    for outer in &outers {
        refuse_pinch(outer, arrangement)?;
        regions.push(Region {
            outer: stretches(outer, pieces),
            holes: vec![],
        });
    }
    for other in &others {
        let point = walked(pieces, other.halves[0]).point();
        let container = outers
            .iter()
            .enumerate()
            .filter(|(_, outer)| outer.group != other.group && contains(outer, pieces, point))
            .min_by(|(_, a), (_, b)| a.area.total_cmp(&b.area));
        if let Some((index, _)) = container {
            refuse_pinch(other, arrangement)?;
            regions[index].holes.push(stretches(other, pieces));
        }
    }
    Ok(regions)
}

fn stretches(walk: &Walk, pieces: &[Piece]) -> Vec<Stretch> {
    walk.halves
        .iter()
        .map(|half| Stretch {
            curve: pieces[half / 2].curve,
            path: walked(pieces, *half),
        })
        .collect()
}

/// Whether a walk encloses a point that is not on it.
fn contains(walk: &Walk, pieces: &[Piece], point: Point2) -> bool {
    let turned: f64 = walk
        .halves
        .iter()
        .map(|half| walked(pieces, *half).subtended_at(point))
        .sum();
    (turned / TAU).round() != 0.
}

/// A boundary loop visits each corner once, so a walk through a node twice has no loop.
fn refuse_pinch(walk: &Walk, arrangement: &Arrangement) -> Result<(), SketchError> {
    let mut seen = vec![false; arrangement.nodes.len()];
    for half in &walk.halves {
        let Some((from, to)) = arrangement.pieces[half / 2].nodes else {
            continue;
        };
        let node = if half.is_multiple_of(2) { from } else { to };
        if std::mem::replace(&mut seen[node], true) {
            return Err(SketchError::PinchedRegion {
                point: arrangement.nodes[node],
            });
        }
    }
    Ok(())
}

/// Labels each piece with its connected group: pieces sharing a node, directly or not.
fn connected_groups(arrangement: &Arrangement) -> Vec<usize> {
    let pieces = &arrangement.pieces;
    let mut group = vec![usize::MAX; pieces.len()];
    for start in 0..pieces.len() {
        if group[start] != usize::MAX {
            continue;
        }
        group[start] = start;
        let mut pending = vec![start];
        while let Some(index) = pending.pop() {
            for other in 0..pieces.len() {
                if group[other] == usize::MAX && share_a_node(&pieces[index], &pieces[other]) {
                    group[other] = start;
                    pending.push(other);
                }
            }
        }
    }
    group
}

fn share_a_node(a: &Piece, b: &Piece) -> bool {
    let (Some((a0, a1)), Some((b0, b1))) = (a.nodes, b.nodes) else {
        return false;
    };
    a0 == b0 || a0 == b1 || a1 == b0 || a1 == b1
}
