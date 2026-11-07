use super::SketchError;
use super::drawn::{Drawn, Geometry};
use super::joints::{JOINT_CONTACT, Joints};
use super::path::Path;
use crate::geometry::dim2::curves::Circle2;
use crate::geometry::dim2::intersections::CurveCurveIntersection2;
use crate::geometry::dim2::utils::Point2;
use crate::geometry::parameter::Fraction;
use crate::geometry::{IntersectionCoverage, LINEAR_TOLERANCE};
use crate::solver::CurveId;
use std::f64::consts::PI;

/// One stretch of a drawn curve between two nodes.
#[derive(Clone, Copy, Debug)]
pub(super) struct Piece {
    pub curve: CurveId,
    pub path: Path,
    /// The nodes it runs from and to; a circle nothing touches has none.
    pub nodes: Option<(usize, usize)>,
}

/// The curves cut wherever they meet.
pub(super) struct Arrangement {
    /// Every point where something meets something, in order of discovery.
    pub nodes: Vec<Point2>,
    pub pieces: Vec<Piece>,
}

/// One cut along a curve: how far along it, and at which node.
type Cut = (f64, usize);

/// Cuts every curve at its ends and wherever it meets another. Coincidences only say where
/// joined ends sit: curves that meet form a node whether or not anything joins them.
pub(super) fn cut_curves(curves: &[Drawn], joints: &Joints) -> Result<Arrangement, SketchError> {
    let mut nodes = vec![];
    let mut cuts: Vec<Vec<Cut>> = curves
        .iter()
        .map(|curve| {
            curve
                .ends()
                .iter()
                .zip([Fraction::START, Fraction::END])
                .map(|((end, _), fraction)| {
                    (fraction.value(), node_at(&mut nodes, joints.position(*end)))
                })
                .collect()
        })
        .collect();
    cut_at_contacts(curves, &mut nodes, &mut cuts)?;

    let mut pieces = vec![];
    for (curve, mut cuts) in curves.iter().zip(cuts) {
        cuts.sort_by(|a, b| a.0.total_cmp(&b.0));
        cuts.dedup_by_key(|cut| cut.1);
        pieces.extend(pieces_of(curve, &cuts, &nodes)?);
    }
    Ok(Arrangement { nodes, pieces })
}

/// The node at `point`, added if nothing is within contact of it.
fn node_at(nodes: &mut Vec<Point2>, point: Point2) -> usize {
    nodes
        .iter()
        .position(|node| (node - point).norm() <= JOINT_CONTACT)
        .unwrap_or_else(|| {
            nodes.push(point);
            nodes.len() - 1
        })
}

/// Intersects every pair of curves and records where each is cut.
fn cut_at_contacts(
    curves: &[Drawn],
    nodes: &mut Vec<Point2>,
    cuts: &mut [Vec<Cut>],
) -> Result<(), SketchError> {
    let spans: Vec<_> = curves.iter().map(Drawn::span).collect();
    for a in 0..curves.len() {
        for b in a + 1..curves.len() {
            let (first, second) = (curves[a].curve, curves[b].curve);
            let found = spans[a].intersect_curve(&spans[b]).map_err(|source| {
                SketchError::Intersection {
                    first,
                    second,
                    source,
                }
            })?;
            for intersection in found.iter() {
                let CurveCurveIntersection2::Point { point, u_a, u_b } = intersection else {
                    return Err(SketchError::Overlapping { first, second });
                };
                let node = node_at(nodes, *point);
                cuts[a].push((u_a.value(), node));
                cuts[b].push((u_b.value(), node));
            }
            if *found.coverage() != IntersectionCoverage::Complete {
                return Err(SketchError::UncertainCrossing { first, second });
            }
        }
    }
    Ok(())
}

/// The stretches of one curve between its sorted cuts.
fn pieces_of(drawn: &Drawn, cuts: &[Cut], nodes: &[Point2]) -> Result<Vec<Piece>, SketchError> {
    let curve = drawn.curve;
    let node_ids: Vec<usize> = cuts.iter().map(|cut| cut.1).collect();
    let piece = |from: usize, to: usize, path: Path| Piece {
        curve,
        path,
        nodes: Some((from, to)),
    };
    let circle = match &drawn.geometry {
        Geometry::Segment(_) => {
            return Ok(node_ids
                .windows(2)
                .map(|pair| {
                    let (start, end) = (nodes[pair[0]], nodes[pair[1]]);
                    piece(pair[0], pair[1], Path::Line { start, end })
                })
                .collect());
        }
        Geometry::Circle(circle) => circle,
        Geometry::Arc(arc) => arc.curve(),
    };
    let (center, radius) = (circle.center(), circle.radius());
    for &id in &node_ids {
        refuse_off_circle(curve, circle, nodes[id])?;
    }
    let counterclockwise = runs_counterclockwise(drawn, center);
    let closes = match &drawn.geometry {
        Geometry::Arc(arc) => {
            arc.interval().delta().abs() > PI && node_ids.first() == node_ids.last()
        }
        _ => true,
    };

    let mut around = node_ids;
    if closes && around.len() > 1 && around.first() == around.last() {
        around.pop();
    }
    if closes && around.len() < 2 {
        let corner = around.first().copied();
        return Ok(vec![Piece {
            curve,
            path: Path::Circle {
                center,
                radius,
                counterclockwise,
                corner: corner.map(|id| nodes[id]),
            },
            nodes: corner.map(|id| (id, id)),
        }]);
    }
    let mut pairs: Vec<(usize, usize)> = around.windows(2).map(|pair| (pair[0], pair[1])).collect();
    if closes {
        pairs.push((around[around.len() - 1], around[0]));
    }
    Ok(pairs
        .into_iter()
        .map(|(from, to)| {
            let path = Path::arc(center, radius, nodes[from], nodes[to], counterclockwise);
            piece(from, to, path)
        })
        .collect())
}

/// Whether a round curve advances counter-clockwise, reading both its support's handedness
/// and its span's direction.
fn runs_counterclockwise(drawn: &Drawn, center: Point2) -> bool {
    let span = drawn.span();
    let outward = span.point_at(Fraction::START) - center;
    let advance = span.derivative_at(Fraction::START, 1);
    outward.x * advance.y - outward.y * advance.x > 0.
}

fn refuse_off_circle(curve: CurveId, circle: &Circle2, point: Point2) -> Result<(), SketchError> {
    let gap = ((point - circle.center()).norm() - circle.radius()).abs();
    if gap > LINEAR_TOLERANCE {
        return Err(SketchError::CutOffCurve { curve, point, gap });
    }
    Ok(())
}

/// Re-joins the stretches of one curve at every node only they still touch.
///
/// Dropping a curve can leave such a node behind, and it marks nothing any more: a line
/// that overshoots a rectangle and is dropped leaves the side it touched in one piece.
pub(super) fn rejoin_pieces(arrangement: &mut Arrangement) {
    while let Some((node, first, second)) = rejoinable(arrangement) {
        let pieces = &mut arrangement.pieces;
        if first == second {
            let Path::Circle {
                center,
                radius,
                counterclockwise,
                ..
            } = pieces[first].path
            else {
                unreachable!("only a circle runs from a node to itself");
            };
            pieces[first].path = Path::Circle {
                center,
                radius,
                counterclockwise,
                corner: None,
            };
            pieces[first].nodes = None;
            continue;
        }
        // `first` arrives at the node and `second` leaves it.
        let (from, _) = pieces[first].nodes.expect("both pieces touch the node");
        let (arriving, leaving) = pieces[second].nodes.expect("both pieces touch the node");
        debug_assert_eq!(arriving, node);
        pieces[first].path = pieces[first]
            .path
            .joined(pieces[second].path, from == leaving);
        pieces[first].nodes = Some((from, leaving));
        pieces.remove(second);
    }
}

/// A node exactly two piece ends of one curve still touch: the piece arriving and the
/// piece leaving, which are one piece where a circle closes there.
fn rejoinable(arrangement: &Arrangement) -> Option<(usize, usize, usize)> {
    (0..arrangement.nodes.len()).find_map(|node| {
        let mut arriving = vec![];
        let mut leaving = vec![];
        for (index, piece) in arrangement.pieces.iter().enumerate() {
            let Some((from, to)) = piece.nodes else {
                continue;
            };
            if to == node {
                arriving.push(index);
            }
            if from == node {
                leaving.push(index);
            }
        }
        let ([first], [second]) = (&arriving[..], &leaving[..]) else {
            return None;
        };
        let same_curve = arrangement.pieces[*first].curve == arrangement.pieces[*second].curve;
        same_curve.then_some((node, *first, *second))
    })
}
