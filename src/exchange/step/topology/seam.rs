//! A face's boundary as STEP has to spell it: closed, and cut open.
//!
//! NGK stores a periodic face seamlessly. A cylinder wall is one ring face
//! with two wrapping loops and no seam edge; a sphere is one face with no
//! boundary at all. STEP has neither: every `ADVANCED_FACE` is bounded by
//! `EDGE_LOOP`s that close in parameter space, so a periodic face has to be
//! handed over with its domain cut open and the cut written as edges.
//!
//! **The cut is derived, never stored.** Un-sewing the map to insert a seam
//! would mutate the user's model in order to serialize it, and would put back
//! exactly the arbitrary, rotation-dependent seam that seamless storage exists
//! to avoid. So the boundary is read through [`UnwrappedFaceDomain`], which
//! already cuts a periodic domain open on demand, reports where the cut fell,
//! and places every loop on one continuous branch.
//!
//! What this module adds is *identity*. An unwrapped domain is parameter-space
//! geometry: it says where the boundary runs, not which edge of the map each
//! piece came from, and a STEP shell is unusable without that — two faces
//! meeting along an edge must name the same `EDGE_CURVE` or the file is a heap
//! of loose faces. The domain places exactly one curve per loop edge, in order,
//! so the identities are recovered by walking the face's loops in the same
//! order and pairing them off. What is left over — the gaps between placed
//! curves — is the synthesized part, and it is synthesized *here* rather than
//! in the writer, which never learns which kind of face it came from.
//!
//! A face with no boundary at all takes a shorter path: there is no loop to
//! place and no identity to recover, so the cut is the edge of the support's
//! own domain and every piece of the boundary is synthesized.

use crate::geometry::{LINEAR_TOLERANCE, Point2};
use crate::topology::attributes::LoopKind;
use crate::topology::face::Face;
use crate::topology::gmap::Dart;
use crate::topology::orientation::Orientation;
use crate::topology::payload::Payload;
use crate::topology::unwrapped_face_domain::{
    UnwrappedFaceDomain, UnwrappedFaceDomainError, UnwrappedFaceDomainLoop,
};

use super::super::error::TopologyError;

/// One piece of a seamed boundary, walked in the loop's own direction.
#[derive(Debug, Clone)]
pub enum SeamedEdge {
    /// An edge the map carries, reached through the dart the loop walks it by.
    Real {
        /// The dart the face's walk enters the edge on.
        dart: Dart,
    },
    /// A stretch of the cut, which the map carries nothing for.
    ///
    /// The two ends are stated in parameter space; the walk runs from `from`
    /// to `to` along the parameter line they share. A synthetic edge whose two
    /// ends have the same image on the surface is a collapsed row — a pole —
    /// and is dropped rather than written, so what survives here always has
    /// extent.
    Synthetic {
        /// Where the cut starts, in the surface's parameter space.
        from: Point2,
        /// Where it ends.
        to: Point2,
    },
}

/// One closed boundary of a seamed face.
#[derive(Debug, Clone)]
pub struct SeamedBound {
    /// Whether this is the boundary the face lies inside of.
    pub outer: bool,
    /// The pieces of the boundary, in traversal order.
    pub edges: Vec<SeamedEdge>,
}

/// A face's boundaries, cut open so that each one closes in parameter space.
#[derive(Debug, Clone)]
pub struct SeamedFace {
    /// The boundaries, the outer one first.
    pub bounds: Vec<SeamedBound>,
}

impl SeamedFace {
    /// Derives the seamed view of a face.
    ///
    /// The outer boundary is one closed walk however many loops the face
    /// stores: a ring's two wrapping loops are joined by the two stretches of
    /// cut between them, which is the same rectangle a stored seam used to
    /// spell out.
    pub fn of_face<P: Payload>(face: &Face<'_, P>) -> Result<Self, TopologyError> {
        if face.loops().is_empty() {
            return Ok(Self {
                bounds: vec![domain_bound(face)?],
            });
        }

        let domain = UnwrappedFaceDomain::of_face(face).map_err(|error| uncuttable(face, error))?;

        // `UnwrappedFaceDomain` fuses every non-hole loop into its first
        // boundary and keeps the holes apart, each in the order the face lists
        // them. Replaying that split is what pairs a placed curve back with the
        // edge it was placed from.
        let mut outer_darts = Vec::new();
        let mut hole_darts = Vec::new();
        for loop_ in face.loops() {
            let darts: Vec<Dart> = loop_.edges().iter().map(|edge| edge.dart()).collect();
            match loop_.kind() {
                LoopKind::Inner => hole_darts.push(darts),
                _ => outer_darts.extend(darts),
            }
        }

        let mut bounds = Vec::with_capacity(1 + hole_darts.len());
        let mut placed = domain.loops().iter();
        let outer = placed.next().ok_or_else(|| TopologyError::UncuttableFace {
            face: face.key(),
            detail: "an unwrapped domain has no boundaries at all".to_string(),
        })?;
        bounds.push(seamed_bound(face, outer, &outer_darts, true)?);
        for (hole, darts) in placed.zip(&hole_darts) {
            bounds.push(seamed_bound(face, hole, darts, false)?);
        }

        Ok(Self { bounds })
    }
}

/// Cuts a face with no boundary at all open along the edge of its domain.
///
/// Such a face covers a closed support — a whole sphere, a whole torus — so
/// there is no loop to place and nothing to pair identities with: the entire
/// boundary is cut. The cut is the domain rectangle itself, walked so the face
/// lies to its left: counter-clockwise in parameter space when the face faces
/// the way its support does, clockwise when it faces the other way. A side
/// running along a collapsed row — a sphere's pole — bounds nothing and is
/// dropped, which is what leaves a sphere with the two meridian walks and a
/// pole at each end that a stored model spells out.
fn domain_bound<P: Payload>(face: &Face<'_, P>) -> Result<SeamedBound, TopologyError> {
    let (u, v) = face.surface().domain();
    if !u.is_finite() || !v.is_finite() {
        return Err(TopologyError::UncuttableFace {
            face: face.key(),
            detail: "a face with no boundary runs off the edge of its domain".to_string(),
        });
    }

    let mut corners = [
        Point2::new(u.start, v.start),
        Point2::new(u.end, v.start),
        Point2::new(u.end, v.end),
        Point2::new(u.start, v.end),
    ];
    if face.sense() == Orientation::Reversed {
        corners.reverse();
    }

    let mut edges = Vec::with_capacity(corners.len());
    for index in 0..corners.len() {
        let (from, to) = (corners[index], corners[(index + 1) % corners.len()]);
        if collapses(face, from, to) {
            continue;
        }
        edges.push(SeamedEdge::Synthetic { from, to });
    }
    if edges.is_empty() {
        return Err(TopologyError::UncuttableFace {
            face: face.key(),
            detail: "every side of the domain collapses, leaving no cut to write".to_string(),
        });
    }

    Ok(SeamedBound { outer: true, edges })
}

/// Pairs one placed boundary with the darts it was placed from.
fn seamed_bound<P: Payload>(
    face: &Face<'_, P>,
    placed: &UnwrappedFaceDomainLoop,
    darts: &[Dart],
    outer: bool,
) -> Result<SeamedBound, TopologyError> {
    let curves = placed.curves();
    if curves.len() != darts.len() {
        // The pairing is positional, so a count that does not line up means
        // the domain placed something this walk did not produce. Refusing is
        // the only safe answer: carrying on would attach a pcurve to the wrong
        // edge, which no validator downstream can catch.
        return Err(TopologyError::UncuttableFace {
            face: face.key(),
            detail: format!(
                "the unwrapped domain placed {} curves for {} boundary edges",
                curves.len(),
                darts.len(),
            ),
        });
    }

    let mut edges = Vec::with_capacity(curves.len());
    for (curve, &dart) in curves.iter().zip(darts) {
        // A curve's corners are the points the boundary turns through on its
        // way from the previous curve's end to this one's start. The first is
        // that previous end; the rest are places the cut bends, such as the
        // trip out to a collapsed row and back.
        let mut gap: Vec<Point2> = curve.corners().to_vec();
        if !gap.is_empty() {
            gap.push(curve.start());
        }
        for pair in gap.windows(2) {
            let (from, to) = (pair[0], pair[1]);
            if collapses(face, from, to) {
                continue;
            }
            edges.push(SeamedEdge::Synthetic { from, to });
        }
        edges.push(SeamedEdge::Real { dart });
    }

    Ok(SeamedBound { outer, edges })
}

/// Whether a stretch of cut bounds nothing, and so carries no edge.
///
/// Two ways for that to happen, and both are ordinary rather than exceptional.
/// A stretch with no extent in parameter space is a loop that already closed:
/// a circular parameter curve ends a rounding error away from where it began,
/// and the domain records that as a gap because it compares exactly. A stretch
/// running along a collapsed row is a pole — one surface point carrying a whole
/// row of parameters — which the boundary must travel *through* and cannot
/// travel *along*. Dropping either joins the two neighbours at one corner,
/// which is what a stored model spells out as a pole vertex.
///
/// The row is sampled in the middle as well as at the ends, because the ends
/// alone do not tell the two cases apart: a sphere's domain rectangle runs one
/// side from pole to pole, degenerate at both ends and a whole meridian in
/// between.
fn collapses<P: Payload>(face: &Face<'_, P>, from: Point2, to: Point2) -> bool {
    if (to - from).norm() <= LINEAR_TOLERANCE {
        return true;
    }
    let surface = face.surface();
    [from, from.lerp(&to, 0.5), to]
        .iter()
        .all(|point| surface.is_degenerate_at(point.x, point.y))
}

fn uncuttable<P: Payload>(face: &Face<'_, P>, error: UnwrappedFaceDomainError) -> TopologyError {
    TopologyError::UncuttableFace {
        face: face.key(),
        detail: error.to_string(),
    }
}
