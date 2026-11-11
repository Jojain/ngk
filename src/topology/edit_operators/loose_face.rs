//! The loose-face operator: one ring of `2n` darts built from an ordered
//! boundary.
//!
//! A face that is not sewn to anything along its boundary is one raw 2-cell
//! walked by `2n` darts: side `k` runs on the pair `(darts[2k], darts[2k+1])`,
//! `alpha0`-linked at the side's corner and `alpha1`-linked from the backward
//! dart to the next side's forward dart. Builders that need such a face — a
//! swept wall, a loft column, an extruded or revolved lateral — hand
//! [`ModelEdit::add_loose_face`] the boundary and get back which dart runs
//! which side, instead of doing the ring arithmetic themselves.

use std::collections::HashMap;

use super::OperatorError;
use crate::geometry::{Curve, Point3, Surface, TrimmedCurve2};
use crate::topology::attributes::{EdgeAttr, FaceAttr, ProfileAttr, VertexAttr};
use crate::topology::edit::{EditKey, ModelEdit, ModelEditError};
use crate::topology::gmap::{Dart, Dim};
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, FaceKey};

/// One side of a loose face's boundary, in loop order.
#[derive(Debug, Clone)]
pub struct LooseFaceSide {
    /// The corner the side runs from, registered as a vertex when present.
    ///
    /// A side whose corner an existing entity already owns — a lateral wall
    /// sewn onto a cap's corner — names none.
    pub corner: Option<LooseFaceCorner>,
    /// The edge the side runs along, registered when present. A side sewn
    /// onto an existing edge names none.
    pub edge: Option<LooseFaceEdge>,
    /// The side's pcurve, keyed on the side's forward dart.
    pub pcurve: TrimmedCurve2,
}

impl LooseFaceSide {
    /// A side that registers its own corner and edge.
    pub fn registered(
        point: Point3,
        corner_origins: Vec<EditKey>,
        curve: Curve,
        edge_origins: Vec<EditKey>,
        pcurve: TrimmedCurve2,
    ) -> Self {
        Self {
            corner: Some(LooseFaceCorner {
                point,
                origins: corner_origins,
            }),
            edge: Some(LooseFaceEdge {
                curve,
                origins: edge_origins,
            }),
            pcurve,
        }
    }

    /// A side whose corner and edge already exist, or arrive by sewing: it
    /// registers neither.
    pub fn sewn(pcurve: TrimmedCurve2) -> Self {
        Self {
            corner: None,
            edge: None,
            pcurve,
        }
    }
}

/// The corner at the start of a [`LooseFaceSide`].
#[derive(Debug, Clone)]
pub struct LooseFaceCorner {
    /// Where the corner sits.
    pub point: Point3,
    /// What the registered vertex is derived from.
    pub origins: Vec<EditKey>,
}

/// The edge a [`LooseFaceSide`] runs along.
#[derive(Debug, Clone)]
pub struct LooseFaceEdge {
    /// The edge's curve, in the side's forward direction.
    pub curve: Curve,
    /// What the registered edge is derived from.
    pub origins: Vec<EditKey>,
}

/// The ordered boundary of one loose face.
#[derive(Debug, Clone)]
pub struct LooseFaceBoundary {
    /// The sides in loop order: side `k` runs from corner `k` to corner
    /// `k + 1`, the last one closing back onto the first.
    pub sides: Vec<LooseFaceSide>,
    /// The surface the face lies on.
    pub surface: Surface,
    /// What the registered profile and face are derived from.
    pub origins: Vec<EditKey>,
}

/// What [`ModelEdit::add_loose_face`] built.
#[derive(Debug, Clone)]
pub struct LooseFace {
    /// The face's key.
    pub face: FaceKey,
    sides: Vec<LooseFaceSideDarts>,
}

/// The darts one side of a [`LooseFace`] runs on.
#[derive(Debug, Clone, Copy)]
struct LooseFaceSideDarts {
    forward: Dart,
    back: Dart,
    edge: Option<EdgeKey>,
}

impl LooseFace {
    /// The dart running side `side` forward: the dart its pcurve is keyed on
    /// and its edge is anchored at.
    pub fn forward(&self, side: usize) -> Dart {
        self.sides[side].forward
    }

    /// The side's backward dart: the forward dart's `alpha0` partner, at the
    /// side's corner.
    pub fn back(&self, side: usize) -> Dart {
        self.sides[side].back
    }

    /// The edge registered on side `side`, when the boundary named one.
    pub fn edge(&self, side: usize) -> Option<EdgeKey> {
        self.sides[side].edge
    }
}

impl<P: Payload> ModelEdit<'_, P> {
    /// Builds one loose face — a single ring of `2n` darts bounding its own
    /// profile and nothing else — from an ordered boundary.
    ///
    /// Side `k` gets two fresh darts: the forward dart, which carries the
    /// side's pcurve and anchors its edge, and the backward dart, its
    /// `alpha0` partner at the side's corner. `alpha1` links each backward
    /// dart to the next side's forward dart, closing the ring. Each side's
    /// corner and edge are registered when the boundary names them, and the
    /// profile and the face are registered derived from `origins`, so the
    /// model is valid when this returns — no deferred block is needed. A
    /// boundary of fewer than three sides cannot close a ring and is refused
    /// with the model untouched.
    pub fn add_loose_face(
        &mut self,
        boundary: LooseFaceBoundary,
    ) -> Result<LooseFace, ModelEditError> {
        let count = boundary.sides.len();
        if count < 3 {
            return Err(OperatorError::TooFewLooseFaceSides { sides: count }.into());
        }
        let darts: Vec<Dart> = (0..2 * count).map(|_| self.add_dart()).collect();
        for i in 0..count {
            self.link(Dim::Zero, darts[2 * i], darts[2 * i + 1])
                .expect("a fresh dart is alpha0-free");
            self.link(Dim::One, darts[2 * i + 1], darts[(2 * i + 2) % (2 * count)])
                .expect("a fresh dart is alpha1-free");
        }
        let mut sides = Vec::with_capacity(count);
        let mut pcurves = HashMap::with_capacity(count);
        for (i, side) in boundary.sides.into_iter().enumerate() {
            let forward = darts[2 * i];
            let back = darts[2 * i + 1];
            if let Some(corner) = side.corner {
                self.add_vertex_derived_from(
                    corner.origins,
                    VertexAttr::new(forward, corner.point),
                );
            }
            let edge = side.edge.map(|edge| {
                self.add_edge_derived_from(edge.origins, EdgeAttr::new(forward, edge.curve))
            });
            pcurves.insert(forward, side.pcurve);
            sides.push(LooseFaceSideDarts {
                forward,
                back,
                edge,
            });
        }
        self.add_profile_derived_from(boundary.origins.clone(), ProfileAttr::new(darts[0]));
        let face = self.add_face_derived_from(
            boundary.origins,
            FaceAttr::with_pcurves(boundary.surface, darts[0], Vec::new(), pcurves),
        );
        Ok(LooseFace { face, sides })
    }
}
