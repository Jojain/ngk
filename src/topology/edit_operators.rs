//! Edit operators: changes to the map that leave the model valid.
//!
//! The primitives of [`ModelEdit`] (`add_dart`, `link`, `unlink`, `sew`,
//! `own_cell`, ...) rewire one thing each, so a sequence of them passes through
//! states where the map and the labelling disagree. An operator is the whole
//! sequence: it reads what it needs before it touches anything, refuses a
//! precondition that does not hold, performs the rewiring *and* the relabelling,
//! and checks the cells it touched before returning. A view read between two
//! operators therefore sees a model whose labelling describes its map, which is
//! what the lazily derived embedding index requires.
//!
//! The set is small on purpose, each member the unit a builder needs and no
//! smaller: [`ModelEdit::mark_edge`] and [`ModelEdit::cut_edge`] add a corner to
//! an edge, [`ModelEdit::sew_edges`] and [`ModelEdit::unsew_edge`] fuse and
//! separate two edges where faces meet. Geometry stays with the caller -- an
//! operator is handed the curves and points the new entities carry.

use std::collections::{HashMap, HashSet};

use thiserror::Error;

use super::attributes::{EdgeAttr, VertexAttr};
use super::edge::Edge;
use super::edit::{EditKey, ModelEdit, ModelEditError};
use super::embedding::EntityOwner;
use super::gmap::{Dart, Dim};
use super::payload::Payload;
use super::shape_keys::{EdgeKey, VertexKey};
use super::vertex::Vertex;
use crate::geometry::{Curve, Point3};

/// Why an operator refused, before it changed anything.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum OperatorError {
    /// The edge is not in the model.
    #[error("edge {edge:?} does not exist")]
    MissingEdge { edge: EdgeKey },

    /// The dart belongs to no registered edge.
    #[error("dart {dart:?} belongs to no edge")]
    NotAnEdgeDart { dart: Dart },

    /// An operator that needs a marked or bounded edge was given an unmarked
    /// one, which has no corner to separate it from.
    #[error("edge {edge:?} has no corner: mark it before cutting it")]
    EdgeHasNoCorner { edge: EdgeKey },

    /// An operator that marks an edge was given one that already has a corner.
    #[error("edge {edge:?} is not an unmarked edge")]
    EdgeNotUnmarked { edge: EdgeKey },

    /// The raw 0-cell where an unmarked edge closes is not labelled as that
    /// edge's own.
    #[error("the 0-cell where edge {edge:?} closes is labelled {held:?}")]
    ClosureNotOwned {
        edge: EdgeKey,
        held: Option<EntityOwner>,
    },

    /// Two edges whose cells do not match one another cannot be sewn.
    #[error("darts {first:?} and {second:?} cannot be sewn along their edges")]
    NotSewable { first: Dart, second: Dart },

    /// The dart is not sewn to anything along its edge.
    #[error("the edge at dart {dart:?} is not sewn to another")]
    NotSewn { dart: Dart },

    /// Unsewing is defined for an edge shared by exactly two sides, each
    /// reaching the edge's two corners once.
    #[error("the edge at dart {dart:?} is shared by more than two sides")]
    SharedByMoreThanTwoSides { dart: Dart },

    /// A cell the operator would split or fuse carries a label of a kind the
    /// operator does not know how to carry across.
    #[error("the {dimension:?}-cell at {dart:?} is claimed by {claims:?}")]
    ContestedCell {
        dimension: Dim,
        dart: Dart,
        claims: Vec<EntityOwner>,
    },
}

/// Where an edge is cut, with the geometry the two halves carry.
#[derive(Debug, Clone)]
pub struct EdgeCutPlan {
    /// The corner the cut adds.
    pub point: Point3,
    /// The curve of the half that keeps the edge's key, in the direction of the
    /// edge's stored dart.
    pub first: Curve,
    /// The curve of the half beyond the cut, which gets a new key.
    pub second: Curve,
}

/// What [`ModelEdit::cut_edge`] made.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EdgeCutResult {
    /// The half before the cut, under the key the edge had.
    pub first: EdgeKey,
    /// The half beyond the cut, under a key split from the first.
    pub second: EdgeKey,
    /// The corner the two halves meet at.
    pub vertex: VertexKey,
}

/// What [`ModelEdit::sew_edges`] fused.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EdgeSewing {
    /// The edge both sides now belong to.
    pub edge: EdgeKey,
    /// The vertices that were fused, as `(survivor, removed)`.
    pub vertices: Vec<(VertexKey, VertexKey)>,
}

/// What [`ModelEdit::unsew_edge`] separated.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EdgeUnsewing {
    /// The edge, still under the key it had, on the side holding its anchor.
    pub kept: EdgeKey,
    /// The edge of the other side, under a key split from the first.
    pub created: EdgeKey,
    /// The vertices the separation had to duplicate, as `(kept, created)`.
    pub vertices: Vec<(VertexKey, VertexKey)>,
}

impl<P: Payload> ModelEdit<'_, P> {
    /// Gives an unmarked edge the corner a cut asked for.
    ///
    /// An unmarked edge is a closed edge no vertex sits on: the 0-cell where its
    /// parameterization closes is scaffold, labelled as the edge's own. Marking
    /// it places a vertex at `point` on that same 0-cell and drops the label,
    /// in one step. No dart is added and nothing is linked: one cut cannot
    /// separate an edge that has no second corner to separate it from.
    pub fn mark_edge(&mut self, edge: EdgeKey, point: Point3) -> Result<VertexKey, ModelEditError> {
        let dart = self
            .model
            .edge_attr(edge)
            .ok_or(OperatorError::MissingEdge { edge })?
            .dart;
        let far = self.model.alpha(Dim::Zero, dart);
        let closed = self.model.cell_representative(dart, Dim::Zero)
            == self.model.cell_representative(far, Dim::Zero);
        if !closed || Vertex::from_dart(self.model, dart).is_some() {
            return Err(OperatorError::EdgeNotUnmarked { edge }.into());
        }
        let claims = self.claims_on_cell(Dim::Zero, dart);
        if claims != [EntityOwner::Edge(edge)] {
            return Err(OperatorError::ClosureNotOwned {
                edge,
                held: claims.first().copied(),
            }
            .into());
        }
        self.disown_cell(Dim::Zero, dart);
        Ok(self.add_vertex_derived_from(vec![EditKey::Edge(edge)], VertexAttr::new(dart, point)))
    }

    /// Cuts an edge that has a corner into two edges meeting at a new corner.
    ///
    /// Every dart of the edge's 1-cell is doubled, so the edge keeps all the
    /// faces that run along it: each side of the cell gets a middle dart
    /// `alpha0`-linked in between, the two middle darts of a pair are joined by
    /// `alpha1` to make the new 0-cell, and `alpha2` partners are mirrored on
    /// the middle darts. The 0-cells at the edge's old ends keep their darts, so
    /// the labelling is untouched; the new 0-cell is born with its vertex.
    pub fn cut_edge(
        &mut self,
        edge: EdgeKey,
        plan: EdgeCutPlan,
    ) -> Result<EdgeCutResult, ModelEditError> {
        let first_dart = self
            .model
            .edge_attr(edge)
            .ok_or(OperatorError::MissingEdge { edge })?
            .dart;
        if Vertex::from_dart(self.model, first_dart).is_none() {
            return Err(OperatorError::EdgeHasNoCorner { edge }.into());
        }
        let second_dart = self.model.alpha(Dim::Zero, first_dart);
        let cell: Vec<Dart> = self
            .model
            .orbit(first_dart, self.model.orbit_indices(Dim::One))
            .collect();
        let alpha0_pairs = self.pairs_within(&cell, Dim::Zero);
        let alpha2_pairs = self.pairs_within(&cell, Dim::Two);
        let middle: HashMap<Dart, Dart> = cell.iter().map(|&dart| (dart, self.add_dart())).collect();

        for (first, second) in alpha0_pairs {
            self.unlink(Dim::Zero, first)?;
            self.link(Dim::Zero, first, middle[&first])?;
            self.link(Dim::Zero, second, middle[&second])?;
            self.link(Dim::One, middle[&first], middle[&second])?;
        }
        for (first, second) in alpha2_pairs {
            self.link(Dim::Two, middle[&first], middle[&second])?;
        }

        let vertex = self.add_vertex_derived_from(
            vec![EditKey::Edge(edge)],
            VertexAttr::new(middle[&first_dart], plan.point),
        );
        self.edge_attr_mut_unchecked(edge).curve = plan.first;
        let second = self.add_edge_split_from(edge, EdgeAttr::new(middle[&second_dart], plan.second));
        Ok(EdgeCutResult {
            first: edge,
            second,
            vertex,
        })
    }

    /// Sews the 2-cells that `survivor` and `removed` lie on along their edges,
    /// fusing the edges and the vertices at their ends.
    ///
    /// The two edges become `survivor`'s edge and each pair of end vertices
    /// becomes the one of `survivor` (when both ends carry one). The 0-cells
    /// fuse with the sew, and so does what labels them: a closed edge whose
    /// closure is scaffold stays scaffold of the fused edge, unless a vertex
    /// now sits on that 0-cell -- a vertex marks the closure, and an edge that
    /// is marked has no scaffold there. Sewing an unmarked edge to a marked one
    /// therefore yields a marked edge.
    ///
    /// Refuses, with the model untouched, darts that are not on edges, edges
    /// that do not match cell for cell, and 0-cells claimed by anything but the
    /// two edges.
    pub fn sew_edges(&mut self, survivor: Dart, removed: Dart) -> Result<EdgeSewing, ModelEditError> {
        let edge_at = |dart: Dart| {
            Edge::from_dart(self.model, dart)
                .map(|edge| edge.key())
                .ok_or(OperatorError::NotAnEdgeDart { dart })
        };
        let survivor_edge = edge_at(survivor)?;
        let removed_edge = edge_at(removed)?;
        let ends = [
            (survivor, removed),
            (
                self.model.alpha(Dim::Zero, survivor),
                self.model.alpha(Dim::Zero, removed),
            ),
        ];
        let vertex_pairs: Vec<(VertexKey, VertexKey)> = ends
            .iter()
            .filter_map(|&(keep, drop)| {
                Some((
                    Vertex::from_dart(self.model, keep)?.key(),
                    Vertex::from_dart(self.model, drop)?.key(),
                ))
            })
            .collect();
        for (keep, drop) in ends {
            for dart in [keep, drop] {
                let claims = self.claims_on_cell(Dim::Zero, dart);
                let own = |owner: &EntityOwner| {
                    matches!(owner, EntityOwner::Vertex(_))
                        || *owner == EntityOwner::Edge(survivor_edge)
                        || *owner == EntityOwner::Edge(removed_edge)
                };
                if !claims.iter().all(own) {
                    return Err(OperatorError::ContestedCell {
                        dimension: Dim::Zero,
                        dart,
                        claims,
                    }
                    .into());
                }
            }
        }
        // A closed edge cannot be sewn to one that has two ends: the map would
        // allow it, and fuse the two corners of one into the single corner of
        // the other.
        let closed = |dart: Dart| {
            self.model.cell_representative(dart, Dim::Zero)
                == self
                    .model
                    .cell_representative(self.model.alpha(Dim::Zero, dart), Dim::Zero)
        };
        if closed(survivor) != closed(removed)
            || self.model.is_sewable(survivor, removed, Dim::Two).is_none()
        {
            return Err(OperatorError::NotSewable {
                first: survivor,
                second: removed,
            }
            .into());
        }

        self.sew(Dim::Two, survivor, removed)?;
        if survivor_edge != removed_edge {
            self.merge_edges_into(survivor_edge, removed_edge);
        }
        let mut merged = HashSet::<VertexKey>::new();
        let mut fused = Vec::new();
        for (keep, drop) in vertex_pairs {
            if keep != drop && merged.insert(drop) {
                self.merge_vertices_into(keep, drop);
                fused.push((keep, drop));
            }
        }
        if survivor_edge != removed_edge {
            for (dart, _) in ends {
                self.settle_closure(dart, survivor_edge)?;
            }
        }
        Ok(EdgeSewing {
            edge: survivor_edge,
            vertices: fused,
        })
    }

    /// Separates the two sides of an edge two 2-cells share, giving the side of
    /// `dart` its own edge.
    ///
    /// The inverse of [`Self::sew_edges`] for an edge shared by exactly two
    /// sides: the `alpha2` links along the edge are cut, and each 0-cell at its
    /// ends that no longer holds together gets a vertex of its own, split from
    /// the one it shared. The key stays with the side that holds the edge's
    /// anchor dart; the other side gets a new key carrying the same curve.
    ///
    /// Refuses an edge that is not sewn, one shared by more than two sides, and
    /// one whose end 0-cells are claimed by anything but their vertices.
    pub fn unsew_edge(&mut self, dart: Dart) -> Result<EdgeUnsewing, ModelEditError> {
        let edge = Edge::from_dart(self.model, dart)
            .map(|edge| edge.key())
            .ok_or(OperatorError::NotAnEdgeDart { dart })?;
        let partner = self.model.alpha(Dim::Two, dart);
        if partner == dart {
            return Err(OperatorError::NotSewn { dart }.into());
        }
        let far = self.model.alpha(Dim::Zero, dart);
        let far_partner = self.model.alpha(Dim::Two, far);
        let cell: HashSet<Dart> = self
            .model
            .orbit(dart, self.model.orbit_indices(Dim::One))
            .collect();
        if cell != HashSet::from([dart, partner, far, far_partner]) {
            return Err(OperatorError::SharedByMoreThanTwoSides { dart }.into());
        }
        for end in [dart, far] {
            let claims = self.claims_on_cell(Dim::Zero, end);
            if !claims.iter().all(|owner| matches!(owner, EntityOwner::Vertex(_))) {
                return Err(OperatorError::ContestedCell {
                    dimension: Dim::Zero,
                    dart: end,
                    claims,
                }
                .into());
            }
        }

        let (anchor, curve) = {
            let attr = self.model.edge_attr_unchecked(edge);
            (attr.dart, attr.curve.clone())
        };
        // The other side's dart at the same end of the edge as `of`.
        let counterpart = |of: Dart| match of {
            of if of == dart => partner,
            of if of == partner => dart,
            of if of == far => far_partner,
            _ => far,
        };
        // The vertices at the two ends, read before the cells split.
        let ends = [(dart, partner), (far, far_partner)];
        let mut seen = HashSet::<VertexKey>::new();
        let vertices_before: Vec<(Dart, Dart, VertexKey, Point3)> = ends
            .iter()
            .filter_map(|&(here, there)| {
                let vertex = Vertex::from_dart(self.model, here)?;
                seen.insert(vertex.key())
                    .then(|| (here, there, vertex.key(), *vertex.point()))
            })
            .collect();

        self.unlink(Dim::Two, dart)?;
        self.unlink(Dim::Two, far)?;
        let created = self.add_edge_split_from(edge, EdgeAttr::new(counterpart(anchor), curve));

        let mut duplicated = Vec::new();
        for (here, there, vertex, point) in vertices_before {
            let cell: Vec<Dart> = self
                .model
                .orbit(here, self.model.orbit_indices(Dim::Zero))
                .collect();
            if cell.contains(&there) {
                continue;
            }
            // The vertex's anchor sits in one of the two cells; the other one
            // needs a vertex of its own at the same place.
            let holds_anchor = cell.contains(&self.model.vertex_attr_unchecked(vertex).dart);
            let orphan = if holds_anchor { there } else { here };
            let split = self.add_vertex_split_from(vertex, VertexAttr::new(orphan, point));
            duplicated.push((vertex, split));
        }
        Ok(EdgeUnsewing {
            kept: edge,
            created,
            vertices: duplicated,
        })
    }

    /// Reports whether the model's labelling describes its map right now.
    ///
    /// Operators leave this true; a sequence of raw primitives may not. A
    /// builder that wants to know where a state went wrong checks between two
    /// steps instead of waiting for a view to meet it.
    pub fn check_consistent(&self) -> Result<(), ModelEditError> {
        self.model
            .try_embedding_index()
            .map(|_| ())
            .map_err(ModelEditError::InvalidEmbedding)
    }

    /// Everything that claims the raw `dimension`-cell at `dart`: the entity
    /// anchored in it, and each entity a stored label there names.
    pub(crate) fn claims_on_cell(&self, dimension: Dim, dart: Dart) -> Vec<EntityOwner> {
        let mut claims = Vec::new();
        let mut note = |owner: EntityOwner| {
            if !claims.contains(&owner) {
                claims.push(owner);
            }
        };
        match dimension {
            Dim::Zero => {
                if let Some(vertex) = self.model.cell_key::<crate::model::Cell0>(dart) {
                    note(EntityOwner::Vertex(vertex));
                }
            }
            Dim::One => {
                if let Some(edge) = self.model.cell_key::<crate::model::Cell1>(dart) {
                    note(EntityOwner::Edge(edge));
                }
            }
            Dim::Two => {
                if let Some(face) = self.model.cell_key::<crate::model::Cell2>(dart) {
                    note(EntityOwner::Face(face));
                }
            }
            Dim::Three => {
                if let Some(solid) = self.model.solid_key(dart) {
                    note(EntityOwner::Solid(solid));
                }
            }
        }
        for member in self
            .model
            .orbit(dart, self.model.orbit_indices(dimension))
            .collect::<Vec<_>>()
        {
            if let Some(owner) = self.model.embedding().owner_at(dimension, member) {
                note(owner);
            }
        }
        claims
    }

    /// Settles who labels the 0-cell at `dart` after two edges were fused into
    /// `edge`: a vertex there means no scaffold, and otherwise one scaffold
    /// label of `edge` stands for whatever the fused edges had.
    fn settle_closure(&mut self, dart: Dart, edge: EdgeKey) -> Result<(), ModelEditError> {
        let claims = self.claims_on_cell(Dim::Zero, dart);
        let has_vertex = claims
            .iter()
            .any(|owner| matches!(owner, EntityOwner::Vertex(_)));
        let scaffold = claims.contains(&EntityOwner::Edge(edge));
        if has_vertex && scaffold {
            self.disown_cell(Dim::Zero, dart);
        } else if scaffold {
            self.disown_cell(Dim::Zero, dart);
            self.own_cell(Dim::Zero, dart, EntityOwner::Edge(edge));
        }
        let claims = self.claims_on_cell(Dim::Zero, dart);
        if claims.len() > 1 {
            return Err(OperatorError::ContestedCell {
                dimension: Dim::Zero,
                dart,
                claims,
            }
            .into());
        }
        Ok(())
    }

    /// The pairs `(a, b)` of `darts` that are `alpha_dim` partners, once each.
    fn pairs_within(&self, darts: &[Dart], dim: Dim) -> Vec<(Dart, Dart)> {
        let members: HashSet<Dart> = darts.iter().copied().collect();
        darts
            .iter()
            .filter_map(|&dart| {
                let linked = self.model.alpha(dim, dart);
                (linked != dart && dart.id() < linked.id() && members.contains(&linked))
                    .then_some((dart, linked))
            })
            .collect()
    }
}
