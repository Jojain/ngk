//! The combinatorial picture of a solid blend.
//!
//! A network holds the selected solid edges, the two faces on either side of
//! each, and — for every vertex those edges end at, and every corner cut — the
//! ring of faces and edges around it in rotation order. It holds no blend
//! geometry. Cutting `k` selected edges splits a vertex's ring into `k` fans,
//! which is how a treatment knows what the vertex falls apart into; the ring
//! already covers any valence, so a new vertex treatment needs nothing more
//! from here.

use std::collections::{BTreeMap, HashSet};

use super::errors::BlendError;
use crate::geometry::{Point3, TrimmedCurve};
use crate::model::{Cell1, Cell2, Model};
use crate::topology::edge::Edge;
use crate::topology::embedding::turn;
use crate::topology::gmap::{Dart, Dim};
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, FaceKey, VertexKey};

/// The selected solid edges and everything around the vertices they end at.
#[derive(Debug)]
pub(crate) struct BlendNetwork {
    pub(crate) edges: Vec<NetworkEdge>,
    pub(crate) vertices: Vec<NetworkVertex>,
}

/// A selected solid edge.
#[derive(Debug)]
pub(crate) struct NetworkEdge {
    pub(crate) key: EdgeKey,
    /// The two faces, each with its dart on the edge at the edge's start.
    pub(crate) sides: [EdgeSide; 2],
    /// Indices into [`BlendNetwork::vertices`] of the vertex at the edge's
    /// start, then at its end.
    pub(crate) ends: [usize; 2],
    /// The edge's span, from its start to its end.
    pub(crate) span: TrimmedCurve,
}

/// One face of a selected edge.
#[derive(Debug, Clone, Copy)]
pub(crate) struct EdgeSide {
    pub(crate) face: FaceKey,
    /// The face's dart on the edge, at the edge's start.
    pub(crate) start: Dart,
}

/// A vertex a selected edge ends at, or a solid corner to cut.
#[derive(Debug)]
pub(crate) struct NetworkVertex {
    pub(crate) key: VertexKey,
    pub(crate) point: Point3,
    /// The edges and faces around the vertex, in rotation order.
    pub(crate) ring: Vec<RingSlot>,
    /// Whether the vertex is a corner-cut target.
    pub(crate) cut: bool,
}

/// One step round a vertex: an edge, then the face that follows it.
#[derive(Debug, Clone, Copy)]
pub(crate) struct RingSlot {
    pub(crate) edge: EdgeKey,
    /// The edge's index in [`BlendNetwork::edges`] when it is selected.
    pub(crate) selected: Option<usize>,
    /// The face between this slot's edge and the next slot's.
    pub(crate) face: FaceKey,
    /// The face's darts at the vertex: on this slot's edge, then on the next
    /// slot's.
    pub(crate) darts: [Dart; 2],
}

impl NetworkVertex {
    /// How many selected edges meet here.
    pub(crate) fn selected_count(&self) -> usize {
        self.ring
            .iter()
            .filter(|slot| slot.selected.is_some())
            .count()
    }

    /// The ring turned so that it starts at `slot`.
    pub(crate) fn ring_from(&self, slot: usize) -> Vec<RingSlot> {
        let mut ring = self.ring.clone();
        ring.rotate_left(slot);
        ring
    }
}

impl NetworkEdge {
    /// Which of this edge's sides lies on `face`.
    pub(crate) fn side_on(&self, face: FaceKey) -> Option<usize> {
        self.sides.iter().position(|side| side.face == face)
    }

    /// Which end of this edge the network vertex `vertex` is.
    pub(crate) fn end_at(&self, vertex: usize) -> usize {
        usize::from(self.ends[0] != vertex)
    }
}

/// Captures the network of `edges` and `corner_cuts` from the model.
pub(crate) fn capture<P: Payload>(
    model: &Model<P>,
    edges: &[EdgeKey],
    corner_cuts: &[VertexKey],
) -> Result<BlendNetwork, BlendError> {
    let index = edges
        .iter()
        .enumerate()
        .map(|(position, &edge)| (edge, position))
        .collect::<BTreeMap<_, _>>();
    let mut vertices = Vec::<NetworkVertex>::new();
    let mut vertex_index = BTreeMap::<VertexKey, usize>::new();
    let mut vertex_at =
        |model: &Model<P>, key: VertexKey, cut: bool| -> Result<usize, BlendError> {
            if let Some(&existing) = vertex_index.get(&key) {
                if cut || vertices[existing].cut {
                    return Err(BlendError::ConflictingSelection { vertex: key });
                }
                return Ok(existing);
            }
            let attr = model
                .vertex_attr(key)
                .ok_or(BlendError::MissingVertex { vertex: key })?;
            let ring = walk_ring(model, key, attr.dart, &index)?;
            vertices.push(NetworkVertex {
                key,
                point: attr.point,
                ring,
                cut,
            });
            vertex_index.insert(key, vertices.len() - 1);
            Ok(vertices.len() - 1)
        };

    let mut network_edges = Vec::with_capacity(edges.len());
    for &key in edges {
        let edge = model
            .edge(key)
            .ok_or(BlendError::MissingEdge { edge: key })?;
        let Some(bounded) = edge.bounded() else {
            return Err(BlendError::UnsupportedEdge {
                edge: key,
                reason: "it is closed, and a blend of a closed edge needs a seam",
            });
        };
        let (start_vertex, end_vertex) = bounded.vertices();
        let start = model.edge_attr_unchecked(key).dart;
        let across = model.alpha(Dim::Two, start);
        let turned = turn(model.topology(), model.embedding_index(), Dim::Two, start);
        if across == start || turned != Some(across) {
            return Err(BlendError::UnsupportedEdge {
                edge: key,
                reason: "its two faces are not sewn directly to each other",
            });
        }
        let faces = [start, across].map(|dart| model.cell_key::<Cell2>(dart));
        let [Some(first), Some(second)] = faces else {
            return Err(BlendError::PlanarEdge { edge: key });
        };
        if first == second {
            return Err(BlendError::UnsupportedEdge {
                edge: key,
                reason: "one face runs along both of its sides",
            });
        }
        if model.solid_key(start).is_none() {
            return Err(BlendError::UnsupportedEdge {
                edge: key,
                reason: "its faces bound no solid",
            });
        }
        let ends = [
            vertex_at(model, start_vertex.key(), false)?,
            vertex_at(model, end_vertex.key(), false)?,
        ];
        network_edges.push(NetworkEdge {
            key,
            sides: [
                EdgeSide { face: first, start },
                EdgeSide {
                    face: second,
                    start: across,
                },
            ],
            ends,
            span: Edge::from_dart(model, start)
                .expect("a selected edge's stored dart names it")
                .trimmed_curve(),
        });
    }
    for &vertex in corner_cuts {
        vertex_at(model, vertex, true)?;
    }
    Ok(BlendNetwork {
        edges: network_edges,
        vertices,
    })
}

/// Walks the faces and edges around a vertex, turning across scaffold.
fn walk_ring<P: Payload>(
    model: &Model<P>,
    vertex: VertexKey,
    dart: Dart,
    selected: &BTreeMap<EdgeKey, usize>,
) -> Result<Vec<RingSlot>, BlendError> {
    let open = || BlendError::UnsupportedVertex {
        vertex,
        reason: "its faces do not close around it",
    };
    let index = model.embedding_index();
    // A vertex's anchor may sit on a bridge the face owns, which is scaffold
    // and names no edge; the walk starts on a logical edge and turns across
    // scaffold from there.
    let dart = model
        .orbit(dart, model.orbit_indices(Dim::Zero))
        .find(|&candidate| {
            model.cell_key::<Cell1>(candidate).is_some() && !index.is_embedded(Dim::One, candidate)
        })
        .ok_or_else(open)?;
    let mut ring = Vec::new();
    let mut seen = HashSet::new();
    let mut current = dart;
    loop {
        if !seen.insert(current) {
            break;
        }
        let next_edge = turn(model.topology(), index, Dim::One, current).ok_or_else(open)?;
        let (Some(edge), Some(face)) = (
            model.cell_key::<Cell1>(current),
            model.cell_key::<Cell2>(current),
        ) else {
            return Err(BlendError::UnsupportedVertex {
                vertex,
                reason: "a face or edge around it is not registered",
            });
        };
        ring.push(RingSlot {
            edge,
            selected: selected.get(&edge).copied(),
            face,
            darts: [current, next_edge],
        });
        current = turn(model.topology(), index, Dim::Two, next_edge).ok_or_else(open)?;
        if current == dart {
            return Ok(ring);
        }
    }
    Err(open())
}
