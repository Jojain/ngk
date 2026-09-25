//! What a blend applies to, and the one set every spelling of it resolves to.
//!
//! A target is a list of vertices, edges, profiles and faces in any mix. It is
//! resolved against the model into three sets — 2D corners, solid edges and
//! solid corner cuts — and whether an element is 2D or 3D is read from its
//! incidence, never from the call. A face, its profiles and the list of its
//! edges therefore resolve to the same set, and a set has no order, so no
//! spelling of a selection can change the result.
//!
//! A solid edge brings the edges it runs on into without a corner: a blend
//! cannot stop part way along a smooth crease, so a slot's straight rim and
//! the arcs it runs into are blended together whichever of them is named.

use std::collections::{BTreeMap, BTreeSet};

use nalgebra::Vector3;

use super::errors::BlendError;
use super::law::BlendLaw;
use crate::geometry::ANGULAR_TOLERANCE;
use crate::geometry::parameter::Fraction;
use crate::model::Model;
use crate::topology::edge::Edge;
use crate::topology::gmap::{Dart, Dim};
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, FaceKey, ProfileKey, VertexKey};
use crate::topology::vertex::Vertex;

/// Fractions of an edge at which its two faces are compared for tangency.
const FLATNESS_SAMPLES: [f64; 3] = [0.25, 0.5, 0.75];

/// One element of a blend target.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum BlendSelection {
    /// A corner of a wire or free face, or a solid vertex to cut (chamfer only).
    Vertex(VertexKey),
    /// A solid edge.
    Edge(EdgeKey),
    /// Every corner of a wire or free-face loop, or every edge of a solid face's loop.
    Profile(ProfileKey),
    /// Every corner of a free face, or every edge of a solid face.
    Face(FaceKey),
}

/// What a chamfer or fillet applies to: any mix of vertices, edges, profiles
/// and faces, resolved as one set.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct BlendTarget {
    selections: Vec<BlendSelection>,
}

impl BlendTarget {
    /// Creates an empty target; [`Self::with`] adds selections to it.
    pub fn new() -> Self {
        Self::default()
    }

    /// Adds one more selection to the target.
    pub fn with(mut self, selection: impl Into<BlendSelection>) -> Self {
        self.selections.push(selection.into());
        self
    }

    /// Returns the selections in the order they were given.
    pub fn selections(&self) -> &[BlendSelection] {
        &self.selections
    }
}

impl From<BlendSelection> for BlendTarget {
    fn from(selection: BlendSelection) -> Self {
        Self {
            selections: vec![selection],
        }
    }
}

impl From<Vec<BlendSelection>> for BlendTarget {
    fn from(selections: Vec<BlendSelection>) -> Self {
        Self { selections }
    }
}

macro_rules! selection_from_key {
    ($key:ty, $variant:ident) => {
        impl From<$key> for BlendSelection {
            fn from(key: $key) -> Self {
                Self::$variant(key)
            }
        }

        impl From<$key> for BlendTarget {
            fn from(key: $key) -> Self {
                BlendSelection::from(key).into()
            }
        }

        impl From<Vec<$key>> for BlendTarget {
            fn from(keys: Vec<$key>) -> Self {
                Self {
                    selections: keys.into_iter().map(BlendSelection::from).collect(),
                }
            }
        }

        impl<const N: usize> From<[$key; N]> for BlendTarget {
            fn from(keys: [$key; N]) -> Self {
                Vec::from(keys).into()
            }
        }
    };
}

selection_from_key!(VertexKey, Vertex);
selection_from_key!(EdgeKey, Edge);
selection_from_key!(ProfileKey, Profile);
selection_from_key!(FaceKey, Face);

/// One corner of a wire or free face: the vertex where two edges of it meet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct CornerRef {
    pub(crate) vertex: VertexKey,
    /// The free face whose loop turns here, or `None` on a wire.
    pub(crate) face: Option<FaceKey>,
    /// The incoming edge's dart at the corner.
    ///
    /// On a face this follows the face's stored loop direction, so the edge
    /// inserted after it runs the way the loop does.
    pub(crate) after: Dart,
    /// The outgoing edge's dart at the corner.
    pub(crate) next: Dart,
}

/// A target resolved against the model: three sets, each ordered by key.
#[derive(Debug, Default)]
pub(crate) struct Resolution {
    /// Corners of wires and free faces.
    pub(crate) corners: Vec<CornerRef>,
    /// Solid edges.
    pub(crate) edges: Vec<EdgeKey>,
    /// Solid vertices whose corner is cut off (a chamfer only).
    pub(crate) corner_cuts: Vec<VertexKey>,
}

impl Resolution {
    pub(crate) fn is_empty(&self) -> bool {
        self.corners.is_empty() && self.edges.is_empty() && self.corner_cuts.is_empty()
    }
}

/// Resolves every selection of `target` into corners, edges and corner cuts.
///
/// A flat corner or edge reached by expanding a profile or face is skipped,
/// since there is nothing there to blend; named on its own it is refused.
pub(crate) fn resolve<P: Payload>(
    model: &Model<P>,
    target: &BlendTarget,
    law: BlendLaw,
) -> Result<Resolution, BlendError> {
    let mut corners = BTreeMap::new();
    let mut edges = BTreeSet::new();
    let mut corner_cuts = BTreeSet::new();

    for selection in target.selections() {
        match *selection {
            BlendSelection::Vertex(vertex) => {
                let view = model
                    .vertex(vertex)
                    .ok_or(BlendError::MissingVertex { vertex })?;
                if bounds_solid_vertex(&view) {
                    if law.is_fillet() {
                        return Err(BlendError::SolidVertexFillet { vertex });
                    }
                    corner_cuts.insert(vertex);
                    continue;
                }
                let corner = planar_corner(model, &view)?;
                if is_flat_corner(model, &corner) {
                    return Err(BlendError::FlatCorner { vertex });
                }
                corners.insert(vertex, corner);
            }
            BlendSelection::Edge(edge) => {
                let view = model.edge(edge).ok_or(BlendError::MissingEdge { edge })?;
                check_solid_edge(&view)?;
                if is_flat_edge(model, &view) {
                    return Err(BlendError::FlatEdge { edge });
                }
                edges.insert(edge);
            }
            BlendSelection::Profile(profile) => {
                let view = model
                    .profile(profile)
                    .ok_or(BlendError::MissingProfile { profile })?;
                let profile_edges = view.edges();
                let free_face = profile_edges
                    .iter()
                    .find_map(|edge| edge.faces().into_iter().next())
                    .map(|face| face.key());
                if profile_edges.iter().any(|edge| edge.faces().len() == 2) {
                    expand_solid_edges(model, &profile_edges, &mut edges)?;
                } else if let Some(face) = free_face {
                    let darts = profile_loop(model, face, profile)?;
                    expand_loop_corners(model, face, darts, &mut corners)?;
                } else {
                    expand_wire_corners(model, view.darts().collect(), &mut corners)?;
                }
            }
            BlendSelection::Face(face) => {
                let view = model.face(face).ok_or(BlendError::MissingFace { face })?;
                let face_edges = view.edges();
                if face_edges.iter().any(|edge| edge.faces().len() == 2) {
                    expand_solid_edges(model, &face_edges, &mut edges)?;
                } else {
                    let face = model.face_unchecked(face);
                    for loop_ in face.loops() {
                        expand_loop_corners(
                            model,
                            face.key(),
                            loop_.darts().collect(),
                            &mut corners,
                        )?;
                    }
                }
            }
        }
    }

    grow_tangent_chains(model, &mut edges);
    Ok(Resolution {
        corners: corners.into_values().collect(),
        edges: edges.into_iter().collect(),
        corner_cuts: corner_cuts.into_iter().collect(),
    })
}

/// Adds to `edges` every solid edge that runs on without a corner from one
/// already there, until each chain closes or ends at a corner.
///
/// An edge runs on from another at a vertex where it leaves along the tangent
/// the other arrives on. Where two edges would, the chain is ambiguous there
/// and does not grow.
fn grow_tangent_chains<P: Payload>(model: &Model<P>, edges: &mut BTreeSet<EdgeKey>) {
    let mut pending = edges.iter().copied().collect::<Vec<_>>();
    while let Some(key) = pending.pop() {
        let Some(bounded) = model.edge_unchecked(key).bounded() else {
            continue;
        };
        for vertex in [bounded.start().key(), bounded.end().key()] {
            let Some(arriving) = leaving(model, key, vertex).map(|direction| -direction) else {
                continue;
            };
            let Some(view) = model.vertex(vertex) else {
                continue;
            };
            let continuations = view
                .edges()
                .into_iter()
                .filter(|edge| edge.key() != key && edge.faces().len() == 2)
                .filter(|edge| {
                    leaving(model, edge.key(), vertex).is_some_and(|direction| {
                        direction.dot(&arriving) >= 1.0 - ANGULAR_TOLERANCE.sqrt()
                    })
                })
                .collect::<Vec<_>>();
            let [next] = continuations.as_slice() else {
                continue;
            };
            if !is_flat_edge(model, next) && edges.insert(next.key()) {
                pending.push(next.key());
            }
        }
    }
}

/// The unit tangent a bounded edge leaves `vertex` along, if it ends there
/// once.
fn leaving<P: Payload>(model: &Model<P>, edge: EdgeKey, vertex: VertexKey) -> Option<Vector3<f64>> {
    let view = model.edge_unchecked(edge);
    let bounded = view.bounded()?;
    let span = view.trimmed_curve();
    let direction = match (
        bounded.start().key() == vertex,
        bounded.end().key() == vertex,
    ) {
        (true, false) => span.derivative_at(Fraction::START, 1),
        (false, true) => -span.derivative_at(Fraction::END, 1),
        _ => return None,
    };
    (direction.norm() > 0.0).then(|| direction.normalize())
}

/// Whether a vertex is where solid faces meet rather than a planar corner.
fn bounds_solid_vertex<P: Payload>(vertex: &Vertex<'_, P>) -> bool {
    vertex.edges().iter().any(|edge| edge.faces().len() >= 2)
}

/// Resolves a vertex named on its own to the one planar corner it is.
fn planar_corner<P: Payload>(
    model: &Model<P>,
    vertex: &Vertex<'_, P>,
) -> Result<CornerRef, BlendError> {
    let key = vertex.key();
    let faces = vertex.faces();
    match faces.as_slice() {
        [] => wire_corner(model, key),
        [face] => {
            let face = model.face_unchecked(face.key());
            let mut found = None;
            for loop_ in face.loops() {
                let darts = loop_.darts().collect::<Vec<_>>();
                for corner in loop_corners(model, face.key(), &darts) {
                    if corner.vertex == key {
                        if found.is_some() {
                            return Err(BlendError::AmbiguousCorner { vertex: key });
                        }
                        found = Some(corner);
                    }
                }
            }
            found.ok_or(BlendError::OpenEnd { vertex: key })
        }
        _ => Err(BlendError::AmbiguousCorner { vertex: key }),
    }
}

/// The darts of the loop of free face `face` that runs along `profile`.
fn profile_loop<P: Payload>(
    model: &Model<P>,
    face: FaceKey,
    profile: ProfileKey,
) -> Result<Vec<Dart>, BlendError> {
    model
        .face_unchecked(face)
        .loops()
        .into_iter()
        .find(|loop_| loop_.profile_key() == Some(profile))
        .map(|loop_| loop_.darts().collect())
        .ok_or(BlendError::MissingProfile { profile })
}

/// Resolves a wire vertex: the two edge ends that meet there.
fn wire_corner<P: Payload>(model: &Model<P>, vertex: VertexKey) -> Result<CornerRef, BlendError> {
    let dart = model.vertex_attr_unchecked(vertex).dart;
    let darts = model
        .orbit(dart, model.orbit_indices(Dim::Zero))
        .collect::<Vec<_>>();
    match darts.as_slice() {
        [first, second] if model.alpha(Dim::One, *first) == *second => Ok(CornerRef {
            vertex,
            face: None,
            after: (*first).min(*second),
            next: (*first).max(*second),
        }),
        [_] => Err(BlendError::OpenEnd { vertex }),
        _ => Err(BlendError::AmbiguousCorner { vertex }),
    }
}

/// Every corner of one face loop, read in the loop's stored direction.
fn loop_corners<P: Payload>(model: &Model<P>, face: FaceKey, darts: &[Dart]) -> Vec<CornerRef> {
    let count = darts.len();
    (0..count)
        .filter_map(|index| {
            let incoming = darts[(index + count - 1) % count];
            let outgoing = darts[index];
            let vertex = Vertex::from_dart(model, outgoing)?.key();
            Some(CornerRef {
                vertex,
                face: Some(face),
                after: model.alpha(Dim::Zero, incoming),
                next: outgoing,
            })
        })
        .collect()
}

/// Adds every real corner of one free-face loop, skipping flat ones.
fn expand_loop_corners<P: Payload>(
    model: &Model<P>,
    face: FaceKey,
    darts: Vec<Dart>,
    corners: &mut BTreeMap<VertexKey, CornerRef>,
) -> Result<(), BlendError> {
    for corner in loop_corners(model, face, &darts) {
        if is_flat_corner(model, &corner) {
            continue;
        }
        if let Some(existing) = corners.insert(corner.vertex, corner)
            && existing != corner
        {
            return Err(BlendError::AmbiguousCorner {
                vertex: corner.vertex,
            });
        }
    }
    Ok(())
}

/// Adds every interior corner of a wire, skipping flat ones.
fn expand_wire_corners<P: Payload>(
    model: &Model<P>,
    darts: Vec<Dart>,
    corners: &mut BTreeMap<VertexKey, CornerRef>,
) -> Result<(), BlendError> {
    let mut vertices = BTreeSet::new();
    for dart in darts {
        if model.is_free(dart, Dim::One) {
            continue;
        }
        if let Some(vertex) = Vertex::from_dart(model, dart) {
            vertices.insert(vertex.key());
        }
    }
    for vertex in vertices {
        let corner = wire_corner(model, vertex)?;
        if !is_flat_corner(model, &corner) {
            corners.insert(vertex, corner);
        }
    }
    Ok(())
}

/// Adds every edge of a solid face or loop, skipping flat ones.
fn expand_solid_edges<P: Payload>(
    model: &Model<P>,
    edges: &[Edge<'_, P>],
    selected: &mut BTreeSet<EdgeKey>,
) -> Result<(), BlendError> {
    for edge in edges {
        check_solid_edge(edge)?;
        if !is_flat_edge(model, edge) {
            selected.insert(edge.key());
        }
    }
    Ok(())
}

/// Refuses an edge that is not shared by exactly two faces.
fn check_solid_edge<P: Payload>(edge: &Edge<'_, P>) -> Result<(), BlendError> {
    match edge.faces().len() {
        2 => Ok(()),
        0 | 1 => Err(BlendError::PlanarEdge { edge: edge.key() }),
        count => Err(BlendError::NonManifoldEdge {
            edge: edge.key(),
            count,
        }),
    }
}

/// Whether the two edges of a corner leave it along one tangent line, the
/// incoming one arriving the way the outgoing one leaves.
fn is_flat_corner<P: Payload>(model: &Model<P>, corner: &CornerRef) -> bool {
    let incoming = Edge::from_dart(model, model.alpha(Dim::Zero, corner.after));
    let outgoing = Edge::from_dart(model, corner.next);
    let (Some(incoming), Some(outgoing)) = (incoming, outgoing) else {
        return false;
    };
    let arriving = incoming.trimmed_curve().derivative_at(Fraction::END, 1);
    let leaving = outgoing.trimmed_curve().derivative_at(Fraction::START, 1);
    let (arriving, leaving) = (arriving.normalize(), leaving.normalize());
    arriving.cross(&leaving).norm() <= ANGULAR_TOLERANCE.sqrt() && arriving.dot(&leaving) > 0.0
}

/// Whether an edge's two faces continue each other all along it, so it is no
/// crease at all: two faces of one plane, or a round and the face it runs
/// tangent into.
///
/// Normals are read in each face's stored orientation, which is outward on a
/// consistently oriented shell; a view's own sense depends on the dart it was
/// reached by and would say nothing here.
fn is_flat_edge<P: Payload>(model: &Model<P>, edge: &Edge<'_, P>) -> bool {
    let faces = edge.faces();
    let [first, second] = faces.as_slice() else {
        return false;
    };
    let [first, second] = [first.key(), second.key()].map(|key| model.face_unchecked(key));
    let span = edge.trimmed_curve();
    FLATNESS_SAMPLES.iter().all(|&fraction| {
        let point = span.point_at(Fraction::new(fraction));
        let normals = [&first, &second].map(|face| {
            face.surface()
                .param_at(point)
                .ok()
                .map(|uv| face.normal_at(uv.x, uv.y))
        });
        let [Some(first_normal), Some(second_normal)] = normals else {
            return false;
        };
        first_normal.dot(&second_normal) >= 1.0 - ANGULAR_TOLERANCE.sqrt()
    })
}
