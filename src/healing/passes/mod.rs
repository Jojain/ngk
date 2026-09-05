//! The healing passes and the shared lookups they need.
//!
//! Each pass proposes cells to one of the removal operations, in the order the
//! driver in [`super`] runs them: edges first, so that faces fuse and expose
//! newly shape-free vertices, then vertices.

pub(super) mod edges;
pub(super) mod vertices;

use crate::topology::gmap::{Cell1, Cell2, Dart, Dim, GMap};
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, FaceKey, VertexKey};

use super::errors::HealingError;
use super::options::{HealingOptions, HealingScope};

/// Returns the vertices the 0-removal pass may consider, in a stable order.
pub(super) fn scoped_vertices<P: Payload>(
    g: &GMap<P>,
    options: &HealingOptions,
) -> Result<Vec<VertexKey>, HealingError> {
    Ok(match &options.scope {
        HealingScope::WholeMap => g.iter_vertices().map(|(key, _)| key).collect(),
        HealingScope::Solid(solid) => g
            .solid(*solid)
            .ok_or(HealingError::MissingSolid { solid: *solid })?
            .vertices()
            .iter()
            .map(|vertex| vertex.key())
            .collect(),
        HealingScope::Cells { vertices, .. } => vertices.clone(),
    })
}

/// Returns the edges the 1-removal pass may consider, in a stable order.
pub(super) fn scoped_edges<P: Payload>(
    g: &GMap<P>,
    options: &HealingOptions,
) -> Result<Vec<EdgeKey>, HealingError> {
    Ok(match &options.scope {
        HealingScope::WholeMap => g.iter_edges().map(|(key, _)| key).collect(),
        HealingScope::Solid(solid) => g
            .solid(*solid)
            .ok_or(HealingError::MissingSolid { solid: *solid })?
            .edges()
            .iter()
            .map(|edge| edge.key())
            .collect(),
        HealingScope::Cells { edges, .. } => edges.clone(),
    })
}

/// Returns the distinct faces carrying the edge orbit rooted at `dart`.
pub(super) fn incident_faces<P: Payload>(g: &GMap<P>, dart: Dart) -> Vec<FaceKey> {
    let mut faces = Vec::new();
    for d in g.orbit(dart, g.orbit_indices(Dim::One)) {
        if let Some(face) = g.cell_key::<Cell2>(d)
            && !faces.contains(&face)
        {
            faces.push(face);
        }
    }
    faces
}

/// Returns one dart of the edge orbit rooted at `dart` that belongs to `face`.
pub(super) fn edge_dart_in_face<P: Payload>(
    g: &GMap<P>,
    dart: Dart,
    face: FaceKey,
) -> Option<Dart> {
    g.orbit(dart, g.orbit_indices(Dim::One))
        .find(|&d| g.cell_key::<Cell2>(d) == Some(face))
}

/// Returns the dart from which `face` traverses `edge` in boundary order.
///
/// This is the dart the profile builders key a parameter curve on, so a
/// rebuilt curve must be keyed on it too.
pub(super) fn boundary_dart<P: Payload>(g: &GMap<P>, face: FaceKey, edge: EdgeKey) -> Option<Dart> {
    let view = g.face(face)?;
    view.loops()
        .iter()
        .flat_map(|boundary| boundary.edges())
        .find(|candidate| candidate.key() == edge)
        .map(|candidate| candidate.dart())
}

/// Returns the key of the edge containing `dart`.
pub(super) fn edge_key<P: Payload>(g: &GMap<P>, dart: Dart) -> Option<EdgeKey> {
    g.cell_key::<Cell1>(dart)
}
