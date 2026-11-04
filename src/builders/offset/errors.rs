//! Why an entity has no offset.

use thiserror::Error;

use crate::builders::errors::ModelEditFailure;
use crate::geometry::OffsetError;
use crate::topology::edit::ModelEditError;
use crate::topology::shape_keys::{EdgeKey, FaceKey, ProfileKey, SheetKey, SolidKey, VertexKey};

/// Why an entity has no offset at the distance asked for.
///
/// Every refusal names the entity it could not move; the model is left as the
/// call found it.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum OffsetBuildError {
    #[error("edge {edge:?} does not exist")]
    MissingEdge { edge: EdgeKey },
    #[error("profile {profile:?} does not exist")]
    MissingProfile { profile: ProfileKey },
    #[error("face {face:?} does not exist")]
    MissingFace { face: FaceKey },
    #[error("sheet {sheet:?} does not exist")]
    MissingSheet { sheet: SheetKey },
    #[error("solid {solid:?} does not exist")]
    MissingSolid { solid: SolidKey },
    #[error("offset distance must be finite, got {distance}")]
    NonFiniteDistance { distance: f64 },
    /// The support has no offset at this distance.
    #[error("edge {edge:?} has no offset: {source}")]
    EdgeSupport {
        edge: EdgeKey,
        #[source]
        source: OffsetError,
    },
    #[error("face {face:?} has no offset: {source}")]
    FaceSupport {
        face: FaceKey,
        #[source]
        source: OffsetError,
    },
    /// More than two edges, or two faces' worth of sharp edges with nothing to
    /// close them, meet at this corner; where its offset goes is not decided
    /// by the moved supports alone.
    #[error("vertex {vertex:?} has no single offset position: {reason}")]
    UndecidedVertex {
        vertex: VertexKey,
        reason: &'static str,
    },
    /// Two moved neighbours that met at this corner no longer reach each
    /// other, so joining them needs a new edge.
    #[error("the moved edges at vertex {vertex:?} no longer meet (gap {gap})")]
    CornerGap { vertex: VertexKey, gap: f64 },
    /// The moved neighbours meet past this edge's far end: it vanishes.
    #[error("edge {edge:?} vanishes at this offset")]
    EdgeVanishes { edge: EdgeKey },
    /// The two moved faces along a sharp edge do not meet near where the edge
    /// would go.
    #[error("the moved faces along edge {edge:?} do not meet: {reason}")]
    SharpEdge { edge: EdgeKey, reason: &'static str },
    #[error("intersecting the moved faces along edge {edge:?} failed: {reason}")]
    SharpEdgeIntersection { edge: EdgeKey, reason: String },
    /// A pcurve could not be rewritten on the moved face.
    #[error("edge {edge:?} has no pcurve on moved face {face:?}: {reason}")]
    Pcurve {
        face: FaceKey,
        edge: EdgeKey,
        reason: String,
    },
    #[error("offset model edit failed")]
    ModelEditFailed(#[source] ModelEditFailure),
}

impl From<ModelEditError> for OffsetBuildError {
    fn from(error: ModelEditError) -> Self {
        Self::ModelEditFailed(ModelEditFailure::new(error))
    }
}
