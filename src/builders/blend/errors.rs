//! Why a chamfer or fillet refused.

use thiserror::Error;

use crate::builders::errors::ModelEditFailure;
use crate::geometry::IntersectionError;
use crate::topology::ModelEditError;
use crate::topology::shape_keys::{EdgeKey, FaceKey, ProfileKey, SolidKey, VertexKey};
use crate::topology::validation::ModelValidationError;

/// A blend that was refused, naming the entity it could not blend.
///
/// Every refusal leaves the model exactly as it was: a blend is one
/// transaction, and nothing it planned has been applied when it says no.
#[derive(Debug, Error)]
pub enum BlendError {
    #[error("fillet radius must be positive and finite, got {radius}")]
    InvalidRadius { radius: f64 },
    #[error("chamfer distance must be positive and finite, got {distance}")]
    InvalidDistance { distance: f64 },
    #[error("the blend target selects nothing to blend")]
    EmptyTarget,
    #[error("vertex {vertex:?} does not exist")]
    MissingVertex { vertex: VertexKey },
    #[error("edge {edge:?} does not exist")]
    MissingEdge { edge: EdgeKey },
    #[error("profile {profile:?} does not exist")]
    MissingProfile { profile: ProfileKey },
    #[error("face {face:?} does not exist")]
    MissingFace { face: FaceKey },
    /// A solid vertex has no fillet of its own: rounding it means rounding
    /// the edges that meet there, which is a selection of edges.
    #[error("vertex {vertex:?} bounds a solid; fillet its edges instead")]
    SolidVertexFillet { vertex: VertexKey },
    /// An edge bounding at most one face has no second face to blend into.
    #[error("edge {edge:?} bounds no solid; blend the corners at its ends instead")]
    PlanarEdge { edge: EdgeKey },
    #[error("edge {edge:?} bounds {count} faces; a solid blend needs exactly two")]
    NonManifoldEdge { edge: EdgeKey, count: usize },
    #[error("vertex {vertex:?} ends an open profile and is not a corner")]
    OpenEnd { vertex: VertexKey },
    #[error("vertex {vertex:?} is not a corner: its two edges continue each other")]
    FlatCorner { vertex: VertexKey },
    #[error("edge {edge:?} is not a crease: its two faces continue each other")]
    FlatEdge { edge: EdgeKey },
    #[error("vertex {vertex:?} does not name one corner of one wire or free face")]
    AmbiguousCorner { vertex: VertexKey },
    #[error("vertex {vertex:?} is a corner-cut target and also ends a selected edge")]
    ConflictingSelection { vertex: VertexKey },
    #[error("the corner at vertex {vertex:?} is not supported: {reason}")]
    UnsupportedCorner {
        vertex: VertexKey,
        reason: &'static str,
    },
    #[error("edge {edge:?} is not supported: {reason}")]
    UnsupportedEdge { edge: EdgeKey, reason: &'static str },
    #[error("vertex {vertex:?} is not supported: {reason}")]
    UnsupportedVertex {
        vertex: VertexKey,
        reason: &'static str,
    },
    #[error("the blend does not fit at vertex {vertex:?}: {reason}")]
    VertexDoesNotFit {
        vertex: VertexKey,
        reason: &'static str,
    },
    #[error("the blend does not fit along edge {edge:?}: {reason}")]
    EdgeDoesNotFit { edge: EdgeKey, reason: &'static str },
    #[error("the blend leaves face {face:?} with crossing or inverted boundaries")]
    FaceDoesNotFit { face: FaceKey },
    #[error("the blended solid {solid:?} is not a valid closed shell")]
    InvalidResult {
        solid: SolidKey,
        #[source]
        source: ModelValidationError,
    },
    #[error("a blend pcurve could not be written")]
    Pcurve(#[from] IntersectionError),
    #[error("a blend pcurve strays {deviation} from its curve")]
    PcurveDeviation { deviation: f64 },
    /// A planning step handed the executor something it cannot build. This is
    /// a defect in the blend itself, reported instead of panicking.
    #[error("the blend planned an inconsistent surgery: {reason}")]
    InconsistentSurgery { reason: &'static str },
    #[error("blend model edit failed")]
    ModelEditFailed(#[source] ModelEditFailure),
}

impl From<ModelEditError> for BlendError {
    fn from(error: ModelEditError) -> Self {
        Self::ModelEditFailed(ModelEditFailure::new(error))
    }
}
