//! Immutable geometry derived from one oriented entity at one model revision.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use thiserror::Error;

use crate::geometry::{Surface, TrimmedCurve};
use crate::topology::orientation::Orientation;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, FaceKey};
use crate::topology::unwrapped_face_domain::{UnwrappedFaceDomain, UnwrappedFaceDomainError};

use super::Model;

/// The consumer requesting a geometric interpretation of a logical entity.
///
/// Purposes have separate cache entries. These realizations retain exact
/// curves and use the existing face-domain placement rules; no sampling
/// tolerance or caller-selected cut is applied.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RealizationPurpose {
    /// Geometry queries and spatial classification.
    Geometry,
    /// Boundary sampling and surface meshing.
    Tessellation,
    /// Temporary boundary representation for file exchange.
    Exchange,
}

/// Failure to realize a selected entity's geometry.
#[derive(Debug, Error)]
pub enum RealizationError {
    /// The requested edge is absent from the model.
    #[error("edge {0:?} is not registered in this model")]
    MissingEdge(EdgeKey),
    /// The requested face is absent from the model.
    #[error("face {0:?} is not registered in this model")]
    MissingFace(FaceKey),
    /// The edge does not provide enough geometry to recover its native span.
    #[error("edge {0:?} has no realizable curve span")]
    MissingEdgeGeometry(EdgeKey),
    /// A face boundary cannot be placed in its support's parameter domain.
    #[error(transparent)]
    FaceDomain(#[from] UnwrappedFaceDomainError),
}

/// A support and its oriented boundary domain, independent of later model edits.
#[derive(Debug, Clone, PartialEq)]
pub struct FaceRealization {
    surface: Surface,
    domain: UnwrappedFaceDomain,
    sense: Orientation,
}

impl FaceRealization {
    /// Returns the support as it was when this realization was requested.
    pub fn surface(&self) -> &Surface {
        &self.surface
    }

    /// Returns the boundary pcurves placed in one unwrapped domain.
    pub fn domain(&self) -> &UnwrappedFaceDomain {
        &self.domain
    }

    /// Returns the requested orientation relative to the face's default view.
    pub fn orientation(&self) -> Orientation {
        self.sense
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct Request<K> {
    revision: u64,
    entity: K,
    sense: Orientation,
    purpose: RealizationPurpose,
}

#[derive(Default)]
pub(super) struct RealizationCache {
    edges: Mutex<HashMap<Request<EdgeKey>, Arc<TrimmedCurve>>>,
    faces: Mutex<HashMap<Request<FaceKey>, Arc<FaceRealization>>>,
}

impl<P: Payload> Model<P> {
    /// Realizes the chosen edge section in the requested direction.
    ///
    /// Same-revision requests share immutable geometry. Reversing traverses
    /// the same native interval backward, including major arcs and whole
    /// circles. Held results remain valid after the model changes.
    pub fn realize_edge(
        &self,
        edge: EdgeKey,
        sense: Orientation,
        purpose: RealizationPurpose,
    ) -> Result<Arc<TrimmedCurve>, RealizationError> {
        let request = Request {
            revision: self.revision(),
            entity: edge,
            sense,
            purpose,
        };
        if let Some(value) = self
            .realizations
            .edges
            .lock()
            .expect("realization cache lock")
            .get(&request)
        {
            return Ok(Arc::clone(value));
        }
        let view = self.edge(edge).ok_or(RealizationError::MissingEdge(edge))?;
        let view = match sense {
            Orientation::Same => view,
            Orientation::Reversed => view.reversed(),
        };
        let curve = view
            .trimmed_curve()
            .ok_or(RealizationError::MissingEdgeGeometry(edge))?;
        let mut entries = self
            .realizations
            .edges
            .lock()
            .expect("realization cache lock");
        Ok(Arc::clone(
            entries.entry(request).or_insert_with(|| Arc::new(curve)),
        ))
    }

    /// Realizes a face's support and oriented parameter-domain boundary.
    ///
    /// Geometry is built outside the cache lock with local traversal state.
    /// Concurrent requests publish one shared result. Mutations discard the
    /// cache even during a transaction, before its revision advances; rollback
    /// and deserialization rebuild from authoritative state.
    pub fn realize_face(
        &self,
        face: FaceKey,
        sense: Orientation,
        purpose: RealizationPurpose,
    ) -> Result<Arc<FaceRealization>, RealizationError> {
        let request = Request {
            revision: self.revision(),
            entity: face,
            sense,
            purpose,
        };
        if let Some(value) = self
            .realizations
            .faces
            .lock()
            .expect("realization cache lock")
            .get(&request)
        {
            return Ok(Arc::clone(value));
        }
        let view = self.face(face).ok_or(RealizationError::MissingFace(face))?;
        let view = match sense {
            Orientation::Same => view,
            Orientation::Reversed => view.reversed(),
        };
        let value = FaceRealization {
            surface: view.surface().clone(),
            domain: UnwrappedFaceDomain::of_face(&view)?,
            sense,
        };
        let mut entries = self
            .realizations
            .faces
            .lock()
            .expect("realization cache lock");
        Ok(Arc::clone(
            entries.entry(request).or_insert_with(|| Arc::new(value)),
        ))
    }
}
