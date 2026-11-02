//! Owned-`Shape` facade over the low-level [`crate::builders`] API.
//!
//! Each constructor allocates one fresh model, runs the corresponding builder,
//! and returns the builder's primary handle in an owned [`Shape`]. The facade
//! adds no validation that belongs to a builder and preserves errors through
//! their composing types. Fallible operations are free functions; only
//! infallible operations are inherent methods on `Shape`.
//!
//! Payload-generic constructors come in pairs. The plain function chooses
//! [`crate::StandardPayload`], while its `_with` companion lets the caller
//! choose `P`, because a constructor has no shape argument from which Rust
//! could infer the payload. Operations that receive a `Shape` infer `P` and
//! therefore remain single generic functions.

pub mod blend;
pub mod edges;
pub mod faces;
pub mod heal;
pub(crate) mod import;
pub mod loft;
pub mod profiles;
pub mod revolve;
pub mod sheets;
pub mod solids;
pub mod sweep;
pub mod transform;

use crate::builders::boolean::Split;
use crate::model::Model;
use crate::topology::payload::Payload;
use crate::topology::shape::{Shape, ShapeKind};

/// The two parts a shape was split into, each owning a copy of one model.
pub struct SplitShapes<K: ShapeKind, P: Payload> {
    /// The part on the side the sheet's normals point to.
    pub front: Shape<K, P>,
    /// The part behind the sheet.
    pub back: Shape<K, P>,
}

impl<K: ShapeKind, P: Payload> SplitShapes<K, P> {
    /// Hands each part its own copy of the model both were built in.
    pub(crate) fn new(model: Model<P>, parts: Split<K::Handle>) -> Self {
        Self {
            front: Shape::new(model.clone(), parts.front),
            back: Shape::new(model, parts.back),
        }
    }
}
