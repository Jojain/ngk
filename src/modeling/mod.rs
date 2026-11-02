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

use crate::builders::boolean::{self, BooleanError, BooleanOptions, Split, SplitTarget};
use crate::model::Model;
use crate::modeling::sheets::copy_sheet;
use crate::topology::payload::Payload;
use crate::topology::shape::{Shape, ShapeKind, SheetTag};

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

/// Splits `target` along `tool` into the parts in front of and behind it,
/// each of the target's own kind: a solid, a sheet, a face, a profile or an
/// edge.
///
/// Both parts are built in a copy of `target`'s model, with `tool` copied in;
/// the inputs are unchanged. See [`crate::builders::boolean::split`] for what
/// the sheet and the target must satisfy.
pub fn split<K, P>(
    target: &Shape<K, P>,
    tool: &Shape<SheetTag, P>,
) -> Result<SplitShapes<K, P>, BooleanError>
where
    K: ShapeKind,
    K::Handle: SplitTarget,
    P: Payload,
{
    let mut model = target.model().clone();
    let tool = model.transaction(|edit| Ok::<_, BooleanError>(copy_sheet(edit, tool.sheet())))?;
    let parts = boolean::split(&mut model, target.handle(), tool, BooleanOptions::default())?;
    Ok(SplitShapes::new(model, parts))
}
