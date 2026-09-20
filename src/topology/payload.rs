//! Type-level bundles of user data attached at each dimension of a [`Model`](crate::model::Model).

use serde::{Deserialize, Serialize};

use super::edit::{EditPolicy, PreservePayload};

/// Per-dimension payload types for a generalized map, and the policy that
/// maintains them.
///
/// `V` / `E` are carried in [`VertexAttr`](super::attributes::VertexAttr) /
/// [`EdgeAttr`](super::attributes::EdgeAttr). `Profile`, `F`, `Sheet`, and `S`
/// are carried by their corresponding domain attributes.
///
/// # The policy belongs to the payload
///
/// How a colour survives a split, or which of two tags wins a merge, is a
/// property of the data rather than of the operation that happens to move it.
/// So it is named once here, as [`Policy`](Payload::Policy), and every builder
/// in the kernel then runs it with nothing said at any call site —
/// [`Model::transaction`](crate::model::Model::transaction) resolves it from
/// `P` alone.
///
/// What a *particular* call wants to stamp or record is the other question, and
/// that one does belong in a signature:
/// [`Model::transaction_with_policy`](crate::model::Model::transaction_with_policy)
/// overrides this for one transaction.
pub trait Payload: Clone + 'static {
    /// User data stored on vertex attributes.
    type V: Clone + 'static;
    /// User data stored on edge attributes.
    type E: Clone + 'static;
    /// User data stored on profile attributes.
    type Profile: Clone + 'static;
    /// User data stored on face attributes.
    type F: Clone + 'static;
    /// User data stored on sheet attributes.
    type Sheet: Clone + 'static;
    /// User data stored on solid attributes.
    type S: Clone + 'static;

    /// How this data maintains itself across an edit that names no policy.
    ///
    /// A payload whose dimensions all implement `Default` can name
    /// [`PreservePayload`], which clones a split's source, keeps the survivor
    /// of a merge, drops on a consume and defaults the rest. One that must be
    /// assigned — a colour, a stable id, an owning feature — names its own
    /// instead, and is then free to have no `Default` at any dimension.
    type Policy: EditPolicy<Self> + Default;
}

/// Default payload: no extra data (`()` at every dimension).
#[derive(Clone, Copy, Default, Debug, Serialize, Deserialize)]
pub struct StandardPayload;

impl Payload for StandardPayload {
    type V = ();
    type E = ();
    type Profile = ();
    type F = ();
    type Sheet = ();
    type S = ();

    type Policy = PreservePayload;
}
