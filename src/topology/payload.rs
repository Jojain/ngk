//! Type-level bundles of user data attached at each dimension of a [`Model`](crate::model::Model).

use serde::{Deserialize, Serialize};

/// Per-dimension payload types for a generalized map.
///
/// `V` / `E` are carried in [`VertexAttr`](super::attributes::VertexAttr) /
/// [`EdgeAttr`](super::attributes::EdgeAttr). `Profile`, `F`, `Sheet`, and `S`
/// are carried by their corresponding domain attributes.
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
}

/// A [`Payload`] whose every associated type has a sensible default.
///
/// This is what [`Model::transaction`](crate::model::Model::transaction) and
/// [`PreservePayload`](super::edit::PreservePayload) require: a payload that
/// must be assigned by a policy has no default at one or more dimensions, and
/// has to open its transaction with
/// [`Model::transaction_with_policy`](crate::model::Model::transaction_with_policy)
/// and its own [`EditPolicy`](super::edit::EditPolicy) instead.
pub trait DefaultPayload:
    Payload<V: Default, E: Default, Profile: Default, F: Default, Sheet: Default, S: Default>
{
}

impl<P> DefaultPayload for P where
    P: Payload<V: Default, E: Default, Profile: Default, F: Default, Sheet: Default, S: Default>
{
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
}
