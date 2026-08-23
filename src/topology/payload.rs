//! Type-level bundles of user data attached at each dimension of a [`GMap`](super::gmap::GMap).

use serde::{Deserialize, Serialize};

/// Per-dimension payload types for a generalized map.
///
/// `V` / `E` are carried in [`VertexAttr`](super::attributes::VertexAttr) /
/// [`EdgeAttr`](super::attributes::EdgeAttr). `Profile`, `F`, `Sheet`, and `S`
/// are carried by their corresponding domain attributes.
pub trait Payload: Clone + 'static {
    /// User data stored on vertex attributes.
    type V: Clone + Default + 'static;
    /// User data stored on edge attributes.
    type E: Clone + Default + 'static;
    /// User data stored on profile attributes.
    type Profile: Clone + Default + 'static;
    /// User data stored on face attributes.
    type F: Clone + Default + 'static;
    /// User data stored on sheet attributes.
    type Sheet: Clone + Default + 'static;
    /// User data stored on solid attributes.
    type S: Clone + Default + 'static;
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
