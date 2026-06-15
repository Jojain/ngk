//! Type-level bundles of user data attached at each dimension of a [`GMap`](super::gmap::GMap).

/// Per-dimension payload types for a generalized map.
///
/// `V` / `E` are carried in [`VertexAttr`](super::attributes::VertexAttr) /
/// [`EdgeAttr`](super::attributes::EdgeAttr). `F` is carried in the shared
/// [`FacetAttr`](super::attributes::FacetAttr), while `S` is carried in
/// [`SolidAttr`](super::attributes::SolidAttr).
pub trait Payload: Clone + 'static {
    /// User data stored on vertex attributes.
    type V: Clone + Default + 'static;
    /// User data stored on edge attributes.
    type E: Clone + Default + 'static;
    /// User data stored on shared facet attributes.
    type F: Clone + Default + 'static;
    /// User data stored on solid attributes.
    type S: Clone + Default + 'static;
}

/// Default payload: no extra data (`()` at every dimension).
#[derive(Clone, Copy, Default, Debug)]
pub struct StandardPayload;

impl Payload for StandardPayload {
    type V = ();
    type E = ();
    type F = ();
    type S = ();
}
