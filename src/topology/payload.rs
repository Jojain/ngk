//! Type-level bundles of user data attached at each dimension of a [`GMap`](super::gmap::GMap).

/// Per-dimension payload types for a generalized map.
///
/// `V` / `E` are carried in [`VertexAttr`](super::attributes::VertexAttr) /
/// [`EdgeAttr`](super::attributes::EdgeAttr). `F` / `S` are carried in
/// [`FaceAttr`](super::attributes::FaceAttr) and on [`Solid`](super::solid::Solid).
pub trait Payload: Clone + 'static {
    type V: Clone + Default + 'static;
    type E: Clone + Default + 'static;
    type F: Clone + Default + 'static;
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
