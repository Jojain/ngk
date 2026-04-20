use slotmap::new_key_type;

use super::closed::Closed;
use super::gmap::GMap;
use super::payload::{Payload, StandardPayload};
use super::profile::{LoopRef, ProfileRef};
use crate::geometry::surfaces::Surface;
use crate::topology::attributes::FaceAttr;

new_key_type! {pub struct FaceId;}

/// Domain-level face: one side of a [`FacetRef`](super::facet::FacetRef)
/// together with its outer boundary loop and any inner hole loops.
///
/// This is a **view** over [`FaceAttr`] in a [`GMap`]; it does not own topology
/// data.
pub struct Face<'g, P: Payload = StandardPayload> {
    gmap: &'g GMap<P>,
    id: FaceId,
}

impl<'g, P: Payload> Clone for Face<'g, P> {
    fn clone(&self) -> Self {
        Self {
            gmap: self.gmap,
            id: self.id,
        }
    }
}

impl<'g, P: Payload> Face<'g, P> {
    /// Returns a view if `id` exists in `gmap`.
    pub fn view(gmap: &'g GMap<P>, id: FaceId) -> Option<Self> {
        if gmap.face_attr(id).is_some() {
            Some(Self { gmap, id })
        } else {
            None
        }
    }

    pub fn id(&self) -> FaceId {
        self.id
    }

    pub fn outer_loop(&self) -> LoopRef<'g, P> {
        let d = self.attr().outer_loop;
        Closed::new_unchecked(ProfileRef::new(self.gmap, d))
    }

    pub fn inner_loops(&self) -> Vec<LoopRef<'g, P>> {
        self.attr()
            .inner_loops
            .iter()
            .copied()
            .map(|d| Closed::new_unchecked(ProfileRef::new(self.gmap, d)))
            .collect()
    }

    pub fn surface(&self) -> &Surface {
        &self.attr().surface
    }

    pub fn data(&self) -> &P::F {
        &self.attr().data
    }

    fn attr(&self) -> &FaceAttr<P::F> {
        self.gmap
            .face_attr(self.id)
            .expect("Face view with invalid FaceId")
    }
}
