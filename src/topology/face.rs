use slotmap::new_key_type;

use super::payload::{Payload, StandardPayload};
use super::profile::LoopRef;
use crate::{geometry::surfaces::Surface, topology::attributes::FaceAttr};

new_key_type! {pub struct FaceId;}

/// A domain-level face: one side of a [`FacetRef`](super::facet::FacetRef)
/// together with its outer boundary loop and any inner hole loops. The first
/// entry of `loops` is the outer boundary.
pub struct Face<'a, P: Payload = StandardPayload> {
    pub attribute: FaceAttr<P::F>,
    pub loops: Vec<LoopRef<'a, P>>,
}

impl<'a, P: Payload> Clone for Face<'a, P> {
    fn clone(&self) -> Self {
        Self {
            attribute: self.attribute.clone(),
            loops: self.loops.clone(),
        }
    }
}

impl<'a, P: Payload> Face<'a, P> {
    pub fn new(attribute: FaceAttr<P::F>, loops: Vec<LoopRef<'a, P>>) -> Self {
        Self { attribute, loops }
    }

    pub fn outer_loop(&self) -> Option<&LoopRef<'a, P>> {
        self.loops.first()
    }

    pub fn surface(&self) -> &Surface {
        &self.attribute.surface
    }
}
