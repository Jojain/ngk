use super::closed::Closed;
use super::gmap::GMap;
use super::payload::{Payload, StandardPayload};
use super::profile::{Loop, Profile};
use crate::geometry::surfaces::Surface;
use crate::topology::attributes::FaceAttr;

pub struct Face<'g, P: Payload = StandardPayload> {
    gmap: &'g GMap<P>,
    attr: &'g FaceAttr<P::F>,
}

impl<'g, P: Payload> Clone for Face<'g, P> {
    fn clone(&self) -> Self {
        Self {
            gmap: self.gmap,
            attr: self.attr,
        }
    }
}

impl<'g, P: Payload> Face<'g, P> {
    pub fn new(gmap: &'g GMap<P>, attr: &'g FaceAttr<P::F>) -> Self {
        Self { gmap, attr }
    }

    pub fn outer_loop(&self) -> Loop<'g, P> {
        let d = self.attr.outer_loop;
        Closed::new_unchecked(Profile::new(self.gmap, d))
    }

    pub fn inner_loops(&self) -> Vec<Loop<'g, P>> {
        self.attr
            .inner_loops
            .iter()
            .map(|d| Closed::new_unchecked(Profile::new(self.gmap, *d)))
            .collect()
    }

    pub fn surface(&self) -> &Surface {
        &self.attr.surface
    }

    pub fn data(&self) -> &P::F {
        &self.attr.data
    }
}
