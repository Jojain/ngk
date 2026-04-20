use slotmap::new_key_type;

use super::closed::Closed;
use super::gmap::GMap;
use super::payload::{Payload, StandardPayload};
use super::sheet::{ShellRef, SheetRef};
use crate::topology::attributes::SolidAttr;

new_key_type! {pub struct SolidId;}

/// Domain-level solid: a 3D region bounded by an outer closed shell plus
/// optional inner shells (cavities). View over [`SolidAttr`] in a [`GMap`].
pub struct Solid<'g, P: Payload = StandardPayload> {
    gmap: &'g GMap<P>,
    id: SolidId,
}

impl<'g, P: Payload> Clone for Solid<'g, P> {
    fn clone(&self) -> Self {
        Self {
            gmap: self.gmap,
            id: self.id,
        }
    }
}

impl<'g, P: Payload> Solid<'g, P> {
    pub fn view(gmap: &'g GMap<P>, id: SolidId) -> Option<Self> {
        if gmap.solid_attr(id).is_some() {
            Some(Self { gmap, id })
        } else {
            None
        }
    }

    pub fn id(&self) -> SolidId {
        self.id
    }

    pub fn data(&self) -> &P::S {
        &self.attr().data
    }

    pub fn outer_shell(&self) -> ShellRef<'g, P> {
        let d = self.attr().outer_shell;
        Closed::new_unchecked(SheetRef::new(self.gmap, d))
    }

    pub fn inner_shells(&self) -> Option<Vec<ShellRef<'g, P>>> {
        self.attr().inner_shells.as_ref().map(|inner| {
            inner
                .iter()
                .copied()
                .map(|d| Closed::new_unchecked(SheetRef::new(self.gmap, d)))
                .collect()
        })
    }

    pub fn shells(&self) -> Vec<ShellRef<'g, P>> {
        let mut shells = vec![self.outer_shell()];
        if let Some(inners) = self.inner_shells() {
            shells.extend(inners);
        }
        shells
    }

    fn attr(&self) -> &SolidAttr<P::S> {
        self.gmap
            .solid_attr(self.id)
            .expect("Solid view with invalid SolidId")
    }
}
