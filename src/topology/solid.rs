use super::closed::Closed;
use super::gmap::GMap;
use super::payload::{Payload, StandardPayload};
use super::sheet::{Sheet, ShellRef};
use crate::topology::attributes::SolidAttr;

/// Domain-level solid: a 3D region bounded by an outer closed shell plus
/// optional inner shells (cavities). View over [`SolidAttr`] in a [`GMap`].
pub struct Solid<'g, P: Payload = StandardPayload> {
    gmap: &'g GMap<P>,
    attr: &'g SolidAttr<P::S>,
}

impl<'g, P: Payload> Clone for Solid<'g, P> {
    fn clone(&self) -> Self {
        Self {
            gmap: self.gmap,
            attr: self.attr,
        }
    }
}

impl<'g, P: Payload> Solid<'g, P> {
    pub fn new(gmap: &'g GMap<P>, attr: &'g SolidAttr<P::S>) -> Self {
        Self { gmap, attr }
    }

    pub fn data(&self) -> &P::S {
        &self.attr.data
    }

    pub fn outer_shell(&self) -> ShellRef<'g, P> {
        let d = self.attr.outer_shell;
        Closed::new_unchecked(Sheet::new(self.gmap, d))
    }

    pub fn inner_shells(&self) -> Option<Vec<ShellRef<'g, P>>> {
        self.attr.inner_shells.as_ref().map(|inner| {
            inner
                .iter()
                .map(|d| Closed::new_unchecked(Sheet::new(self.gmap, *d)))
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
}
