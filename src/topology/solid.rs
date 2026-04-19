use slotmap::new_key_type;

use super::payload::{Payload, StandardPayload};
use super::sheet::ShellRef;

new_key_type! {pub struct SolidId;}

/// A domain-level solid: a 3D region bounded by an outer closed shell plus
/// optional inner shells (cavities).
pub struct Solid<'a, P: Payload = StandardPayload> {
    pub data: P::S,
    outer: ShellRef<'a, P>,
    inners: Option<Vec<ShellRef<'a, P>>>,
}

impl<'a, P: Payload> Clone for Solid<'a, P> {
    fn clone(&self) -> Self {
        Self {
            data: self.data.clone(),
            outer: self.outer.clone(),
            inners: self.inners.clone(),
        }
    }
}

impl<'a, P: Payload> Solid<'a, P> {
    pub fn new(data: P::S, outer: ShellRef<'a, P>, inners: Option<Vec<ShellRef<'a, P>>>) -> Self {
        Self {
            data,
            outer,
            inners,
        }
    }

    pub fn shells(&self) -> Vec<&ShellRef<'a, P>> {
        let mut shells = vec![&self.outer];
        if let Some(inners) = &self.inners {
            shells.extend(inners.iter());
        }
        shells
    }
}
