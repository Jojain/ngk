use crate::topology::gmap::Dim;

use super::closed::{Closeable, Closed};
use super::gmap::{Dart, GMap};
use super::payload::{Payload, StandardPayload};

/// A sheet is the 2-dimensional connected sub-structure traced by α₀, α₁ and
/// α₂ — a surface patch. Equivalent to the connected component of `dart` in
/// the induced 2-Gmap, independent of the host n-Gmap's dimension.
///
/// Open sheets have a boundary (at least one dart is free on one of
/// α₀, α₁, α₂). A closed sheet has no such free dart and is expressed at the
/// type level as [`ShellRef`] (= `Closed<SheetRef>`).
pub struct Sheet<'a, P: Payload = StandardPayload> {
    gmap: &'a GMap<P>,
    pub dart: Dart,
}

impl<'a, P: Payload> Clone for Sheet<'a, P> {
    fn clone(&self) -> Self {
        Self {
            gmap: self.gmap,
            dart: self.dart,
        }
    }
}

impl<'a, P: Payload> Sheet<'a, P> {
    pub fn new(gmap: &'a GMap<P>, dart: Dart) -> Self {
        Self { gmap, dart }
    }

    /// Every dart of this sheet, traversed via ⟨α₀, α₁, α₂⟩.
    pub fn darts(&self) -> impl Iterator<Item = Dart> + '_ {
        self.gmap.orbit(self.dart, vec![0, 1, 2])
    }
}

impl<'a, P: Payload> Closeable for Sheet<'a, P> {
    /// A sheet is closed when no dart in it is 0-, 1-, or 2-free.
    fn is_closed(&self) -> bool {
        self.darts().all(|d| {
            !self.gmap.is_free(d, Dim::Zero)
                && !self.gmap.is_free(d, Dim::One)
                && !self.gmap.is_free(d, Dim::Two)
        })
    }
}

/// A closed sheet — a surface with no boundary. The closedness invariant is
/// checked at construction via [`Closed::new`].
pub type ShellRef<'a, P = StandardPayload> = Closed<Sheet<'a, P>>;
