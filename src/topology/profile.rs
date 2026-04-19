use super::closed::{Closeable, Closed};
use super::edge::EdgeRef;
use super::gmap::{Dart, GMap};
use super::payload::{Payload, StandardPayload};
use super::vertex::VertexRef;

/// A profile is the 1-dimensional connected sub-structure traced by α₀ and α₁
/// — a "wire" of alternating vertex/edge involutions. Equivalent to the
/// connected component of `dart` in the induced 1-Gmap (§3.3 of Damiand &
/// Lienhardt), independent of the host n-Gmap's dimension.
///
/// Open profiles have at least one endpoint (a dart is 0- or 1-free).
/// A closed profile is a cycle and is expressed at the type level as
/// [`LoopRef`] (= `Closed<ProfileRef>`).
pub struct ProfileRef<'a, P: Payload = StandardPayload> {
    gmap: &'a GMap<'a, P>,
    pub dart: Dart,
}

impl<'a, P: Payload> Clone for ProfileRef<'a, P> {
    fn clone(&self) -> Self {
        Self {
            gmap: self.gmap,
            dart: self.dart,
        }
    }
}

impl<'a, P: Payload> ProfileRef<'a, P> {
    pub fn new(gmap: &'a GMap<'a, P>, dart: Dart) -> Self {
        Self { gmap, dart }
    }

    pub fn darts(&self) -> impl Iterator<Item = Dart> + '_ {
        LoopIterator::new(self.gmap, self.dart)
    }

    pub fn start(&self) -> VertexRef<'a, P> {
        self.vertices()[0].clone()
    }

    pub fn end(&self) -> VertexRef<'a, P> {
        if self.is_closed() {
            self.vertices()[0].clone()
        } else {
            self.vertices()[self.vertices().len() - 1].clone()
        }
    }

    pub fn edges(&self) -> Vec<EdgeRef<'a, P>> {
        self.darts()
            .step_by(2)
            .map(|d| EdgeRef::new(self.gmap, self.gmap.cell_representative(d, 1)))
            .collect()
    }
    pub fn vertices(&self) -> Vec<VertexRef<'a, P>> {
        self.darts()
            .step_by(2)
            .map(|d| VertexRef::new(self.gmap, self.gmap.cell_representative(d, 0)))
            .collect()
    }
}

impl<'a, P: Payload> Closeable for ProfileRef<'a, P> {
    /// A profile is closed when no dart in it is 0-free or 1-free.
    fn is_closed(&self) -> bool {
        self.darts()
            .all(|d| !self.gmap.is_free(d, 0) && !self.gmap.is_free(d, 1))
    }
}

/// A closed profile — a wire with no endpoints. The closedness invariant is
/// checked at construction via [`Closed::new`].
pub type LoopRef<'a, P = StandardPayload> = Closed<ProfileRef<'a, P>>;

enum LoopInvolution {
    A0,
    A1,
}
impl LoopInvolution {
    fn next(&self) -> Self {
        match self {
            LoopInvolution::A0 => LoopInvolution::A1,
            LoopInvolution::A1 => LoopInvolution::A0,
        }
    }
}

struct LoopIterator<'a, P: Payload = StandardPayload> {
    start: Dart,
    previous: Option<Dart>,
    inv: LoopInvolution,
    gmap: &'a GMap<'a, P>,
}

impl<'a, P: Payload> LoopIterator<'a, P> {
    pub fn new(gmap: &'a GMap<'a, P>, start: Dart) -> Self {
        Self {
            start,
            previous: None,
            inv: LoopInvolution::A0,
            gmap,
        }
    }
}

impl<'a, P: Payload> Iterator for LoopIterator<'a, P> {
    type Item = Dart;

    fn next(&mut self) -> Option<Self::Item> {
        let current = match self.previous {
            None => self.start, // first call: start without moving
            Some(d) => {
                let inv = match self.inv {
                    LoopInvolution::A0 => 0,
                    LoopInvolution::A1 => 1,
                };
                self.inv = self.inv.next();
                self.gmap.alpha(inv, d)
            }
        };

        if self.gmap.is_free(current, 0)
            || self.gmap.is_free(current, 1)
            || (self.previous.is_some() && current == self.start)
        {
            None
        } else {
            self.previous = Some(current);
            Some(current)
        }
    }
}
