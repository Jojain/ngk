use super::closed::{Closeable, Closed};
use super::edge::Edge;
use super::gmap::{Dart, Dim, GMap};
use super::payload::{Payload, StandardPayload};
use super::vertex::Vertex;

/// A profile is the 1-dimensional connected sub-structure traced by α₀ and α₁
/// — a "wire" of alternating vertex/edge involutions. Equivalent to the
/// connected component of `dart` in the induced 1-Gmap (§3.3 of Damiand &
/// Lienhardt), independent of the host n-Gmap's dimension.
///
/// Open profiles have at least one endpoint (a dart is 0- or 1-free).
/// A closed profile is a cycle and is expressed at the type level as
/// [`LoopRef`] (= `Closed<ProfileRef>`).
pub struct Profile<'a, P: Payload = StandardPayload> {
    gmap: &'a GMap<P>,
    pub dart: Dart,
}

impl<'a, P: Payload> Clone for Profile<'a, P> {
    fn clone(&self) -> Self {
        Self {
            gmap: self.gmap,
            dart: self.dart,
        }
    }
}

impl<'a, P: Payload> Profile<'a, P> {
    pub fn new(gmap: &'a GMap<P>, dart: Dart) -> Self {
        Self { gmap, dart }
    }

    pub fn darts(&self) -> impl Iterator<Item = Dart> + '_ {
        LoopIterator::new(self.gmap, self.dart)
    }

    pub fn start(&self) -> Vertex<'a, P> {
        self.vertices()[0].clone()
    }

    pub fn end(&self) -> Vertex<'a, P> {
        if self.is_closed() {
            self.vertices()[0].clone()
        } else {
            self.vertices()[self.vertices().len() - 1].clone()
        }
    }

    pub fn edges(&self) -> Vec<Edge<'a, P>> {
        self.darts()
            .step_by(2)
            .map(|d| Edge::new(self.gmap, self.gmap.cell_representative(d, Dim::One)))
            .collect()
    }
    pub fn vertices(&self) -> Vec<Vertex<'a, P>> {
        self.darts()
            .step_by(2)
            .map(|d| Vertex::new(self.gmap, self.gmap.cell_representative(d, Dim::Zero)))
            .collect()
    }
}

impl<'a, P: Payload> Closeable for Profile<'a, P> {
    /// A profile is closed when no dart in it is 0-free or 1-free.
    fn is_closed(&self) -> bool {
        self.darts()
            .all(|d| !self.gmap.is_free(d, Dim::Zero) && !self.gmap.is_free(d, Dim::One))
    }
}

/// A closed profile — a wire with no endpoints. The closedness invariant is
/// checked at construction via [`Closed::new`].
pub type Loop<'a, P = StandardPayload> = Closed<Profile<'a, P>>;

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
    gmap: &'a GMap<P>,
}

impl<'a, P: Payload> LoopIterator<'a, P> {
    pub fn new(gmap: &'a GMap<P>, start: Dart) -> Self {
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
                let dim = match self.inv {
                    LoopInvolution::A0 => Dim::Zero,
                    LoopInvolution::A1 => Dim::One,
                };
                self.inv = self.inv.next();
                self.gmap.alpha(dim, d)
            }
        };

        if self.gmap.is_free(current, Dim::Zero)
            || self.gmap.is_free(current, Dim::One)
            || (self.previous.is_some() && current == self.start)
        {
            None
        } else {
            self.previous = Some(current);
            Some(current)
        }
    }
}
