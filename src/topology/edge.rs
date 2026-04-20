use crate::geometry::curves::Curve;
use crate::topology::gmap::{Cell1, Dim};

use super::facet::Facet;
use super::gmap::{Dart, GMap};
use super::payload::{Payload, StandardPayload};
use super::sheet::Sheet;
use super::vertex::Vertex;

pub struct Edge<'a, P: Payload = StandardPayload> {
    gmap: &'a GMap<P>,
    pub dart: Dart,
}

impl<'a, P: Payload> Edge<'a, P> {
    pub fn new(gmap: &'a GMap<P>, dart: Dart) -> Self {
        Self { gmap, dart }
    }

    pub fn darts(&self) -> impl Iterator<Item = Dart> + '_ {
        self.gmap
            .orbit(self.dart, self.gmap.orbit_indices(Dim::One))
    }

    pub fn start(&self) -> Vertex<'a, P> {
        Vertex::new(self.gmap, self.dart)
    }

    pub fn end(&self) -> Vertex<'a, P> {
        Vertex::new(self.gmap, self.gmap.alpha(Dim::Zero, self.dart))
    }

    pub fn vertices(&self) -> Vec<Vertex<'a, P>> {
        self.gmap
            .incident_cells(self.dart, Dim::One, Dim::Zero)
            .map(|d| Vertex::new(self.gmap, d))
            .collect()
    }

    /// Gmap 2-cells incident to this edge.
    pub fn facets(&self) -> Vec<Facet<'a, P>> {
        self.gmap
            .incident_cells(self.dart, Dim::One, Dim::Two)
            .map(|d| Facet::new(self.gmap, d))
            .collect()
    }

    /// 2D sheets (⟨α₀, α₁, α₂⟩) incident to this edge. Wrap in
    /// [`Closed::new`](super::closed::Closed::new) to promote to a shell.
    pub fn sheets(&self) -> Vec<Sheet<'a, P>> {
        self.gmap
            .incident_cells(self.dart, Dim::One, Dim::Three)
            .map(|d| Sheet::new(self.gmap, d))
            .collect()
    }

    pub fn curve(&self) -> Option<&Curve> {
        self.gmap
            .attribute::<Cell1>(self.dart)
            .map(|attr| &attr.curve)
    }

    pub fn length(&self) -> Option<f64> {
        let t0 = self
            .start()
            .point()
            .map(|p| self.curve().map(|c| c.param_at(*p)))
            .unwrap()
            .unwrap();
        let t1 = self
            .end()
            .point()
            .map(|p| self.curve().map(|c| c.param_at(*p)))
            .unwrap()
            .unwrap();
        self.curve().map(|c| c.length(t0, t1))
    }
}
