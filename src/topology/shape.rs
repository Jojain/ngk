use crate::topology::gmap::GMap;
use crate::topology::payload::{Payload, StandardPayload};
use crate::topology::shape_keys::VertexKey;
use crate::topology::vertex::Vertex;

pub struct Shape<K, P: Payload = StandardPayload> {
    map: GMap<P>,
    key: K,
}

impl<H, P: Payload> Shape<H, P> {
    pub fn new(map: GMap<P>, key: H) -> Self {
        Self { map, key }
    }
    pub fn map(&self) -> &GMap<P> {
        &self.map
    }
    pub fn map_mut(&mut self) -> &mut GMap<P> {
        &mut self.map
    }
    pub fn into_map(self) -> (GMap<P>, H) {
        (self.map, self.key)
    }
}

impl<P: Payload> Shape<VertexKey, P> {
    pub fn vertex(&self) -> Vertex<'_, P> {
        self.map
            .vertex(self.key)
            .expect("Vertex must be in the map")
    }
}
