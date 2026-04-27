use crate::{Payload, StandardPayload, topology::gmap::GMap};

pub struct Model<P: Payload = StandardPayload> {
    map: GMap<P>,
}

impl<P: Payload> Model<P> {
    pub fn new() -> Self {
        Self { map: GMap::new() }
    }
}
