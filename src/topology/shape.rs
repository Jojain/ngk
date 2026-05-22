use std::marker::PhantomData;

use crate::topology::edge::Edge;
use crate::topology::face::Face;
use crate::topology::facet::Facet;
use crate::topology::gmap::{Dart, GMap};
use crate::topology::payload::{Payload, StandardPayload};
use crate::topology::profile::Profile;
use crate::topology::shape_keys::{EdgeKey, FaceKey, SolidKey, VertexKey};
use crate::topology::sheet::Sheet;
use crate::topology::solid::Solid;
use crate::topology::vertex::Vertex;

pub struct VertexTag;
pub struct EdgeTag;
pub struct FaceTag;
pub struct FacetTag;
pub struct ProfileTag;
pub struct SheetTag;
pub struct SolidTag;

pub trait ShapeKind {
    type Handle: Copy;
}

impl ShapeKind for VertexTag {
    type Handle = VertexKey;
}

impl ShapeKind for EdgeTag {
    type Handle = EdgeKey;
}

impl ShapeKind for FaceTag {
    type Handle = FaceKey;
}

impl ShapeKind for FacetTag {
    type Handle = Dart;
}

impl ShapeKind for ProfileTag {
    type Handle = Dart;
}

impl ShapeKind for SheetTag {
    type Handle = Dart;
}

impl ShapeKind for SolidTag {
    type Handle = SolidKey;
}

pub struct Shape<K: ShapeKind = SheetTag, P: Payload = StandardPayload> {
    map: GMap<P>,
    handle: K::Handle,
    kind: PhantomData<K>,
}

impl<K: ShapeKind, P: Payload> Shape<K, P> {
    pub fn new(map: GMap<P>, handle: K::Handle) -> Self {
        Self {
            map,
            handle,
            kind: PhantomData,
        }
    }

    pub fn map(&self) -> &GMap<P> {
        &self.map
    }

    pub fn map_mut(&mut self) -> &mut GMap<P> {
        &mut self.map
    }

    pub fn handle(&self) -> K::Handle {
        self.handle
    }

    pub fn into_map(self) -> (GMap<P>, K::Handle) {
        (self.map, self.handle)
    }
}

impl<P: Payload> Shape<VertexTag, P> {
    pub fn vertex(&self) -> Vertex<'_, P> {
        self.map
            .vertex(self.handle)
            .map(|v| v.vertex(&self.map))
            .expect("vertex shape key must be in the map")
    }

    pub fn key(&self) -> VertexKey {
        self.handle
    }
}

impl<P: Payload> Shape<EdgeTag, P> {
    pub fn edge(&self) -> Edge<'_, P> {
        self.map
            .edge(self.handle)
            .map(|e| e.edge(&self.map))
            .expect("edge shape key must be in the map")
    }

    pub fn key(&self) -> EdgeKey {
        self.handle
    }
}

impl<P: Payload> Shape<FaceTag, P> {
    pub fn face(&self) -> Face<'_, P> {
        self.map
            .face(self.handle)
            .map(|f| f.face(&self.map))
            .expect("face shape key must be in the map")
    }

    pub fn key(&self) -> FaceKey {
        self.handle
    }
}

impl<P: Payload> Shape<ProfileTag, P> {
    pub fn profile(&self) -> Profile<'_, P> {
        Profile::new(&self.map, self.handle)
    }

    pub fn dart(&self) -> Dart {
        self.handle
    }
}

impl<P: Payload> Shape<FacetTag, P> {
    pub fn facet(&self) -> Facet<'_, P> {
        Facet::new(&self.map, self.handle)
    }

    pub fn dart(&self) -> Dart {
        self.handle
    }
}

impl<P: Payload> Shape<SheetTag, P> {
    pub fn sheet(&self) -> Sheet<'_, P> {
        Sheet::new(&self.map, self.handle)
    }

    pub fn dart(&self) -> Dart {
        self.handle
    }
}

impl<P: Payload> Shape<SolidTag, P> {
    pub fn solid(&self) -> Solid<'_, P> {
        self.map
            .solid(self.handle)
            .map(|s| Solid::new(&self.map, s))
            .expect("solid shape key must be in the map")
    }

    pub fn key(&self) -> SolidKey {
        self.handle
    }
}
