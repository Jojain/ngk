use std::marker::PhantomData;

use crate::topology::edge::Edge;
use crate::topology::face::Face;
use crate::topology::gmap::GMap;
use crate::topology::payload::{Payload, StandardPayload};
use crate::topology::profile::Profile;
use crate::topology::shape_keys::{EdgeKey, FaceKey, ProfileKey, SheetKey, SolidKey, VertexKey};
use crate::topology::sheet::Sheet;
use crate::topology::solid::Solid;
use crate::topology::vertex::Vertex;

/// Type marker for an owned vertex shape.
pub struct VertexTag;
/// Type marker for an owned edge shape.
pub struct EdgeTag;
/// Type marker for an owned face shape.
pub struct FaceTag;
/// Type marker for an owned profile shape.
pub struct ProfileTag;
/// Type marker for an owned sheet shape.
pub struct SheetTag;
/// Type marker for an owned solid shape.
pub struct SolidTag;

/// Marker trait connecting a shape kind to its handle type.
pub trait ShapeKind {
    /// The value needed to recover the typed view from the owned map.
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

impl ShapeKind for ProfileTag {
    type Handle = ProfileKey;
}

impl ShapeKind for SheetTag {
    type Handle = SheetKey;
}

impl ShapeKind for SolidTag {
    type Handle = SolidKey;
}

/// An owned topology value with a typed primary handle.
///
/// Builders return `Shape` when they create a standalone topology. The `GMap`
/// is owned by the shape, while the marker type records which view should be
/// considered the primary result.
pub struct Shape<K: ShapeKind = SheetTag, P: Payload = StandardPayload> {
    map: GMap<P>,
    handle: K::Handle,
    kind: PhantomData<K>,
}

impl<K: ShapeKind, P: Payload> Shape<K, P> {
    /// Creates an owned shape from a map and primary handle.
    pub fn new(map: GMap<P>, handle: K::Handle) -> Self {
        Self {
            map,
            handle,
            kind: PhantomData,
        }
    }

    /// Returns the owned map by shared reference.
    pub fn map(&self) -> &GMap<P> {
        &self.map
    }

    /// Returns the owned map by mutable reference.
    ///
    /// Mutating the map can invalidate assumptions held by previously created
    /// typed views. Recreate views after structural edits.
    pub fn map_mut(&mut self) -> &mut GMap<P> {
        &mut self.map
    }

    /// Returns the primary handle of this shape.
    pub fn handle(&self) -> K::Handle {
        self.handle
    }

    /// Splits the shape into its owned map and primary handle.
    pub fn into_map(self) -> (GMap<P>, K::Handle) {
        (self.map, self.handle)
    }
}

impl<P: Payload> Shape<VertexTag, P> {
    /// Returns the primary vertex view.
    ///
    /// # Panics
    ///
    /// Panics if the stored vertex key is no longer present in the map.
    pub fn vertex(&self) -> Vertex<'_, P> {
        let v = self.map.vertex_attr_unchecked(self.handle);
        v.vertex(&self.map)
    }

    /// Returns the primary vertex key.
    pub fn key(&self) -> VertexKey {
        self.handle
    }
}

impl<P: Payload> Shape<EdgeTag, P> {
    /// Returns the primary edge view.
    ///
    /// # Panics
    ///
    /// Panics if the stored edge key is no longer present in the map.
    pub fn edge(&self) -> Edge<'_, P> {
        self.map.edge_unchecked(self.handle)
    }

    /// Returns the primary edge key.
    pub fn key(&self) -> EdgeKey {
        self.handle
    }
}

impl<P: Payload> Shape<FaceTag, P> {
    /// Returns the primary face view.
    ///
    /// # Panics
    ///
    /// Panics if the stored face key is no longer present in the map.
    pub fn face(&self) -> Face<'_, P> {
        let f = self.map.face_attr_unchecked(self.handle);
        f.face(&self.map)
    }

    /// Returns the primary face key.
    pub fn key(&self) -> FaceKey {
        self.handle
    }
}

impl<P: Payload> Shape<ProfileTag, P> {
    /// Returns the primary profile view.
    pub fn profile(&self) -> Profile<'_, P> {
        self.map.profile_unchecked(self.handle)
    }

    /// Returns the primary profile key.
    pub fn key(&self) -> ProfileKey {
        self.handle
    }
}

impl<P: Payload> Shape<SheetTag, P> {
    /// Returns the primary sheet view.
    pub fn sheet(&self) -> Sheet<'_, P> {
        self.map.sheet_unchecked(self.handle)
    }

    /// Returns the primary sheet key.
    pub fn key(&self) -> SheetKey {
        self.handle
    }
}

impl<P: Payload> Shape<SolidTag, P> {
    /// Returns the primary solid view.
    ///
    /// # Panics
    ///
    /// Panics if the stored solid key is no longer present in the map.
    pub fn solid(&self) -> Solid<'_, P> {
        self.map.solid_unchecked(self.handle)
    }

    /// Returns the primary solid key.
    pub fn key(&self) -> SolidKey {
        self.handle
    }
}
