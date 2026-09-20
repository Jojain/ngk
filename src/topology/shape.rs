use std::marker::PhantomData;

use crate::model::Model;
use crate::topology::edge::Edge;
use crate::topology::face::Face;
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
/// Builders return `Shape` when they create a standalone topology. The
/// [`Model`] is owned by the shape, while the marker type records which view
/// should be considered the primary result.
pub struct Shape<K: ShapeKind = SheetTag, P: Payload = StandardPayload> {
    model: Model<P>,
    handle: K::Handle,
    kind: PhantomData<K>,
}

impl<K: ShapeKind, P: Payload> Shape<K, P> {
    /// Runs one builder into a fresh model and owns its primary result.
    pub fn build<E>(f: impl FnOnce(&mut Model<P>) -> Result<K::Handle, E>) -> Result<Self, E> {
        let mut model = Model::new();
        let handle = f(&mut model)?;
        Ok(Self::new(model, handle))
    }

    /// Runs one builder over this shape's model and rebinds its primary handle.
    pub fn then<K2: ShapeKind, E>(
        mut self,
        f: impl FnOnce(&mut Model<P>, K::Handle) -> Result<K2::Handle, E>,
    ) -> Result<Shape<K2, P>, E> {
        let handle = f(&mut self.model, self.handle)?;
        Ok(Shape::new(self.model, handle))
    }

    /// Creates an owned shape from a map and primary handle.
    pub fn new(model: Model<P>, handle: K::Handle) -> Self {
        Self {
            model,
            handle,
            kind: PhantomData,
        }
    }

    /// Returns the owned model by shared reference.
    pub fn model(&self) -> &Model<P> {
        &self.model
    }

    /// Returns the owned model by mutable reference.
    ///
    /// Mutating the model can invalidate assumptions held by previously created
    /// typed views. Recreate views after structural edits.
    pub fn model_mut(&mut self) -> &mut Model<P> {
        &mut self.model
    }

    /// Returns the primary handle of this shape.
    pub fn handle(&self) -> K::Handle {
        self.handle
    }

    /// Splits the shape into its owned model and primary handle.
    pub fn into_model(self) -> (Model<P>, K::Handle) {
        (self.model, self.handle)
    }
}

impl<P: Payload> Shape<VertexTag, P> {
    /// Returns the primary vertex view.
    ///
    /// # Panics
    ///
    /// Panics if the stored vertex key is no longer present in the map.
    pub fn vertex(&self) -> Vertex<'_, P> {
        let v = self.model.vertex_attr_unchecked(self.handle);
        v.vertex(&self.model)
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
        self.model.edge_unchecked(self.handle)
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
        let f = self.model.face_attr_unchecked(self.handle);
        f.face(&self.model)
    }

    /// Returns the primary face key.
    pub fn key(&self) -> FaceKey {
        self.handle
    }
}

impl<P: Payload> Shape<ProfileTag, P> {
    /// Returns the primary profile view.
    pub fn profile(&self) -> Profile<'_, P> {
        self.model.profile_unchecked(self.handle)
    }

    /// Returns the primary profile key.
    pub fn key(&self) -> ProfileKey {
        self.handle
    }
}

impl<P: Payload> Shape<SheetTag, P> {
    /// Returns the primary sheet view.
    pub fn sheet(&self) -> Sheet<'_, P> {
        self.model.sheet_unchecked(self.handle)
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
        self.model.solid_unchecked(self.handle)
    }

    /// Returns the primary solid key.
    pub fn key(&self) -> SolidKey {
        self.handle
    }
}
