use std::convert::Infallible;

use crate::model::Model;
use crate::topology::edit::{EditKey, EditPolicy, Origin};
use crate::topology::shape_keys::{EdgeKey, FaceKey, ProfileKey, SheetKey, SolidKey, VertexKey};

/// Records the creation lineage delivered to a transaction policy.
pub(crate) struct LineageRecorder<P: crate::topology::payload::Payload> {
    pub created: Vec<(EditKey, Origin)>,
    payload: P::Policy,
}

impl<P> Default for LineageRecorder<P>
where
    P: crate::topology::payload::Payload,
    P::Policy: Default,
{
    fn default() -> Self {
        Self {
            created: Vec::new(),
            payload: P::Policy::default(),
        }
    }
}

impl<P: crate::topology::payload::Payload> LineageRecorder<P> {
    fn record(&mut self, key: EditKey, origin: Origin) {
        self.created.push((key, origin));
    }

    /// Asserts that the policy saw exactly the expected creation events,
    /// without making callers depend on hook declaration order.
    pub(crate) fn assert_exact(&self, expected: &[(EditKey, Origin)]) {
        let mut remaining = self.created.clone();
        for event in expected {
            let index = remaining
                .iter()
                .position(|candidate| candidate == event)
                .unwrap_or_else(|| panic!("missing lineage event {event:?}"));
            remaining.swap_remove(index);
        }
        assert!(
            remaining.is_empty(),
            "unexpected lineage events: {remaining:?}"
        );
    }
}

impl<P> EditPolicy<P> for LineageRecorder<P>
where
    P: crate::topology::payload::Payload,
    P::Policy: EditPolicy<P, Error = Infallible>,
{
    type Error = Infallible;

    fn vertex_created(
        &mut self,
        key: VertexKey,
        origin: Origin,
        before: &Model<P>,
    ) -> Result<P::V, Self::Error> {
        self.record(EditKey::Vertex(key), origin.clone());
        self.payload.vertex_created(key, origin, before)
    }

    fn edge_created(
        &mut self,
        key: EdgeKey,
        origin: Origin,
        before: &Model<P>,
    ) -> Result<P::E, Self::Error> {
        self.record(EditKey::Edge(key), origin.clone());
        self.payload.edge_created(key, origin, before)
    }

    fn profile_created(
        &mut self,
        key: ProfileKey,
        origin: Origin,
        before: &Model<P>,
    ) -> Result<P::Profile, Self::Error> {
        self.record(EditKey::Profile(key), origin.clone());
        self.payload.profile_created(key, origin, before)
    }

    fn face_created(
        &mut self,
        key: FaceKey,
        origin: Origin,
        before: &Model<P>,
    ) -> Result<P::F, Self::Error> {
        self.record(EditKey::Face(key), origin.clone());
        self.payload.face_created(key, origin, before)
    }

    fn sheet_created(
        &mut self,
        key: SheetKey,
        origin: Origin,
        before: &Model<P>,
    ) -> Result<P::Sheet, Self::Error> {
        self.record(EditKey::Sheet(key), origin.clone());
        self.payload.sheet_created(key, origin, before)
    }

    fn solid_created(
        &mut self,
        key: SolidKey,
        origin: Origin,
        before: &Model<P>,
    ) -> Result<P::S, Self::Error> {
        self.record(EditKey::Solid(key), origin.clone());
        self.payload.solid_created(key, origin, before)
    }
}
