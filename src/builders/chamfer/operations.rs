//! Chamfer operations and edit-scoped implementations.

use crate::builders::blend::{BlendError, BlendLaw, BlendTarget, ChamferLaw, blend_edit};
use crate::model::{Model, OpResult, StaleResult};
use crate::topology::ModelEdit;
use crate::topology::face::Face as FaceView;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, FaceKey};

/// Faces a chamfer added and the solid edges it replaced.
#[derive(Debug)]
pub struct TargetChamfer {
    /// Every face the chamfer added: one bevel per solid edge, and the face
    /// closing each cut corner.
    pub faces: Vec<FaceKey>,
    /// Every solid edge a bevel replaced. These keys no longer exist.
    pub consumed_edges: Vec<EdgeKey>,
    revision: Option<u64>,
}

/// Borrowed views for a committed [`TargetChamfer`] result.
pub struct TargetChamferView<'m, P: Payload> {
    pub faces: Vec<FaceView<'m, P>>,
}

impl OpResult for TargetChamfer {
    fn stamp(&mut self, revision: u64) {
        self.revision = Some(revision);
    }
}

impl TargetChamfer {
    /// Resolves the result against the model revision at which it committed.
    pub fn view<'m, P: Payload>(
        &self,
        model: &'m Model<P>,
    ) -> Result<TargetChamferView<'m, P>, StaleResult> {
        StaleResult::check(self.revision, model)?;
        Ok(TargetChamferView {
            faces: self
                .faces
                .iter()
                .map(|&key| model.face_unchecked(key))
                .collect(),
        })
    }
}

/// Chamfers a selection in place, setting every rail back `distance`.
///
/// The target is any mix of vertices, edges, profiles and faces, resolved as
/// one set:
///
/// - a corner of a wire or free planar face is cut by a straight edge between
///   the points `distance` along each of its edges;
/// - a solid edge between two planar faces becomes a planar bevel whose rails
///   lie `distance` from the edge inside each face, on convex and concave
///   edges alike; an edge where a plane meets an extruded wall becomes a ruled
///   bevel between translated copies of the edge;
/// - a solid profile or face selects every edge of it;
/// - a solid vertex selected on its own has its corner cut off by a triangle
///   through the points `distance` along its three edges.
///
/// Selected edges sharing a vertex are chamfered together: bevels meeting at
/// a vertex are mitred, and three meeting at a corner meet at the point their
/// planes share. The whole selection is planned against the model as it was
/// before the call, so the order it is listed in cannot change the result.
///
/// # Errors
///
/// Returns [`BlendError`] naming the entity when the distance is invalid, the
/// selection is outside what the planner supports, or the chamfer does not
/// fit. The model is unchanged.
pub fn chamfer<P: Payload, T: Into<BlendTarget>>(
    g: &mut Model<P>,
    target: T,
    distance: f64,
) -> Result<TargetChamfer, BlendError> {
    g.transaction_result(|edit| chamfer_edit(edit, target, distance))
}

/// Applies a chamfer inside an existing edit and reports what it added.
pub(crate) fn chamfer_edit<P: Payload, T: Into<BlendTarget>>(
    edit: &mut ModelEdit<'_, P>,
    target: T,
    distance: f64,
) -> Result<TargetChamfer, BlendError> {
    let outcome = blend_edit(
        edit,
        target.into(),
        BlendLaw::Chamfer(ChamferLaw::Distance(distance)),
    )?;
    Ok(TargetChamfer {
        faces: outcome.faces,
        consumed_edges: outcome.consumed_edges,
        revision: None,
    })
}
