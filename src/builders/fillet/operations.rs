//! Fillet operations and edit-scoped implementations.

use crate::builders::blend::{BlendError, BlendLaw, BlendTarget, FilletLaw, blend_edit};
use crate::model::{Model, OpResult, StaleResult};
use crate::topology::ModelEdit;
use crate::topology::face::Face as FaceView;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, FaceKey};

/// Faces a fillet added and the solid edges it replaced.
#[derive(Debug)]
pub struct TargetFillet {
    /// Every face the fillet added: one round per solid edge, and the ball
    /// closing each corner where three rounds meet.
    pub faces: Vec<FaceKey>,
    /// Every solid edge a round replaced. These keys no longer exist.
    pub consumed_edges: Vec<EdgeKey>,
    revision: Option<u64>,
}

/// Borrowed views for a committed [`TargetFillet`] result.
pub struct TargetFilletView<'m, P: Payload> {
    pub faces: Vec<FaceView<'m, P>>,
}

impl OpResult for TargetFillet {
    fn stamp(&mut self, revision: u64) {
        self.revision = Some(revision);
    }
}

impl TargetFillet {
    /// Resolves the result against the model revision at which it committed.
    pub fn view<'m, P: Payload>(
        &self,
        model: &'m Model<P>,
    ) -> Result<TargetFilletView<'m, P>, StaleResult> {
        StaleResult::check(self.revision, model)?;
        Ok(TargetFilletView {
            faces: self
                .faces
                .iter()
                .map(|&key| model.face_unchecked(key))
                .collect(),
        })
    }
}

/// Rounds a selection in place with a constant `radius`.
///
/// The target is any mix of vertices, edges, profiles and faces, resolved as
/// one set:
///
/// - a corner of a wire or free planar face is replaced by the arc of radius
///   `radius` tangent to both of its edges, which may be lines or arcs;
/// - a solid edge between two planar faces is replaced by the cylinder a ball
///   of radius `radius` sweeps touching both faces, on convex and concave
///   edges alike;
/// - a solid profile or face selects every edge of it.
///
/// Selected edges sharing a vertex are rounded together. Where one ends
/// against an unselected face the round runs out on it; two meeting at a
/// vertex are mitred along the curve their cylinders share; three meeting at
/// a corner are closed by the sphere the ball touches all three faces with.
/// The whole selection is planned against the model as it was before the
/// call, so the order it is listed in cannot change the result.
///
/// A solid vertex on its own is refused: rounding a corner means rounding the
/// edges that meet there, which is a selection of edges.
///
/// # Errors
///
/// Returns [`BlendError`] naming the entity when the radius is invalid, the
/// selection is outside what the planner supports, or the fillet does not
/// fit. The model is unchanged.
pub fn fillet<P: Payload, T: Into<BlendTarget>>(
    g: &mut Model<P>,
    target: T,
    radius: f64,
) -> Result<TargetFillet, BlendError> {
    g.transaction_result(|edit| fillet_edit(edit, target, radius))
}

/// Applies a fillet inside an existing edit and reports what it added.
pub(crate) fn fillet_edit<P: Payload, T: Into<BlendTarget>>(
    edit: &mut ModelEdit<'_, P>,
    target: T,
    radius: f64,
) -> Result<TargetFillet, BlendError> {
    let outcome = blend_edit(
        edit,
        target.into(),
        BlendLaw::Fillet(FilletLaw::Radius(radius)),
    )?;
    Ok(TargetFillet {
        faces: outcome.faces,
        consumed_edges: outcome.consumed_edges,
        revision: None,
    })
}

#[cfg(test)]
mod tests {
    use super::fillet_edit;
    use crate::builders::test_support::LineageRecorder;
    use crate::modeling::solids::block;
    use crate::topology::edit::{EditKey, Origin};

    #[test]
    fn a_round_derives_from_the_edge_and_vertices_it_replaces() {
        let mut shape = block(2.0, 3.0, 4.0).expect("block should build");
        let edge = shape
            .solid()
            .edges()
            .into_iter()
            .find(|edge| {
                let bounded = edge.bounded_unchecked();
                let (start, end) = (*bounded.start().point(), *bounded.end().point());
                start.x.abs() + start.y.abs() + end.x.abs() + end.y.abs() < 1.0e-9
            })
            .expect("block should have a vertical edge at the origin");
        let ends = [
            EditKey::Vertex(edge.bounded_unchecked().start().key()),
            EditKey::Vertex(edge.bounded_unchecked().end().key()),
        ];
        let edge = edge.key();
        let mut recorder = LineageRecorder::default();

        let result = shape
            .model_mut()
            .transaction_with_policy(&mut recorder, |edit| fillet_edit(edit, edge, 0.25))
            .expect("block edge should round");

        let from_edge = Origin::derived(EditKey::Edge(edge));
        assert!(
            recorder
                .created
                .contains(&(EditKey::Face(result.faces[0]), from_edge.clone()))
        );
        let rails = recorder
            .created
            .iter()
            .filter(|(key, origin)| matches!(key, EditKey::Edge(_)) && *origin == from_edge)
            .count();
        assert_eq!(rails, 2, "each face's side of the edge becomes a rail");
        for (key, origin) in &recorder.created {
            if let EditKey::Vertex(_) = key {
                let Origin::Derived { sources } = origin else {
                    panic!("a blend's corner should derive from a vertex, got {origin:?}");
                };
                assert!(sources.len() == 1 && ends.contains(&sources[0]));
            }
        }
    }
}
