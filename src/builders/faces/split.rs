use super::*;
use crate::builders::edges::{EdgeSplit, split_face_boundary_edge};
use crate::geometry::parameter::Fraction;
use crate::model::Model;
use crate::topology::ModelEdit;
use crate::topology::edge::Edge;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, FaceKey};

/// Splits a face-boundary edge and all of its incident face pcurves.
pub fn split_face_edge<P: Payload>(
    g: &mut Model<P>,
    face: FaceKey,
    edge: EdgeKey,
    parameter: Fraction,
) -> Result<EdgeSplit, FaceEdgeSplitError> {
    g.transaction(|edit| _split_face_edge(edit, face, edge, parameter))
}

/// Splits topology and all incident face pcurves in the same transaction.
pub(crate) fn _split_face_edge<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    face: FaceKey,
    edge: EdgeKey,
    parameter: Fraction,
) -> Result<EdgeSplit, FaceEdgeSplitError> {
    let boundary_dart = face_edge_dart(edit, face, edge)?;
    let reversed = closed_boundary_curve_reversed(edit, face, edge, boundary_dart)?;
    // Cutting an unmarked edge marks it and leaves one edge, so no pcurve is
    // cut in two. Each incident pcurve is turned to begin at the mark instead,
    // because that is where the marked edge's own span now begins.
    let separates = !matches!(edit.edge_unchecked(edge), Edge::Unmarked(_));
    let pcurves = separates
        .then(|| incident_face_pcurves(edit, edge, parameter))
        .transpose()?
        .unwrap_or_default();
    let rebased = (!separates)
        .then(|| rebased_face_pcurves(edit.model(), edge, parameter))
        .transpose()?
        .unwrap_or_default();

    let split = split_face_boundary_edge(edit, edge, parameter, reversed)?;
    for pcurve in pcurves {
        assign_split_pcurves(edit, pcurve)?;
    }
    for pcurve in rebased {
        assign_rebased_pcurve(edit, pcurve)?;
    }
    Ok(split)
}
