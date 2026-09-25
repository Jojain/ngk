//! One selected edge ending where two unselected edges meet it.
//!
//! The blend runs out on the face opposite it. Each rail carries on until it
//! meets the unselected edge on its own face, and the end face gets one new
//! edge between those two points: where the blend's surface cuts the end
//! face's plane — a segment for a strip, a circle or ellipse for a cylinder.
//! The vertex's single fan is split by that edge into the two corners.

use super::super::errors::BlendError;
use super::super::section::SectionForm;
use super::blend::{VertexBlend, VertexContext};
use super::conic::cylinder_section_arc;
use crate::geometry::TrimmedCurve;
use crate::topology::payload::Payload;

/// Treats a trihedral vertex where exactly one selected edge ends.
pub(crate) fn run_out<P: Payload>(
    context: &VertexContext<'_, P>,
) -> Result<VertexBlend, BlendError> {
    let vertex = context.vertex;
    let first = vertex
        .ring
        .iter()
        .position(|slot| slot.selected.is_some())
        .ok_or_else(|| context.unsupported("no selected edge ends here"))?;
    // The selected edge, the face after it, the end face, the face before it.
    let ring = vertex.ring_from(first);
    let (edge, section) = context.section(&ring[0])?;
    let after = context.side_on(edge, ring[0].face)?;
    let before = context.side_on(edge, ring[2].face)?;
    let end_face = ring[1].face;
    if end_face == ring[0].face || end_face == ring[2].face {
        return Err(context.unsupported("the face it runs out on is one of its own"));
    }
    let end_plane = context.plane(end_face)?;

    let mut blend = VertexBlend::default();
    let landing_after = context.land(edge, after, ring[1].edge)?;
    let landing_before = context.land(edge, before, ring[2].edge)?;
    let corner_after = blend.corner(landing_after);
    let corner_before = blend.corner(landing_before);
    blend.landings.push((ring[1].edge, landing_after));
    blend.landings.push((ring[2].edge, landing_before));

    let near = vertex.point;
    let joint = blend.insert(
        context.model,
        &ring[1],
        [corner_after, corner_before],
        |from, to| match &section.form {
            SectionForm::Strip { .. } | SectionForm::Translated { .. } => {
                Ok(TrimmedCurve::segment(from, to))
            }
            SectionForm::Cylinder {
                origin,
                axis,
                radius,
            } => cylinder_section_arc(
                *origin,
                *axis,
                *radius,
                end_plane.origin(),
                *end_plane.normal(),
                from,
                to,
                near,
            )
            .ok_or_else(|| context.unsupported("its round does not cut the face it runs out on")),
        },
    )?;

    let mut rails = [corner_after; 2];
    rails[after] = corner_after;
    rails[before] = corner_before;
    blend.end(edge, context.end_of(edge), rails, &[joint])?;
    Ok(blend)
}
