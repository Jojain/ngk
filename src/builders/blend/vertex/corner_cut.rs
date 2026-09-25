//! A solid corner cut off by a plane, with no edge selected.
//!
//! Each edge at the vertex is cut back `distance`, each face gets the segment
//! between its two cut points in its corner, and the triangle through the
//! three points closes the solid. Every face corner is replaced, so the
//! vertex falls apart into one piece per edge.

use super::super::errors::BlendError;
use super::blend::{VertexBlend, VertexContext};
use crate::geometry::{Plane, Surface, TrimmedCurve};
use crate::topology::payload::Payload;

/// Cuts a trihedral solid corner `distance` back along each of its edges.
pub(crate) fn corner_cut<P: Payload>(
    context: &VertexContext<'_, P>,
    distance: f64,
) -> Result<VertexBlend, BlendError> {
    let ring = &context.vertex.ring;
    if ring.len() != 3 {
        return Err(context.unsupported("only a corner where three faces meet is cut"));
    }
    let mut blend = VertexBlend::default();
    let mut corners = Vec::with_capacity(3);
    for slot in ring {
        context.plane(slot.face)?;
        let (direction, length) = context.straight(slot.edge)?;
        let point = context.vertex.point + direction * distance;
        context.inside(point, direction, length)?;
        blend.landings.push((slot.edge, point));
        corners.push(blend.corner(point));
    }
    let mut joints = Vec::with_capacity(3);
    for (index, slot) in ring.iter().enumerate() {
        joints.push(blend.insert(
            context.model,
            slot,
            [corners[index], corners[(index + 1) % 3]],
            |from, to| Ok(TrimmedCurve::segment(from, to)),
        )?);
    }
    let [first, second, third] = [0, 1, 2].map(|index| blend.corners[corners[index]]);
    let normal = (second - first).cross(&(third - first));
    let plane = Plane::new(first, second - first, normal);
    blend.patch(Surface::Plane(plane), corners[0], &joints)?;
    Ok(blend)
}
