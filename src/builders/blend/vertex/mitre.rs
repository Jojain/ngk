//! Two selected edges meeting where one unselected edge leaves.
//!
//! The two selected edges share a face, and their rails on it meet at the
//! corner of that face. On the faces across the unselected edge each blend
//! lands on that edge, and both must land on the same point of it: that is
//! the second corner, and the two blends meet along the curve from one corner
//! to the other — a segment where two strips cross, and where two cylinders
//! of one radius cross, the ellipse in the plane bisecting their axes.
//!
//! Where the two faces across the unselected edge make different angles with
//! the shared face, the blends land on different points and need a patch
//! between them; that is refused.

use super::super::errors::BlendError;
use super::super::section::{SectionForm, line_line};
use super::blend::{VertexBlend, VertexContext};
use super::conic::cylinder_section_arc;
use crate::geometry::{Curve, LINEAR_TOLERANCE, TrimmedCurve};
use crate::topology::payload::Payload;

/// Treats a trihedral vertex where exactly two selected edges end.
pub(crate) fn mitre<P: Payload>(context: &VertexContext<'_, P>) -> Result<VertexBlend, BlendError> {
    let vertex = context.vertex;
    let count = vertex.ring.len();
    let first = (0..count)
        .find(|&slot| {
            vertex.ring[slot].selected.is_some()
                && vertex.ring[(slot + 1) % count].selected.is_some()
        })
        .ok_or_else(|| context.unsupported("its two selected edges do not share a face"))?;
    // The two selected edges, with the face they share between them, then the
    // unselected edge and the two faces either side of it.
    let ring = vertex.ring_from(first);
    let shared = ring[0].face;
    let (edge_first, first_section) = context.section(&ring[0])?;
    let (edge_second, second_section) = context.section(&ring[1])?;
    if first_section.convex != second_section.convex {
        return Err(context.unsupported("a convex and a concave blend meet here"));
    }

    let first_shared = context.side_on(edge_first, shared)?;
    let first_across = context.side_on(edge_first, ring[2].face)?;
    let second_shared = context.side_on(edge_second, shared)?;
    let second_across = context.side_on(edge_second, ring[1].face)?;

    let rail_line =
        |section: &super::super::section::EdgeSection, side: usize| match &section.rails[side] {
            Curve::Line(line) => Ok((line.origin(), *line.direction())),
            _ => Err(context.unsupported("only straight rails are mitred")),
        };
    let (first_origin, first_direction) = rail_line(first_section, first_shared)?;
    let (second_origin, second_direction) = rail_line(second_section, second_shared)?;
    let inner = line_line(
        first_origin,
        first_direction,
        second_origin,
        second_direction,
    )
    .ok_or_else(|| context.unsupported("its rails on the shared face do not meet"))?;

    let landing = context.land(edge_first, first_across, ring[2].edge)?;
    let other_landing = context.land(edge_second, second_across, ring[2].edge)?;
    if (landing - other_landing).norm() > LINEAR_TOLERANCE.sqrt() {
        return Err(context.unsupported(
            "its two blends land apart on the edge between them, which needs a patch",
        ));
    }

    let near = vertex.point;
    let curve = match (&first_section.form, &second_section.form) {
        (SectionForm::Strip { .. }, SectionForm::Strip { .. }) => {
            TrimmedCurve::segment(inner, landing)
        }
        (
            SectionForm::Cylinder {
                origin,
                axis,
                radius,
            },
            SectionForm::Cylinder {
                origin: other_origin,
                axis: other_axis,
                radius: other_radius,
            },
        ) => {
            if (radius - other_radius).abs() > LINEAR_TOLERANCE {
                return Err(context.unsupported("rounds of different radius meet here"));
            }
            let centre = line_line(*origin, *axis, *other_origin, *other_axis)
                .ok_or_else(|| context.unsupported("its rounds' axes do not meet"))?;
            // Equal cylinders whose axes cross meet in the plane that mirrors
            // one axis onto the other, the one separating the two blends.
            let mirror = context.leaving(edge_first) - context.leaving(edge_second);
            cylinder_section_arc(
                *origin, *axis, *radius, centre, mirror, inner, landing, near,
            )
            .ok_or_else(|| context.unsupported("its rounds do not meet along one curve"))?
        }
        _ => return Err(context.unsupported("these two blends have no mitre")),
    };

    let mut blend = VertexBlend::default();
    let inner_corner = blend.corner(inner);
    let outer_corner = blend.corner(landing);
    blend.landings.push((ring[2].edge, landing));
    let joint = blend.joint(inner_corner, outer_corner, curve);

    let mut rails = [inner_corner; 2];
    rails[first_shared] = inner_corner;
    rails[first_across] = outer_corner;
    blend.end(edge_first, context.end_of(edge_first), rails, &[joint])?;
    let mut rails = [inner_corner; 2];
    rails[second_shared] = inner_corner;
    rails[second_across] = outer_corner;
    blend.end(edge_second, context.end_of(edge_second), rails, &[joint])?;
    Ok(blend)
}
