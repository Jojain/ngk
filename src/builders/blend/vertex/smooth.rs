//! Two selected edges running on into each other without a corner.
//!
//! Where one edge leaves a vertex along the tangent the other arrives on —
//! a straight rim running into an arc, two arcs of a slot — the faces either
//! side run on smoothly too, and the two blends share their cross-section at
//! the vertex: the plane square to the common tangent cuts both blends in
//! the same arc, or the same segment. That section is the joint between the
//! two blend faces. It runs from where both rails on the face the edges share
//! meet, to where the rails on the faces across the unselected edge meet on
//! that edge, which is cut back to it.
//!
//! Any two sections that agree there join, whatever solved them: a cylinder
//! and a torus along a slot's rim, two skins along a spline. Sections that do
//! not agree — the faces across the unselected edge meet at a crease, or the
//! blends are of different sizes — are refused.

use nalgebra::Vector3;

use super::super::errors::BlendError;
use super::super::section::EdgeSection;
use super::blend::{VertexBlend, VertexContext};
use crate::geometry::parameter::NativeParam;
use crate::geometry::{Circle, Curve, LINEAR_TOLERANCE, Plane, Point3, TrimmedCurve};
use crate::topology::payload::Payload;
use crate::topology::shape_keys::FaceKey;

/// Newton steps a rail is given to reach the section plane.
const MAX_STEPS: usize = 32;

/// Whether the two selected edges at a trihedral vertex run on into each
/// other: one leaves along the tangent the other arrives on.
pub(crate) fn runs_smoothly<P: Payload>(context: &VertexContext<'_, P>) -> bool {
    let selected = context
        .vertex
        .ring
        .iter()
        .filter_map(|slot| slot.selected)
        .collect::<Vec<_>>();
    let [first, second] = selected.as_slice() else {
        return false;
    };
    let turn = context
        .tangent_leaving(*first)
        .dot(&context.tangent_leaving(*second));
    turn <= -1.0 + LINEAR_TOLERANCE.sqrt()
}

/// Joins two blends running on into each other along their common section.
pub(crate) fn smooth<P: Payload>(
    context: &VertexContext<'_, P>,
) -> Result<VertexBlend, BlendError> {
    let vertex = context.vertex;
    let count = vertex.ring.len();
    let first = (0..count)
        .find(|&slot| {
            vertex.ring[slot].selected.is_some()
                && vertex.ring[(slot + 1) % count].selected.is_some()
        })
        .ok_or_else(|| context.unsupported("its two selected edges do not share a face"))?;
    // The two selected edges with the face they share between them, then the
    // unselected edge between the two faces across.
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

    let tangent = context.tangent_leaving(edge_first);
    let first_points = section_points(context, first_section, tangent)?;
    let second_points = section_points(context, second_section, tangent)?;
    let inner = first_points[first_shared];
    let outer = first_points[first_across];
    let apart = |a: Point3, b: Point3| (a - b).norm() > LINEAR_TOLERANCE.sqrt();
    if apart(inner, second_points[second_shared]) || apart(outer, second_points[second_across]) {
        return Err(context.unsupported("its two blends do not meet in one section"));
    }
    context.on_edge(ring[2].edge, outer)?;
    let curve = section_curve(context, first_section, shared, inner, outer)?;

    let mut blend = VertexBlend::default();
    let inner_corner = blend.corner(inner);
    let outer_corner = blend.corner(outer);
    blend.landings.push((ring[2].edge, outer));
    let joint = blend.joint(inner_corner, outer_corner, curve);
    for (edge, shared_side, across_side) in [
        (edge_first, first_shared, first_across),
        (edge_second, second_shared, second_across),
    ] {
        let mut rails = [inner_corner; 2];
        rails[shared_side] = inner_corner;
        rails[across_side] = outer_corner;
        blend.end(edge, context.end_of(edge), rails, &[joint])?;
    }
    Ok(blend)
}

/// Where each of a section's rails crosses the plane through the vertex
/// square to `tangent`.
fn section_points<P: Payload>(
    context: &VertexContext<'_, P>,
    section: &EdgeSection,
    tangent: Vector3<f64>,
) -> Result<[Point3; 2], BlendError> {
    let point = |rail: &Curve| {
        rail_in_plane(rail, context.vertex.point, tangent)
            .ok_or_else(|| context.unsupported("a rail does not reach the section at its end"))
    };
    Ok([point(&section.rails[0])?, point(&section.rails[1])?])
}

/// The point of `rail` in the plane through `origin` square to `normal`,
/// searched from the rail's point nearest `origin`.
fn rail_in_plane(rail: &Curve, origin: Point3, normal: Vector3<f64>) -> Option<Point3> {
    let mut t = rail.parameter_at(rail.project(origin)).value();
    for _ in 0..MAX_STEPS {
        let point = rail.point_at(NativeParam::new(t));
        let offset = (point - origin).dot(&normal);
        if offset.abs() <= LINEAR_TOLERANCE * 0.1 {
            return Some(point);
        }
        let slope = rail.derivative_at(NativeParam::new(t), 1).dot(&normal);
        if slope.abs() <= LINEAR_TOLERANCE {
            return None;
        }
        t -= offset / slope;
    }
    None
}

/// The section of a blend between its two contact points: the arc of the
/// ball touching both faces for a fillet, the segment for a chamfer.
///
/// The ball's centre is read off the contact on the shared face: one radius
/// behind the face on a convex crease, one in front of it on a concave one.
fn section_curve<P: Payload>(
    context: &VertexContext<'_, P>,
    section: &EdgeSection,
    shared: FaceKey,
    inner: Point3,
    outer: Point3,
) -> Result<TrimmedCurve, BlendError> {
    let Some(radius) = section.fillet_radius() else {
        return Ok(TrimmedCurve::segment(inner, outer));
    };
    let sign = if section.convex { 1.0 } else { -1.0 };
    let normal = context
        .outward(shared, inner)
        .ok_or_else(|| context.unsupported("a face around it cannot be located"))?;
    let centre = inner - normal * (sign * radius);
    let (from, to) = (inner - centre, outer - centre);
    if (to.norm() - radius).abs() > LINEAR_TOLERANCE.sqrt() {
        return Err(context.unsupported("its two blends do not meet in one section"));
    }
    let plane = Plane::new(centre, from, from.cross(&to));
    Ok(TrimmedCurve::between(
        Curve::Circle(Circle::new(plane, radius)),
        inner,
        outer,
    ))
}
