//! Three selected edges meeting at a trihedral vertex.
//!
//! Cutting all three splits the vertex into one corner per face, where that
//! face's two rails meet. Three rounds of one radius are closed by the ball
//! that touches all three faces: each round ends on its cross-section through
//! the ball's centre, those cross-sections are great circles of the ball, and
//! the spherical triangle between them is the patch. Three strips meet at the
//! one point their planes share, each pair along the segment from their
//! common face corner to that point.

use nalgebra::{Matrix3, Vector3};

use super::super::errors::BlendError;
use super::super::section::{EdgeSection, SectionForm, line_line};
use super::blend::{VertexBlend, VertexContext};
use super::conic::cylinder_section_arc;
use crate::geometry::{Curve, Frame, LINEAR_TOLERANCE, Point3, Sphere, Surface, TrimmedCurve};
use crate::topology::payload::Payload;

/// Treats a trihedral vertex where all three edges are selected.
pub(crate) fn trihedral<P: Payload>(
    context: &VertexContext<'_, P>,
) -> Result<VertexBlend, BlendError> {
    let ring = &context.vertex.ring;
    let edges = ring
        .iter()
        .map(|slot| context.section(slot))
        .collect::<Result<Vec<_>, _>>()?;
    let convex = edges[0].1.convex;
    if edges.iter().any(|(_, section)| section.convex != convex) {
        return Err(context.unsupported("convex and concave blends meet here"));
    }

    // Face i lies between edge i and edge i + 1; its corner is where those two
    // edges' rails on it meet.
    let mut blend = VertexBlend::default();
    let mut corners = Vec::with_capacity(3);
    for index in 0..3 {
        let face = ring[index].face;
        let (this, this_section) = edges[index];
        let (next, next_section) = edges[(index + 1) % 3];
        let this_rail = rail_line(context, this_section, context.side_on(this, face)?)?;
        let next_rail = rail_line(context, next_section, context.side_on(next, face)?)?;
        let point = line_line(this_rail.0, this_rail.1, next_rail.0, next_rail.1)
            .ok_or_else(|| context.unsupported("two rails on one face do not meet"))?;
        corners.push(blend.corner(point));
    }
    // The rails of edge i run on faces i - 1 and i, so they stop at those
    // faces' corners.
    let rail_corners = |index: usize| -> Result<[usize; 2], BlendError> {
        let (edge, _) = edges[index];
        let mut rails = [0; 2];
        rails[context.side_on(edge, ring[(index + 2) % 3].face)?] = corners[(index + 2) % 3];
        rails[context.side_on(edge, ring[index].face)?] = corners[index];
        Ok(rails)
    };

    match edges
        .iter()
        .map(|(_, section)| &section.form)
        .collect::<Vec<_>>()[..]
    {
        [
            SectionForm::Cylinder { radius, .. },
            SectionForm::Cylinder { radius: second, .. },
            SectionForm::Cylinder { radius: third, .. },
        ] => {
            if (radius - second).abs() > LINEAR_TOLERANCE
                || (radius - third).abs() > LINEAR_TOLERANCE
            {
                return Err(context.unsupported("rounds of different radius meet here"));
            }
            let radius = *radius;
            let sign = if convex { 1.0 } else { -1.0 };
            let normals = ring
                .iter()
                .map(|slot| context.normal(slot.face))
                .collect::<Vec<_>>();
            let offsets = ring
                .iter()
                .zip(&normals)
                .map(|(slot, normal)| {
                    Ok(normal.dot(&context.plane(slot.face)?.origin().coords) - sign * radius)
                })
                .collect::<Result<Vec<_>, BlendError>>()?;
            let centre = solve_planes(&normals, &offsets)
                .ok_or_else(|| context.unsupported("its three faces do not meet at one point"))?;
            for (index, normal) in normals.iter().enumerate() {
                let contact = centre + normal * (sign * radius);
                if (contact - blend.corners[corners[index]]).norm() > LINEAR_TOLERANCE.sqrt() {
                    return Err(context.unsupported("its rounds do not meet on one ball"));
                }
            }
            let mut arcs = Vec::with_capacity(3);
            for index in 0..3 {
                let (edge, section) = edges[index];
                let SectionForm::Cylinder { origin, axis, .. } = &section.form else {
                    unreachable!("matched as a cylinder above");
                };
                let from = corners[(index + 2) % 3];
                let to = corners[index];
                let arc = cylinder_section_arc(
                    *origin,
                    *axis,
                    radius,
                    centre,
                    *axis,
                    blend.corners[from],
                    blend.corners[to],
                    context.vertex.point,
                )
                .ok_or_else(|| context.unsupported("a round does not end on the ball"))?;
                arcs.push(blend.joint(from, to, arc));
                let rails = rail_corners(index)?;
                blend.end(edge, context.end_of(edge), rails, &[arcs[index]])?;
            }
            let middle = normals.iter().sum::<Vector3<f64>>() * sign;
            let pole = any_perpendicular(middle);
            let ball = Sphere::new(Frame::from_xz(centre, -middle, pole), radius);
            blend.patch(Surface::Sphere(ball), corners[0], &arcs)?;
        }
        [
            SectionForm::Strip { plane },
            SectionForm::Strip { plane: second },
            SectionForm::Strip { plane: third },
        ] => {
            let planes = [plane, second, third];
            let normals = planes
                .iter()
                .map(|plane| *plane.normal())
                .collect::<Vec<_>>();
            let offsets = planes
                .iter()
                .map(|plane| plane.normal().dot(&plane.origin().coords))
                .collect::<Vec<_>>();
            let apex = solve_planes(&normals, &offsets)
                .ok_or_else(|| context.unsupported("its three bevels do not meet at one point"))?;
            let apex_corner = blend.corner(apex);
            // The mitre on face i's corner runs between bevels i and i + 1.
            let mitres = corners
                .iter()
                .map(|&corner| {
                    let from = blend.corners[corner];
                    if (apex - from).norm() <= LINEAR_TOLERANCE.sqrt() {
                        return Err(context.does_not_fit("its bevels meet at a face corner"));
                    }
                    Ok(blend.joint(corner, apex_corner, TrimmedCurve::segment(from, apex)))
                })
                .collect::<Result<Vec<_>, _>>()?;
            for index in 0..3 {
                let (edge, _) = edges[index];
                let rails = rail_corners(index)?;
                blend.end(
                    edge,
                    context.end_of(edge),
                    rails,
                    &[mitres[(index + 2) % 3], mitres[index]],
                )?;
            }
        }
        _ => return Err(context.unsupported("these three blends have no common corner")),
    }
    Ok(blend)
}

/// A rail's line: a point on it and its direction.
fn rail_line<P: Payload>(
    context: &VertexContext<'_, P>,
    section: &EdgeSection,
    side: usize,
) -> Result<(Point3, Vector3<f64>), BlendError> {
    match &section.rails[side] {
        Curve::Line(line) => Ok((line.origin(), *line.direction())),
        _ => Err(context.unsupported("only straight rails meet at a corner")),
    }
}

/// The point on three planes `normal · p = offset`, when they meet in one.
fn solve_planes(normals: &[Vector3<f64>], offsets: &[f64]) -> Option<Point3> {
    let matrix = Matrix3::from_rows(&[
        normals[0].transpose(),
        normals[1].transpose(),
        normals[2].transpose(),
    ]);
    let inverse = matrix.try_inverse()?;
    Some(Point3::from(
        inverse * Vector3::new(offsets[0], offsets[1], offsets[2]),
    ))
}

/// Some unit direction square to `direction`.
fn any_perpendicular(direction: Vector3<f64>) -> Vector3<f64> {
    let direction = direction.normalize();
    let helper = if direction.x.abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };
    (helper - direction * helper.dot(&direction)).normalize()
}
