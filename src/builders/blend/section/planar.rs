//! A straight edge between two planes: a planar strip, or a cylinder.

use super::super::errors::BlendError;
use super::super::law::{BlendLaw, ChamferLaw, FilletLaw};
use super::super::network::NetworkEdge;
use super::crease::inward_directions;
use super::edge_section::{EdgeSection, SectionForm};
use crate::geometry::{Curve, Cylinder, LINEAR_TOLERANCE, Plane, Surface};
use crate::model::Model;
use crate::topology::payload::Payload;

/// Solves the section of a straight edge between two planes.
pub(super) fn planar_section<P: Payload>(
    model: &Model<P>,
    edge: &NetworkEdge,
    law: BlendLaw,
) -> Result<EdgeSection, BlendError> {
    let start = edge.span.start();
    let axis = (edge.span.end() - start).normalize();
    let normals = edge
        .sides
        .map(|side| *model.face_unchecked(side.face).normal_at(0.0, 0.0));
    let inward = inward_directions(model, edge, axis, normals);
    let convex = inward[0].dot(&normals[1]) < 0.0;
    if inward[0].dot(&normals[1]).abs() <= LINEAR_TOLERANCE.sqrt() {
        return Err(BlendError::FlatEdge { edge: edge.key });
    }
    let sign = if convex { 1.0 } else { -1.0 };

    match law {
        BlendLaw::Chamfer(ChamferLaw::Distance(distance)) => {
            let points = inward.map(|direction| start + direction * distance);
            let rails = points.map(|point| Curve::line(point, point + axis));
            let across = points[1] - points[0];
            let plane = Plane::new(points[0], axis, axis.cross(&across));
            Ok(EdgeSection {
                surface: Surface::Plane(plane.clone()),
                rails,
                convex,
                form: SectionForm::Strip { plane },
            })
        }
        BlendLaw::Fillet(FilletLaw::Radius(radius)) => {
            let fold = 1.0 + normals[0].dot(&normals[1]);
            if fold <= LINEAR_TOLERANCE.sqrt() {
                return Err(BlendError::UnsupportedEdge {
                    edge: edge.key,
                    reason: "its faces fold back onto each other",
                });
            }
            // The ball's centre is `radius` behind both faces, on the side the
            // material is for a convex edge and in front of both otherwise.
            let origin = start - (normals[0] + normals[1]) * (sign * radius / fold);
            let contacts = normals.map(|normal| origin + normal * (sign * radius));
            let rails = contacts.map(|point| Curve::line(point, point + axis));
            // The seam goes opposite the arc, so no blend face straddles it.
            let arc_middle = (normals[0] + normals[1]) * sign;
            let surface = Surface::Cylinder(Cylinder::new(origin, -arc_middle, axis, radius));
            Ok(EdgeSection {
                surface,
                rails,
                convex,
                form: SectionForm::Cylinder {
                    origin,
                    axis,
                    radius,
                },
            })
        }
    }
}
