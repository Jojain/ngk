//! An edge where a plane meets an extruded wall, chamfered between two
//! translated copies of the edge.

use nalgebra::Vector3;

use super::super::errors::BlendError;
use super::super::network::NetworkEdge;
use super::crease::inward_directions;
use super::edge_section::{EdgeSection, SectionForm};
use crate::geometry::parameter::Fraction;
use crate::geometry::{LINEAR_TOLERANCE, Rigid, RuledSurface, Surface};
use crate::model::Model;
use crate::topology::payload::Payload;

/// Chamfers the edge where a plane meets an extruded wall.
///
/// The wall's rail is the edge moved `distance` down the wall's rulings. The
/// plane's is the edge moved `distance` across its chord, which is a true
/// setback only where the edge runs parallel to its chord: this is a
/// translated chamfer, not a constant one.
pub(super) fn translated_section<P: Payload>(
    model: &Model<P>,
    edge: &NetworkEdge,
    distance: f64,
) -> Result<EdgeSection, BlendError> {
    let unsupported = |reason| BlendError::UnsupportedEdge {
        edge: edge.key,
        reason,
    };
    let (plane_side, wall_side) = match model.face_attr_unchecked(edge.sides[0].face).surface {
        Surface::Plane(_) => (0, 1),
        _ => (1, 0),
    };
    let Surface::Ruled(wall) = model
        .face_attr_unchecked(edge.sides[wall_side].face)
        .surface
        .clone()
    else {
        return Err(unsupported("its wall is not a ruled surface"));
    };
    let wall_surface = Surface::Ruled(wall.clone());
    let heights = [0.0, 0.25, 0.5, 0.75, 1.0].map(|fraction| {
        wall_surface
            .param_at(edge.span.point_at(Fraction::new(fraction)))
            .map(|uv| uv.y)
    });
    let heights = heights
        .into_iter()
        .collect::<Result<Vec<_>, _>>()
        .map_err(|_| unsupported("it cannot be located on its wall"))?;
    if heights
        .iter()
        .any(|height| (height - heights[0]).abs() > LINEAR_TOLERANCE.sqrt())
    {
        return Err(unsupported("it is not a translate of its wall's base"));
    }

    let middle = edge.span.point_at(Fraction::new(0.5));
    let tangent = edge.span.derivative_at(Fraction::new(0.5), 1).normalize();
    let mut normals = [Vector3::zeros(); 2];
    for (normal, side) in normals.iter_mut().zip(&edge.sides) {
        let face = model.face_unchecked(side.face);
        let uv = face
            .surface()
            .param_at(middle)
            .map_err(|_| unsupported("it cannot be located on its faces"))?;
        *normal = *face.normal_at(uv.x, uv.y);
    }
    let inward = inward_directions(model, edge, tangent, normals);
    let ruling = wall.direction().normalize();
    let along_ruling = inward[wall_side].dot(&ruling);
    if along_ruling.abs() < 1.0 - LINEAR_TOLERANCE.sqrt() {
        return Err(unsupported("its wall's rulings are not square to it"));
    }
    let chord = edge.span.end() - edge.span.start();
    let across = normals[plane_side].cross(&chord).normalize();
    let across = if across.dot(&inward[plane_side]) < 0.0 {
        -across
    } else {
        across
    };
    let mut offsets = [Vector3::zeros(); 2];
    offsets[wall_side] = ruling * (distance * along_ruling.signum());
    offsets[plane_side] = across * distance;
    let rails = offsets.map(|offset| edge.span.curve().moved(&Rigid::translation(offset)));
    let convex = inward[0].dot(&normals[1]) < 0.0;
    let surface = Surface::Ruled(RuledSurface::new(
        rails[plane_side].clone(),
        offsets[wall_side] - offsets[plane_side],
    ));
    Ok(EdgeSection {
        surface,
        rails,
        convex,
        form: SectionForm::Translated { offsets },
    })
}
