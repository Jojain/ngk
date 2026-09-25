//! Each selected edge's blend geometry, independent of how its ends are treated.
//!
//! A section is the surface the blend face lies on and, on each of the edge's
//! two faces, the rail the face now ends at. It is solved from a table keyed
//! by the two faces' supports, the edge's curve and the law, closed form
//! first — the same shape as the analytic intersection table — and nothing
//! downstream needs to know which row answered: vertex treatments read the
//! [`SectionForm`] to answer in closed form and refuse a form they do not know.

use std::collections::HashSet;

use nalgebra::Vector3;

use super::errors::BlendError;
use super::law::{BlendLaw, ChamferLaw, FilletLaw};
use super::network::NetworkEdge;
use crate::geometry::parameter::Fraction;
use crate::geometry::{
    Curve, Cylinder, LINEAR_TOLERANCE, Plane, Point3, Rigid, RuledSurface, Surface, TrimmedCurve,
};
use crate::model::Model;
use crate::topology::gmap::Dart;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::FaceKey;

/// One selected edge's blend geometry.
#[derive(Debug, Clone)]
pub(crate) struct EdgeSection {
    /// The support of the blend face.
    pub(crate) surface: Surface,
    /// The rail supports, one per network side: where each face now ends.
    pub(crate) rails: [Curve; 2],
    /// Whether the solid's material lies inside the crease, so the blend
    /// removes material rather than adding it.
    pub(crate) convex: bool,
    pub(crate) form: SectionForm,
}

/// The closed form a section was solved in.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum SectionForm {
    /// A planar chamfer strip between two straight rails.
    Strip { plane: Plane },
    /// A constant-radius fillet on a straight edge: the ball's centre runs
    /// along `axis` through `origin`.
    Cylinder {
        origin: Point3,
        axis: Vector3<f64>,
        radius: f64,
    },
    /// A chamfer between translated copies of the edge; `offsets` moves the
    /// edge onto each side's rail.
    Translated { offsets: [Vector3<f64>; 2] },
}

impl EdgeSection {
    /// The rail of `side`, cut down to run from `from` (at the edge's start)
    /// to `to` (at its end).
    pub(crate) fn rail_span(
        &self,
        side: usize,
        from: Point3,
        to: Point3,
        edge: &TrimmedCurve,
    ) -> TrimmedCurve {
        match self.form {
            // A translate keeps its parameterization, so the rail spans what
            // the edge did.
            SectionForm::Translated { .. } => {
                TrimmedCurve::new(self.rails[side].clone(), edge.interval())
            }
            _ => TrimmedCurve::between(self.rails[side].clone(), from, to),
        }
    }

    /// Where the rail of `side` meets the straight edge through `origin`
    /// along `direction`, next to the edge's end at `vertex`.
    pub(crate) fn land(
        &self,
        side: usize,
        vertex: Point3,
        origin: Point3,
        direction: Vector3<f64>,
    ) -> Option<Point3> {
        match &self.form {
            SectionForm::Translated { offsets } => {
                let point = vertex + offsets[side];
                (distance_to_line(point, origin, direction) <= LINEAR_TOLERANCE.sqrt())
                    .then_some(point)
            }
            _ => {
                let Curve::Line(rail) = &self.rails[side] else {
                    return None;
                };
                line_line(rail.origin(), *rail.direction(), origin, direction)
            }
        }
    }
}

/// Solves one selected edge's section under `law`.
pub(crate) fn solve_section<P: Payload>(
    model: &Model<P>,
    edge: &NetworkEdge,
    law: BlendLaw,
) -> Result<EdgeSection, BlendError> {
    let surfaces = edge
        .sides
        .map(|side| model.face_attr_unchecked(side.face).surface.clone());
    match (&surfaces[0], &surfaces[1], edge.span.curve(), law) {
        // Two planes meet in a line, whatever curve type carries it: a
        // Boolean writes straight sections as NURBS as often as not.
        (Surface::Plane(_), Surface::Plane(_), _, _) if is_straight(&edge.span) => {
            planar_section(model, edge, law)
        }
        (Surface::Plane(_), Surface::Ruled(_), Curve::Nurbs(_), BlendLaw::Chamfer(chamfer))
        | (Surface::Ruled(_), Surface::Plane(_), Curve::Nurbs(_), BlendLaw::Chamfer(chamfer)) => {
            let ChamferLaw::Distance(distance) = chamfer;
            translated_section(model, edge, distance)
        }
        _ => Err(BlendError::UnsupportedEdge {
            edge: edge.key,
            reason: "its faces and curve are outside the section table",
        }),
    }
}

/// A straight edge between two planes: a planar strip, or a cylinder.
fn planar_section<P: Payload>(
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

/// An edge where a plane meets an extruded wall, chamfered between two
/// translated copies of the edge.
///
/// The wall's rail is the edge moved `distance` down the wall's rulings. The
/// plane's is the edge moved `distance` across its chord, which is a true
/// setback only where the edge runs parallel to its chord: this is a
/// translated chamfer, not a constant one.
fn translated_section<P: Payload>(
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

/// The unit direction from the edge into each face, square to `tangent`.
///
/// A face's interior lies to the left of its boundary as the face walks it,
/// seen from its outward normal, so the direction is read off the way each
/// face's stored loop runs along the edge.
fn inward_directions<P: Payload>(
    model: &Model<P>,
    edge: &NetworkEdge,
    tangent: Vector3<f64>,
    normals: [Vector3<f64>; 2],
) -> [Vector3<f64>; 2] {
    std::array::from_fn(|index| {
        let side = edge.sides[index];
        let walked = if runs_along(model, side.face, side.start) {
            tangent
        } else {
            -tangent
        };
        normals[index].cross(&walked).normalize()
    })
}

/// Whether `face`'s stored loops run along `dart` in its own direction.
pub(crate) fn runs_along<P: Payload>(model: &Model<P>, face: FaceKey, dart: Dart) -> bool {
    loop_darts(model, face).contains(&dart)
}

/// Every dart `face`'s stored loops walk.
pub(crate) fn loop_darts<P: Payload>(model: &Model<P>, face: FaceKey) -> HashSet<Dart> {
    model
        .face_unchecked(face)
        .loops()
        .iter()
        .flat_map(|loop_| loop_.darts())
        .collect()
}

/// Whether a span runs along its chord, whichever curve type carries it.
pub(crate) fn is_straight(span: &TrimmedCurve) -> bool {
    if matches!(span.curve(), Curve::Line(_)) {
        return true;
    }
    let (start, end) = (span.start(), span.end());
    let chord = end - start;
    if chord.norm() <= LINEAR_TOLERANCE {
        return false;
    }
    (1..8).all(|index| {
        let point = span.point_at(Fraction::new(f64::from(index) / 8.0));
        distance_to_line(point, start, chord) <= LINEAR_TOLERANCE
    })
}

/// Where two lines meet, or `None` when they are parallel or miss each other.
pub(crate) fn line_line(
    first: Point3,
    first_direction: Vector3<f64>,
    second: Point3,
    second_direction: Vector3<f64>,
) -> Option<Point3> {
    let d = first_direction.normalize();
    let e = second_direction.normalize();
    let between = second - first;
    let cosine = d.dot(&e);
    let denominator = 1.0 - cosine * cosine;
    if denominator <= LINEAR_TOLERANCE {
        return None;
    }
    let s = (between.dot(&d) - cosine * between.dot(&e)) / denominator;
    let t = (cosine * between.dot(&d) - between.dot(&e)) / denominator;
    let on_first = first + d * s;
    let on_second = second + e * t;
    ((on_first - on_second).norm() <= LINEAR_TOLERANCE.sqrt())
        .then(|| Point3::from((on_first.coords + on_second.coords) * 0.5))
}

/// Distance from `point` to the line through `origin` along `direction`.
pub(crate) fn distance_to_line(point: Point3, origin: Point3, direction: Vector3<f64>) -> f64 {
    let direction = direction.normalize();
    let offset = point - origin;
    (offset - direction * offset.dot(&direction)).norm()
}
