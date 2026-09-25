//! What a section is, and the table that solves one.

use nalgebra::Vector3;

use super::super::errors::BlendError;
use super::super::law::{BlendLaw, ChamferLaw};
use super::super::network::NetworkEdge;
use super::super::pcurve::pcurve_on;
use super::crease::Crease;
use super::lines::{distance_to_line, is_straight, line_line};
use super::planar::planar_section;
use super::revolved::revolved_section;
use super::swept::swept_section;
use super::translated::translated_section;
use crate::geometry::{
    Curve, LINEAR_TOLERANCE, Plane, Point2, Point3, Surface, TrimmedCurve, TrimmedCurve2,
};
use crate::model::Model;
use crate::topology::payload::Payload;

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
    /// One section of a circular edge revolved about the circle's axis: a
    /// torus for a fillet, a cone or cylinder for a chamfer. Each side's rail
    /// is a circle about the axis, and the surface's isoline
    /// `v = levels[side]` with the surface's `u` as its parameter.
    Revolved {
        levels: [f64; 2],
        shape: SectionShape,
    },
    /// Sections solved along the edge and skinned into a NURBS surface. Each
    /// side's rail is the skin's isoline `v = side`, with the skin's `u` as
    /// its parameter.
    Swept { shape: SectionShape },
}

/// What every cross-section of a blend is, square to its edge.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum SectionShape {
    /// The arc of a ball of this radius touching both faces.
    Arc { radius: f64 },
    /// The segment between the two rails.
    Segment,
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

    /// The radius of the ball whose arc each cross-section is, for a round.
    pub(crate) fn fillet_radius(&self) -> Option<f64> {
        let shape = match self.form {
            SectionForm::Cylinder { radius, .. } => return Some(radius),
            SectionForm::Strip { .. } | SectionForm::Translated { .. } => return None,
            SectionForm::Revolved { shape, .. } | SectionForm::Swept { shape } => shape,
        };
        match shape {
            SectionShape::Arc { radius } => Some(radius),
            SectionShape::Segment => None,
        }
    }

    /// The level of the blend surface's isoline that `side`'s rail runs
    /// along, when the rail is one.
    fn isoline(&self, side: usize) -> Option<f64> {
        match self.form {
            SectionForm::Revolved { levels, .. } => Some(levels[side]),
            SectionForm::Swept { .. } => Some(side as f64),
            SectionForm::Strip { .. }
            | SectionForm::Cylinder { .. }
            | SectionForm::Translated { .. } => None,
        }
    }

    /// The pcurve of part of `side`'s rail on the blend surface, in the
    /// rail's direction.
    ///
    /// A rail that is one of the surface's isolines has it exactly, since its
    /// own parameter is the surface's `u`; any other is traced.
    pub(crate) fn blend_pcurve(
        &self,
        side: usize,
        rail: &TrimmedCurve,
    ) -> Result<TrimmedCurve2, BlendError> {
        match self.isoline(side) {
            Some(level) => {
                let interval = rail.interval();
                Ok(TrimmedCurve2::segment(
                    Point2::new(interval.start.value(), level),
                    Point2::new(interval.end.value(), level),
                ))
            }
            None => pcurve_on(&self.surface, rail),
        }
    }
}

/// Solves one selected edge's section under `law`.
///
/// Two planes meeting in a straight edge and the plane/extruded-wall chamfer
/// answer in closed form; a circular edge between surfaces of revolution
/// about its axis is revolved from one section; everything else is skinned
/// through sections solved along the edge.
pub(crate) fn solve_section<P: Payload>(
    model: &Model<P>,
    edge: &NetworkEdge,
    law: BlendLaw,
) -> Result<EdgeSection, BlendError> {
    let surfaces = edge
        .sides
        .map(|side| &model.face_attr_unchecked(side.face).surface);
    match (surfaces[0], surfaces[1], edge.span.curve(), law) {
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
        _ => {
            let crease = Crease::capture(model, edge)?;
            let convex = crease.convexity()?;
            revolved_section(&crease, law, convex)
                .unwrap_or_else(|| swept_section(&crease, law, convex))
        }
    }
}
