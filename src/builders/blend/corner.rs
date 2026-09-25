//! Blends at the corners of wires and free planar faces.
//!
//! A planar corner is where two edges of one wire or one free face meet. Its
//! blend is one new edge from a trim point on the incoming edge to one on the
//! outgoing edge: a chamfer's is the segment between points a distance along
//! each edge, a fillet's the arc of the circle tangent to both. The corner
//! vertex is consumed, the two trim points take its place, and the surgery
//! inserts the new edge into the corner.
//!
//! A fillet's circle is found the same way for every pair of supports the
//! table knows: its centre lies on an offset of each, so the candidates are
//! where those offsets cross, and the one kept is the circle that continues
//! both edges tangentially in the direction they are travelled, nearest the
//! corner. Lines and circles offset in closed form, which is what bounds the
//! table today.

use nalgebra::{Unit, Vector2, Vector3};

use super::errors::BlendError;
use super::law::{BlendLaw, ChamferLaw, FilletLaw};
use super::pcurve::pcurve_on;
use super::surgery::{Insertion, Joint, Surgery};
use super::target::CornerRef;
use crate::geometry::parameter::Fraction;
use crate::geometry::{Circle, Curve, Interval, LINEAR_TOLERANCE, Plane, Point3, TrimmedCurve};
use crate::model::Model;
use crate::topology::edge::Edge;
use crate::topology::edit::EditKey;
use crate::topology::gmap::Dim;
use crate::topology::orientation::Orientation;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, VertexKey};

/// Plans a blend at every corner and adds it to `surgery`.
pub(crate) fn plan_corners<P: Payload>(
    model: &Model<P>,
    corners: &[CornerRef],
    law: BlendLaw,
    surgery: &mut Surgery,
) -> Result<(), BlendError> {
    let plans = corners
        .iter()
        .map(|corner| plan_corner(model, corner, law))
        .collect::<Result<Vec<_>, _>>()?;
    check_trims_do_not_cross(&plans)?;
    for plan in plans {
        let sources = vec![EditKey::Vertex(plan.corner.vertex)];
        let first = surgery.corner(plan.trims[0].point, sources.clone());
        let second = surgery.corner(plan.trims[1].point, sources.clone());
        let pcurve = match plan.corner.face {
            Some(face) => Some(pcurve_on(
                &model.face_attr_unchecked(face).surface,
                &plan.curve,
            )?),
            None => None,
        };
        surgery.joint(Joint {
            corners: [first, second],
            curve: plan.curve,
            sources,
            insertion: Some(Insertion {
                face: plan.corner.face,
                after: plan.corner.after,
                pcurve,
            }),
        });
        surgery.consumed_vertices.push(plan.corner.vertex);
    }
    Ok(())
}

/// One corner's blend: where each edge is trimmed and what joins the trims.
struct CornerPlan {
    corner: CornerRef,
    /// On the incoming edge, then on the outgoing one.
    trims: [Trim; 2],
    /// From the incoming trim to the outgoing one.
    curve: TrimmedCurve,
}

/// Where a corner blend cuts one of its edges back to.
struct Trim {
    edge: EdgeKey,
    point: Point3,
    /// The trim's fraction along the edge in its stored orientation.
    fraction: f64,
    /// Whether the corner is at the stored end of the edge rather than its start.
    at_end: bool,
}

/// The two edges of one corner, each travelled the way the corner's walk goes.
struct CornerEdges {
    vertex: VertexKey,
    /// Arrives at the corner at its end.
    incoming: TrimmedCurve,
    incoming_key: EdgeKey,
    incoming_reversed: bool,
    /// Leaves the corner at its start.
    outgoing: TrimmedCurve,
    outgoing_key: EdgeKey,
    outgoing_reversed: bool,
}

fn plan_corner<P: Payload>(
    model: &Model<P>,
    corner: &CornerRef,
    law: BlendLaw,
) -> Result<CornerPlan, BlendError> {
    let vertex = corner.vertex;
    let incoming_dart = model.alpha(Dim::Zero, corner.after);
    let (Some(incoming), Some(outgoing)) = (
        Edge::from_dart(model, incoming_dart),
        Edge::from_dart(model, corner.next),
    ) else {
        return Err(BlendError::UnsupportedCorner {
            vertex,
            reason: "one of its sides is not an edge",
        });
    };
    let edges = CornerEdges {
        vertex,
        incoming: incoming.trimmed_curve(),
        incoming_key: incoming.key(),
        incoming_reversed: model.edge_orientation_at_dart(incoming.key(), incoming_dart)
            == Orientation::Reversed,
        outgoing: outgoing.trimmed_curve(),
        outgoing_key: outgoing.key(),
        outgoing_reversed: model.edge_orientation_at_dart(outgoing.key(), corner.next)
            == Orientation::Reversed,
    };
    let plane = corner_plane(model, corner, &edges)?;
    let (trims, curve) = match law {
        BlendLaw::Chamfer(ChamferLaw::Distance(distance)) => chamfer_corner(&edges, distance)?,
        BlendLaw::Fillet(FilletLaw::Radius(radius)) => fillet_corner(&edges, &plane, radius)?,
    };
    Ok(CornerPlan {
        corner: *corner,
        trims: [edges.trim(trims[0], true), edges.trim(trims[1], false)],
        curve,
    })
}

impl CornerEdges {
    /// Describes a trim point on the incoming (`incoming = true`) or outgoing edge.
    fn trim(&self, point: Point3, incoming: bool) -> Trim {
        let (span, key, reversed) = if incoming {
            (&self.incoming, self.incoming_key, self.incoming_reversed)
        } else {
            (&self.outgoing, self.outgoing_key, self.outgoing_reversed)
        };
        let along = span.parameter_at(point).value();
        // The corner sits at the incoming span's end and the outgoing span's
        // start; a reversed view puts it at the other end of the stored edge.
        let at_end = incoming != reversed;
        Trim {
            edge: key,
            point,
            fraction: if reversed { 1.0 - along } else { along },
            at_end,
        }
    }

    fn unsupported(&self, reason: &'static str) -> BlendError {
        BlendError::UnsupportedCorner {
            vertex: self.vertex,
            reason,
        }
    }

    fn does_not_fit(&self, reason: &'static str) -> BlendError {
        BlendError::VertexDoesNotFit {
            vertex: self.vertex,
            reason,
        }
    }
}

/// The plane a corner lies in, framed at the corner with its x axis along
/// the outgoing edge.
struct CornerFrame {
    origin: Point3,
    x: Vector3<f64>,
    y: Vector3<f64>,
    normal: Vector3<f64>,
}

impl CornerFrame {
    fn to_2d(&self, point: Point3) -> Vector2<f64> {
        let offset = point - self.origin;
        Vector2::new(offset.dot(&self.x), offset.dot(&self.y))
    }

    fn direction_2d(&self, direction: Vector3<f64>) -> Vector2<f64> {
        Vector2::new(direction.dot(&self.x), direction.dot(&self.y))
    }

    fn to_3d(&self, point: Vector2<f64>) -> Point3 {
        self.origin + self.x * point.x + self.y * point.y
    }
}

fn corner_plane<P: Payload>(
    model: &Model<P>,
    corner: &CornerRef,
    edges: &CornerEdges,
) -> Result<CornerFrame, BlendError> {
    let origin = edges.outgoing.start();
    let leaving = edges.outgoing.derivative_at(Fraction::START, 1);
    let arriving = edges.incoming.derivative_at(Fraction::END, 1);
    let normal = match corner.face {
        Some(face) => match &model.face_attr_unchecked(face).surface {
            crate::geometry::Surface::Plane(plane) => *plane.normal(),
            _ => return Err(edges.unsupported("its face is not planar")),
        },
        None => {
            let circle_normal = [edges.incoming.curve(), edges.outgoing.curve()]
                .into_iter()
                .find_map(|curve| match curve {
                    Curve::Circle(circle) => Some(*circle.plane().normal()),
                    _ => None,
                });
            match circle_normal {
                Some(normal) => normal,
                None => {
                    let normal = arriving.cross(&leaving);
                    if normal.norm() <= LINEAR_TOLERANCE * arriving.norm() * leaving.norm() {
                        return Err(edges.unsupported("its edges double back on each other"));
                    }
                    normal.normalize()
                }
            }
        }
    };
    let x = leaving - normal * leaving.dot(&normal);
    if x.norm() <= LINEAR_TOLERANCE {
        return Err(edges.unsupported("its outgoing edge leaves the corner's plane"));
    }
    let x = x.normalize();
    let frame = CornerFrame {
        origin,
        x,
        y: normal.cross(&x),
        normal,
    };
    for span in [&edges.incoming, &edges.outgoing] {
        let off_plane = [Fraction::START, Fraction::new(0.5), Fraction::END]
            .into_iter()
            .map(|fraction| (span.point_at(fraction) - origin).dot(&normal).abs())
            .fold(0.0_f64, f64::max);
        if off_plane > LINEAR_TOLERANCE.sqrt() {
            return Err(edges.unsupported("its edges do not lie in one plane"));
        }
    }
    Ok(frame)
}

/// Trims both edges a distance along them from the corner and joins the trims
/// with a segment.
fn chamfer_corner(
    edges: &CornerEdges,
    distance: f64,
) -> Result<([Point3; 2], TrimmedCurve), BlendError> {
    let mut trims = [Point3::origin(); 2];
    for (slot, (span, from_end)) in trims
        .iter_mut()
        .zip([(&edges.incoming, true), (&edges.outgoing, false)])
    {
        if !matches!(span.curve(), Curve::Line(_) | Curve::Circle(_)) {
            return Err(edges.unsupported("its edges are neither lines nor arcs"));
        }
        let length = span.length();
        if distance >= length - LINEAR_TOLERANCE {
            return Err(edges.does_not_fit("the distance reaches past the end of an edge"));
        }
        // Lines and arcs are parameterized proportionally to length.
        let fraction = if from_end {
            1.0 - distance / length
        } else {
            distance / length
        };
        *slot = span.point_at(Fraction::new(fraction));
    }
    Ok((trims, TrimmedCurve::segment(trims[0], trims[1])))
}

/// A support offset-able in closed form, in the corner's plane.
#[derive(Clone, Copy)]
enum Support {
    Line {
        point: Vector2<f64>,
        direction: Vector2<f64>,
    },
    Circle {
        center: Vector2<f64>,
        radius: f64,
    },
}

impl Support {
    fn from_span(frame: &CornerFrame, span: &TrimmedCurve) -> Option<Self> {
        match span.curve() {
            Curve::Line(line) => Some(Self::Line {
                point: frame.to_2d(line.origin()),
                direction: frame.direction_2d(*line.direction()).normalize(),
            }),
            Curve::Circle(circle) => Some(Self::Circle {
                center: frame.to_2d(circle.plane().origin()),
                radius: circle.radius(),
            }),
            _ => None,
        }
    }

    /// The supports every point a distance `offset` from this one lies on.
    fn offsets(self, offset: f64) -> Vec<Self> {
        match self {
            Self::Line { point, direction } => {
                let normal = Vector2::new(-direction.y, direction.x);
                vec![
                    Self::Line {
                        point: point + normal * offset,
                        direction,
                    },
                    Self::Line {
                        point: point - normal * offset,
                        direction,
                    },
                ]
            }
            Self::Circle { center, radius } => [radius + offset, (radius - offset).abs()]
                .into_iter()
                .filter(|radius| *radius > LINEAR_TOLERANCE)
                .map(|radius| Self::Circle { center, radius })
                .collect(),
        }
    }

    /// The point of this support nearest `point`.
    fn foot(self, point: Vector2<f64>) -> Option<Vector2<f64>> {
        match self {
            Self::Line {
                point: origin,
                direction,
            } => Some(origin + direction * (point - origin).dot(&direction)),
            Self::Circle { center, radius } => {
                let offset = point - center;
                (offset.norm() > LINEAR_TOLERANCE).then(|| center + offset.normalize() * radius)
            }
        }
    }
}

/// Where two supports cross.
fn crossings(first: Support, second: Support) -> Vec<Vector2<f64>> {
    match (first, second) {
        (
            Support::Line {
                point: p,
                direction: d,
            },
            Support::Line {
                point: q,
                direction: e,
            },
        ) => {
            let denominator = cross(d, e);
            if denominator.abs() <= LINEAR_TOLERANCE {
                return Vec::new();
            }
            vec![p + d * (cross(q - p, e) / denominator)]
        }
        (line @ Support::Line { .. }, circle @ Support::Circle { .. })
        | (circle @ Support::Circle { .. }, line @ Support::Line { .. }) => {
            let (Support::Line { point, direction }, Support::Circle { center, radius }) =
                (line, circle)
            else {
                unreachable!("the pattern pairs one line with one circle");
            };
            let foot = point + direction * (center - point).dot(&direction);
            let distance = (foot - center).norm();
            if distance > radius + LINEAR_TOLERANCE {
                return Vec::new();
            }
            let half = (radius * radius - distance * distance).max(0.0).sqrt();
            vec![foot - direction * half, foot + direction * half]
        }
        (
            Support::Circle {
                center: a,
                radius: r,
            },
            Support::Circle {
                center: b,
                radius: s,
            },
        ) => {
            let between = b - a;
            let distance = between.norm();
            if distance <= LINEAR_TOLERANCE
                || distance > r + s + LINEAR_TOLERANCE
                || distance < (r - s).abs() - LINEAR_TOLERANCE
            {
                return Vec::new();
            }
            let along = (r * r - s * s + distance * distance) / (2.0 * distance);
            let half = (r * r - along * along).max(0.0).sqrt();
            let unit = between / distance;
            let base = a + unit * along;
            let normal = Vector2::new(-unit.y, unit.x);
            vec![base + normal * half, base - normal * half]
        }
    }
}

fn cross(a: Vector2<f64>, b: Vector2<f64>) -> f64 {
    a.x * b.y - a.y * b.x
}

/// Finds the circle of `radius` tangent to both edges and continuing both the
/// way they are travelled, nearest the corner.
fn fillet_corner(
    edges: &CornerEdges,
    frame: &CornerFrame,
    radius: f64,
) -> Result<([Point3; 2], TrimmedCurve), BlendError> {
    let incoming = Support::from_span(frame, &edges.incoming)
        .ok_or_else(|| edges.unsupported("its edges are neither lines nor arcs"))?;
    let outgoing = Support::from_span(frame, &edges.outgoing)
        .ok_or_else(|| edges.unsupported("its edges are neither lines nor arcs"))?;
    let tolerance = LINEAR_TOLERANCE.sqrt();

    let mut best: Option<(f64, [Point3; 2], TrimmedCurve)> = None;
    for first in incoming.offsets(radius) {
        for second in outgoing.offsets(radius) {
            for center in crossings(first, second) {
                let (Some(entry), Some(exit)) = (incoming.foot(center), outgoing.foot(center))
                else {
                    continue;
                };
                if ((entry - center).norm() - radius).abs() > tolerance
                    || ((exit - center).norm() - radius).abs() > tolerance
                {
                    continue;
                }
                let entry_point = frame.to_3d(entry);
                let exit_point = frame.to_3d(exit);
                let (Some(entry_along), Some(exit_along)) = (
                    along_span(&edges.incoming, entry_point),
                    along_span(&edges.outgoing, exit_point),
                ) else {
                    continue;
                };
                // The arc has to carry on where the incoming edge was going and
                // leave the way the outgoing edge goes: that is what picks the
                // one circle in the corner among those tangent to both supports.
                let entry_tangent = travel_tangent(frame, &edges.incoming, entry_along);
                let exit_tangent = travel_tangent(frame, &edges.outgoing, exit_along);
                let turn = cross(entry - center, entry_tangent).signum();
                let arc_tangent = |at: Vector2<f64>| {
                    let radial = (at - center) / radius;
                    Vector2::new(-radial.y, radial.x) * turn
                };
                if arc_tangent(entry).dot(&entry_tangent) < 1.0 - tolerance
                    || arc_tangent(exit).dot(&exit_tangent) < 1.0 - tolerance
                {
                    continue;
                }
                let start = entry - center;
                let end = exit - center;
                let sweep = (turn * cross(start, end).atan2(start.dot(&end)))
                    .rem_euclid(std::f64::consts::TAU);
                if sweep <= tolerance || sweep >= std::f64::consts::PI {
                    continue;
                }
                if entry_along >= 1.0 - tolerance || exit_along <= tolerance {
                    continue;
                }
                let setback = (1.0 - entry_along) * edges.incoming.length()
                    + exit_along * edges.outgoing.length();
                if best.as_ref().is_some_and(|(best, ..)| *best <= setback) {
                    continue;
                }
                let center_point = frame.to_3d(center);
                let arc = Circle::new(
                    Plane::new(
                        center_point,
                        Unit::new_normalize(entry_point - center_point),
                        Unit::new_normalize(frame.normal * turn),
                    ),
                    radius,
                );
                best = Some((
                    setback,
                    [entry_point, exit_point],
                    TrimmedCurve::new(Curve::Circle(arc), Interval::new(0.0, sweep)),
                ));
            }
        }
    }
    let (_, trims, curve) =
        best.ok_or_else(|| edges.does_not_fit("no circle of this radius fits in the corner"))?;
    Ok((trims, curve))
}

/// The fraction of `point` along `span`, or `None` when it is off the span.
fn along_span(span: &TrimmedCurve, point: Point3) -> Option<f64> {
    span.contains(point, LINEAR_TOLERANCE.sqrt())
        .then(|| span.parameter_at(point).value())
}

/// The unit direction a span is travelled in at a fraction, in the corner's plane.
fn travel_tangent(frame: &CornerFrame, span: &TrimmedCurve, fraction: f64) -> Vector2<f64> {
    frame
        .direction_2d(span.derivative_at(Fraction::new(fraction), 1))
        .normalize()
}

/// Refuses corners whose trims meet or pass each other on a shared edge.
fn check_trims_do_not_cross(plans: &[CornerPlan]) -> Result<(), BlendError> {
    let mut by_edge = std::collections::HashMap::<EdgeKey, Vec<(&Trim, VertexKey)>>::new();
    for plan in plans {
        for trim in &plan.trims {
            by_edge
                .entry(trim.edge)
                .or_default()
                .push((trim, plan.corner.vertex));
        }
    }
    for (edge, trims) in by_edge {
        let start = trims.iter().find(|(trim, _)| !trim.at_end);
        let end = trims.iter().find(|(trim, _)| trim.at_end);
        if let (Some((start, _)), Some((end, _))) = (start, end)
            && start.fraction >= end.fraction - LINEAR_TOLERANCE
        {
            return Err(BlendError::EdgeDoesNotFit {
                edge,
                reason: "the blends at its two ends overlap",
            });
        }
    }
    Ok(())
}
