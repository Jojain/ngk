//! Blends of solid edges and corners: capture, section, treat, assemble.
//!
//! The network is captured once, every selected edge gets its section and
//! every vertex its treatment, all against the model as the call found it.
//! Assembly then gives every treatment's local corners and joints their ids
//! in the one [`Surgery`], cuts each selected edge along the rails its two end
//! treatments stop at, and closes each blend face with the chains those
//! treatments wrote.

use std::collections::HashMap;

use super::errors::BlendError;
use super::law::BlendLaw;
use super::network::{BlendNetwork, capture};
use super::pcurve::pcurve_on;
use super::section::{EdgeSection, solve_section};
use super::surgery::{Bound, BoundKind, Cut, CutSide, Insertion, Joint, NewFace, Surgery};
use super::target::Resolution;
use super::vertex::{VertexBlend, treat_vertex};
use crate::geometry::parameter::Fraction;
use crate::geometry::{LINEAR_TOLERANCE, Surface, TrimmedCurve};
use crate::model::Model;
use crate::topology::edit::EditKey;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::EdgeKey;

/// Plans the solid part of a resolution and adds it to `surgery`.
pub(crate) fn plan_solid<P: Payload>(
    model: &Model<P>,
    resolution: &Resolution,
    law: BlendLaw,
    surgery: &mut Surgery,
) -> Result<(), BlendError> {
    if resolution.edges.is_empty() && resolution.corner_cuts.is_empty() {
        return Ok(());
    }
    let network = capture(model, &resolution.edges, &resolution.corner_cuts)?;
    let sections = network
        .edges
        .iter()
        .map(|edge| solve_section(model, edge, law))
        .collect::<Result<Vec<_>, _>>()?;
    let blends = (0..network.vertices.len())
        .map(|index| treat_vertex(model, &network, index, &sections, law))
        .collect::<Result<Vec<_>, _>>()?;
    check_landings(model, &network, &blends)?;
    assemble(model, &network, &sections, &blends, surgery)
}

/// Gives every treatment's pieces their surgery ids, then cuts and closes
/// every selected edge.
fn assemble<P: Payload>(
    model: &Model<P>,
    network: &BlendNetwork,
    sections: &[EdgeSection],
    blends: &[VertexBlend],
    surgery: &mut Surgery,
) -> Result<(), BlendError> {
    let mut corner_ids = Vec::with_capacity(blends.len());
    let mut joint_ids = Vec::with_capacity(blends.len());
    for (vertex, blend) in network.vertices.iter().zip(blends) {
        let sources = vec![EditKey::Vertex(vertex.key)];
        let corners = blend
            .corners
            .iter()
            .map(|&point| surgery.corner(point, sources.clone()))
            .collect::<Vec<_>>();
        let mut joints = Vec::with_capacity(blend.joints.len());
        for joint in &blend.joints {
            let insertion = match joint.insertion {
                Some((face, after)) => Some(Insertion {
                    face: Some(face),
                    after,
                    pcurve: Some(pcurve_on(
                        &model.face_attr_unchecked(face).surface,
                        &joint.curve,
                    )?),
                }),
                None => None,
            };
            joints.push(surgery.joint(Joint {
                corners: joint.corners.map(|corner| corners[corner]),
                curve: joint.curve.clone(),
                sources: sources.clone(),
                insertion,
            }));
        }
        surgery.consumed_vertices.push(vertex.key);
        corner_ids.push(corners);
        joint_ids.push(joints);
    }

    let mut ends = vec![[None, None]; network.edges.len()];
    for (vertex, blend) in blends.iter().enumerate() {
        for end in &blend.ends {
            ends[end.edge][end.end] = Some((vertex, end));
        }
    }

    for (index, (edge, section)) in network.edges.iter().zip(sections).enumerate() {
        let [
            Some((first_vertex, first_end)),
            Some((last_vertex, last_end)),
        ] = ends[index]
        else {
            return Err(BlendError::InconsistentSurgery {
                reason: "a selected edge has an end no treatment closed",
            });
        };
        let cut = surgery.cuts.len();
        let sides = std::array::from_fn::<_, 2, _>(|side| side);
        let mut cut_sides = Vec::with_capacity(2);
        let mut rails = Vec::with_capacity(2);
        for side in sides {
            let corners = [
                corner_ids[first_vertex][first_end.rails[side]],
                corner_ids[last_vertex][last_end.rails[side]],
            ];
            let rail = section.rail_span(
                side,
                surgery.corners[corners[0]].point,
                surgery.corners[corners[1]].point,
                &edge.span,
            );
            check_rail(edge.key, &edge.span, &rail)?;
            let face = edge.sides[side].face;
            cut_sides.push(CutSide {
                face,
                start: edge.sides[side].start,
                corners,
                rail: rail.clone(),
                pcurve: pcurve_on(&model.face_attr_unchecked(face).surface, &rail)?,
            });
            rails.push(rail);
        }
        let [first_side, second_side] = [cut_sides[0].clone(), cut_sides[1].clone()];
        surgery.cuts.push(Cut {
            edge: edge.key,
            sides: [first_side, second_side],
        });

        // Along side 0 from the edge's start, round its end, back along side
        // 1, and round its start.
        let mut boundary = vec![bound(
            &section.surface,
            BoundKind::Rail { cut, side: 0 },
            &rails[0],
            false,
        )?];
        for &(joint, reversed) in &last_end.chain {
            let id = joint_ids[last_vertex][joint];
            boundary.push(bound(
                &section.surface,
                BoundKind::Joint(id),
                &surgery.joints[id].curve,
                reversed,
            )?);
        }
        boundary.push(bound(
            &section.surface,
            BoundKind::Rail { cut, side: 1 },
            &rails[1],
            true,
        )?);
        for &(joint, reversed) in first_end.chain.iter().rev() {
            let id = joint_ids[first_vertex][joint];
            boundary.push(bound(
                &section.surface,
                BoundKind::Joint(id),
                &surgery.joints[id].curve,
                !reversed,
            )?);
        }
        surgery.faces.push(NewFace {
            surface: section.surface.clone(),
            sources: vec![EditKey::Edge(edge.key)],
            boundary,
        });
    }

    for ((vertex, blend), joints) in network.vertices.iter().zip(blends).zip(&joint_ids) {
        for patch in &blend.patches {
            let boundary = patch
                .boundary
                .iter()
                .map(|&(joint, reversed)| {
                    let id = joints[joint];
                    bound(
                        &patch.surface,
                        BoundKind::Joint(id),
                        &surgery.joints[id].curve,
                        reversed,
                    )
                })
                .collect::<Result<Vec<_>, _>>()?;
            surgery.faces.push(NewFace {
                surface: patch.surface.clone(),
                sources: vec![EditKey::Vertex(vertex.key)],
                boundary,
            });
        }
    }
    Ok(())
}

/// One boundary edge of a new face, with its pcurve in the walk's direction.
fn bound(
    surface: &Surface,
    kind: BoundKind,
    curve: &TrimmedCurve,
    reversed: bool,
) -> Result<Bound, BlendError> {
    let pcurve = pcurve_on(surface, curve)?;
    Ok(Bound {
        kind,
        reversed,
        pcurve: if reversed { pcurve.reversed() } else { pcurve },
    })
}

/// Refuses a rail that has shrunk to nothing or runs against its edge: the
/// treatments at its two ends have cut it back past each other.
fn check_rail(edge: EdgeKey, span: &TrimmedCurve, rail: &TrimmedCurve) -> Result<(), BlendError> {
    let along = rail
        .derivative_at(Fraction::new(0.5), 1)
        .dot(&span.derivative_at(Fraction::new(0.5), 1));
    let chord = rail.end() - rail.start();
    let edge_chord = span.end() - span.start();
    if rail.length() <= LINEAR_TOLERANCE.sqrt() || along <= 0.0 || chord.dot(&edge_chord) <= 0.0 {
        return Err(BlendError::EdgeDoesNotFit {
            edge,
            reason: "the blends at its two ends overlap",
        });
    }
    Ok(())
}

/// Refuses an unselected edge cut back from both ends past itself.
fn check_landings<P: Payload>(
    model: &Model<P>,
    network: &BlendNetwork,
    blends: &[VertexBlend],
) -> Result<(), BlendError> {
    let mut landings =
        HashMap::<EdgeKey, Vec<(crate::topology::shape_keys::VertexKey, f64)>>::new();
    for (vertex, blend) in network.vertices.iter().zip(blends) {
        for &(edge, point) in &blend.landings {
            let span = model.edge_unchecked(edge).trimmed_curve();
            landings
                .entry(edge)
                .or_default()
                .push((vertex.key, span.parameter_at(point).value()));
        }
    }
    for (edge, cuts) in landings {
        let [first, second] = cuts.as_slice() else {
            continue;
        };
        let bounded = model
            .edge_unchecked(edge)
            .bounded()
            .ok_or(BlendError::UnsupportedEdge {
                edge,
                reason: "it is closed",
            })?;
        let start = bounded.start().key();
        let (from_start, from_end) = if first.0 == start {
            (first.1, second.1)
        } else {
            (second.1, first.1)
        };
        if from_start >= from_end - LINEAR_TOLERANCE {
            return Err(BlendError::EdgeDoesNotFit {
                edge,
                reason: "the blends at its two ends overlap",
            });
        }
    }
    Ok(())
}
