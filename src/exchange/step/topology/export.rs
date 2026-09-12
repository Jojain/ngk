//! Walking a solid into a `MANIFOLD_SOLID_BREP`.
//!
//! **Sharing is the whole job.** A STEP shell is not a list of independent
//! faces: the two faces meeting along an edge must name *the same*
//! `EDGE_CURVE`, and the edges meeting at a corner *the same* `VERTEX_POINT`,
//! or the file describes a heap of loose faces that no reader will sew back
//! into a solid. So each cell key is written once, cached, and referred to
//! afterwards.
//!
//! **Orientation is derived, never stored.** NGK does not record a face's
//! sense; it reads it from the boundary's winding. `same_sense` is therefore
//! computed here rather than carried, by asking the face for its own normal
//! and comparing against the support surface's — which is exactly what STEP's
//! flag means.

use std::collections::HashMap;

use crate::topology::LoopKind;
use crate::topology::edge::Edge;
use crate::topology::face::{Face, Loop};
use crate::topology::gmap::GMap;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, SolidKey, VertexKey};
use crate::topology::vertex::Vertex;

use super::super::builder::InstanceBuilder;
use super::super::convert::curves::write_curve;
use super::super::convert::placement::write_point;
use super::super::convert::surfaces::write_surface;
use super::super::error::{StepError, TopologyError};
use super::super::part21::{EntityId, Record, Value};

/// An `EDGE_CURVE` already written, and the corner it starts from.
///
/// The two travel together because neither answers the question alone: a loop
/// meeting this edge must know both which instance to name *and* whether it is
/// walking it forwards, and the start vertex is what settles the second.
#[derive(Debug, Clone, Copy)]
struct WrittenEdge {
    curve: EntityId,
    start: VertexKey,
}

/// Instances already written, keyed by the cell they came from.
#[derive(Debug, Default)]
struct ExportCache {
    vertices: HashMap<VertexKey, EntityId>,
    edges: HashMap<EdgeKey, WrittenEdge>,
}

/// Writes one solid, returning its `MANIFOLD_SOLID_BREP`.
pub fn write_solid<P: Payload>(
    builder: &mut InstanceBuilder,
    gmap: &GMap<P>,
    key: SolidKey,
) -> Result<EntityId, StepError> {
    let solid = gmap
        .solid(key)
        .ok_or(TopologyError::UnknownSolid { solid: key })?;

    if let Some(inner) = solid.inner_shells()
        && !inner.is_empty()
    {
        return Err(TopologyError::InnerShells {
            solid: key,
            count: inner.len(),
        }
        .into());
    }

    let mut cache = ExportCache::default();
    let mut faces = Vec::new();
    for face in solid.outer_shell().faces() {
        faces.push(Value::Ref(write_face(builder, gmap, &face, &mut cache)?));
    }

    let shell = builder.add(Record::new(
        "CLOSED_SHELL",
        vec![Value::Text(String::new()), Value::List(faces)],
    ));

    Ok(builder.add(Record::new(
        "MANIFOLD_SOLID_BREP",
        vec![Value::Text(String::new()), Value::Ref(shell)],
    )))
}

fn write_face<P: Payload>(
    builder: &mut InstanceBuilder,
    gmap: &GMap<P>,
    face: &Face<'_, P>,
    cache: &mut ExportCache,
) -> Result<EntityId, StepError> {
    let loops = face.loops();
    if loops.is_empty() {
        return Err(TopologyError::BoundarylessFace { face: face.key() }.into());
    }

    let same_sense = face_same_sense(face)?;
    let surface = write_surface(builder, face.surface())?;

    let mut bounds = Vec::with_capacity(loops.len());
    for loop_ in &loops {
        let kind = loop_.kind();
        if !matches!(kind, LoopKind::Outer | LoopKind::Inner) {
            // A wrapping or capping loop closes on the periodic quotient, and
            // STEP has no way to say that: it wants the domain cut open along
            // a seam. Synthesizing that cut is stage 4's work, and guessing at
            // it here would emit a loop that does not close.
            return Err(TopologyError::PeriodicLoop {
                face: face.key(),
                kind: loop_kind_name(kind),
            }
            .into());
        }

        let edge_loop = write_edge_loop(builder, gmap, loop_, cache)?;
        let keyword = if kind == LoopKind::Outer {
            "FACE_OUTER_BOUND"
        } else {
            "FACE_BOUND"
        };
        bounds.push(Value::Ref(builder.add(Record::new(
            keyword,
            vec![
                Value::Text(String::new()),
                Value::Ref(edge_loop),
                // The loop is already written in the face's own traversal
                // direction, so the bound never needs to flip it; the walk
                // direction lives entirely in each ORIENTED_EDGE.
                Value::Enum("T".to_string()),
            ],
        ))));
    }

    Ok(builder.add(Record::new(
        "ADVANCED_FACE",
        vec![
            Value::Text(String::new()),
            Value::List(bounds),
            Value::Ref(surface),
            Value::Enum(bool_enum(same_sense)),
        ],
    )))
}

/// Reports whether the face's outward normal agrees with its surface's.
///
/// This is exactly what `ADVANCED_FACE.same_sense` states, and NGK can answer
/// it without storing anything: [`Face::normal_at`] is the support normal
/// flipped whenever the boundary winds the other way, so comparing the two is
/// the same question asked twice.
fn face_same_sense<P: Payload>(face: &Face<'_, P>) -> Result<bool, TopologyError> {
    // A plane's normal is constant, so any parameter answers for it. A curved
    // support will need a parameter known to lie inside the trimmed region,
    // which arrives with stage 4.
    let (u, v) = (0.0, 0.0);
    let agreement = face.normal_at(u, v).dot(&face.surface().normal_at(u, v));
    if agreement == 0.0 || !agreement.is_finite() {
        return Err(TopologyError::UnreadableFaceSense { face: face.key() });
    }
    Ok(agreement > 0.0)
}

fn write_edge_loop<P: Payload>(
    builder: &mut InstanceBuilder,
    gmap: &GMap<P>,
    loop_: &Loop<'_, P>,
    cache: &mut ExportCache,
) -> Result<EntityId, StepError> {
    let mut oriented = Vec::new();
    // One dart per edge, in the loop's own traversal order — which is already
    // the face view's orientation, since the loop was read through it.
    for dart in loop_.darts().step_by(2) {
        let edge = Edge::from_dart(gmap, dart).ok_or(TopologyError::UnregisteredEdge { dart })?;
        let key = edge.key();
        let written = match cache.edges.get(&key) {
            Some(written) => *written,
            None => {
                let written = write_edge_curve(builder, gmap, key, cache)?;
                cache.edges.insert(key, written);
                written
            }
        };

        // The loop walks this edge forwards exactly when it leaves the same
        // corner the EDGE_CURVE was written from.
        let bounded = edge
            .bounded()
            .ok_or(TopologyError::ClosedEdge { edge: key })?;
        let forward = bounded.start().key() == written.start;

        oriented.push(Value::Ref(builder.add(Record::new(
            "ORIENTED_EDGE",
            vec![
                Value::Text(String::new()),
                Value::Derived,
                Value::Derived,
                Value::Ref(written.curve),
                Value::Enum(bool_enum(forward)),
            ],
        ))));
    }

    Ok(builder.add(Record::new(
        "EDGE_LOOP",
        vec![Value::Text(String::new()), Value::List(oriented)],
    )))
}

/// Writes the `EDGE_CURVE` for an edge, in the edge's own default direction.
///
/// Writing it from the *default* orientation rather than from whichever face
/// reached it first is what makes the instance shareable: both faces then
/// describe the same directed curve and disagree only in their own
/// `ORIENTED_EDGE.orientation`.
fn write_edge_curve<P: Payload>(
    builder: &mut InstanceBuilder,
    gmap: &GMap<P>,
    key: EdgeKey,
    cache: &mut ExportCache,
) -> Result<WrittenEdge, StepError> {
    let bounded = Edge::new(gmap, key)
        .bounded()
        .ok_or(TopologyError::ClosedEdge { edge: key })?;
    let (start, end) = bounded.vertices();

    let start_id = write_vertex(builder, &start, cache)?;
    let end_id = write_vertex(builder, &end, cache)?;

    let geometry = bounded
        .curve()
        .ok_or(TopologyError::MissingCurve { edge: key })?;
    let curve = write_curve(builder, geometry)?;

    // An edge stores no interval, so which way the support runs between the
    // two vertices is derived: the span they bound increases exactly when the
    // curve agrees with start → end.
    let interval = bounded
        .parameter_interval()
        .ok_or(TopologyError::MissingCurve { edge: key })?;
    let same_sense = interval.start <= interval.end;

    let edge_curve = builder.add(Record::new(
        "EDGE_CURVE",
        vec![
            Value::Text(String::new()),
            Value::Ref(start_id),
            Value::Ref(end_id),
            Value::Ref(curve),
            Value::Enum(bool_enum(same_sense)),
        ],
    ));

    Ok(WrittenEdge {
        curve: edge_curve,
        start: start.key(),
    })
}

/// Writes a `VERTEX_POINT`, shared by key.
///
/// Shared by *key*, never by position: two corners of a solid that happen to
/// coincide are still two corners, and merging them would weld the shape.
fn write_vertex<P: Payload>(
    builder: &mut InstanceBuilder,
    vertex: &Vertex<'_, P>,
    cache: &mut ExportCache,
) -> Result<EntityId, TopologyError> {
    let key = vertex.key();
    if let Some(&existing) = cache.vertices.get(&key) {
        return Ok(existing);
    }

    let position = vertex
        .point()
        .ok_or(TopologyError::MissingVertexPoint { vertex: key })?;
    let position = write_point(builder, *position);
    let id = builder.add(Record::new(
        "VERTEX_POINT",
        vec![Value::Text(String::new()), Value::Ref(position)],
    ));
    cache.vertices.insert(key, id);
    Ok(id)
}

fn bool_enum(value: bool) -> String {
    if value { "T" } else { "F" }.to_string()
}

fn loop_kind_name(kind: LoopKind) -> &'static str {
    match kind {
        LoopKind::Outer => "Outer",
        LoopKind::Inner => "Inner",
        LoopKind::Wrapping { .. } => "Wrapping",
        LoopKind::Capping { .. } => "Capping",
    }
}
