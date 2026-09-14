//! Walking a solid into a `MANIFOLD_SOLID_BREP`, or a `BREP_WITH_VOIDS`.
//!
//! **Every shell bounds the material from outside it.** A solid's outer shell
//! faces away from the material and so does each void's, which is what lets
//! both be written the same way: one `CLOSED_SHELL` per shell, the voids named
//! in their own direction rather than flipped.
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
//!
//! **The seam is synthesized, never stored either.** A face whose
//! parameterization closes on itself — a cylinder wall, a revolved cap — has
//! no seam edge in the map, because a seam belongs to a reading of the face
//! rather than to the shape. [`SeamedFace`] derives one on the way out, and
//! this module writes what it hands over without learning which kind of face
//! it came from.

use std::collections::HashMap;

use crate::geometry::{Axis2, LINEAR_TOLERANCE, Point2, Point3, SurfacePeriodicity};
use crate::model::Model;
use crate::topology::edge::Edge;
use crate::topology::face::Face;
use crate::topology::gmap::Dart;
use crate::topology::orientation::Orientation;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, SolidKey, VertexKey};
use crate::topology::sheet::ShellRef;

use super::super::builder::InstanceBuilder;
use super::super::convert::curves::write_curve;
use super::super::convert::iso_curve::iso_curve;
use super::super::convert::placement::write_point;
use super::super::convert::surfaces::write_surface;
use super::super::error::{StepError, TopologyError};
use super::super::part21::EntityId;
use super::super::schema::entities;
use super::seam::{SeamedBound, SeamedEdge, SeamedFace};

/// Instances already written, keyed by the cell they came from.
#[derive(Debug, Default)]
struct ExportCache {
    vertices: HashMap<VertexKey, EntityId>,
    closures: HashMap<EdgeKey, EntityId>,
    edges: HashMap<EdgeKey, EntityId>,
}

/// Which cell the `VERTEX_POINT` at an end of an edge stands for.
///
/// Most edges end at logical vertices, and those are what the file shares: two
/// edges meeting at a corner must name the same instance. An edge that closes
/// on itself may pass through no vertex at all -- a whole circle's closure
/// point is interior to the edge, not a corner anything meets at -- and
/// `EDGE_CURVE` has no spelling without two ends, so the point the curve closes
/// at is written as a vertex of the file alone. It is shared by the edge that
/// closes there, which is the only thing that can arrive at it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Corner {
    Vertex(VertexKey),
    Closure(EdgeKey),
}

/// A stretch of cut already written, and which way it runs.
///
/// The two travel together because neither answers the question alone: the
/// second walk to reach a cut must know both which instance to name *and*
/// whether it is running the same way. The direction is read off the parameter
/// line rather than off the corners, because a cut can leave and arrive at the
/// same corner — a torus's two cuts meet at one point of the surface, so its
/// corners say nothing about direction at all.
#[derive(Debug, Clone, Copy)]
struct WrittenSeam {
    curve: EntityId,
    increasing: bool,
}

/// A synthesized seam, named by what it runs between rather than by a cell.
///
/// A seam has no cell to be keyed on: it exists only in this reading of the
/// face. What identifies it is the pair of corners it joins and the parameter
/// direction it runs along — the same cut reached from either side of the
/// domain gives the same key, which is what makes a wall's two stretches of
/// cut one `EDGE_CURVE` used twice rather than two edges that never meet.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct SeamKey {
    corners: (EntityId, EntityId),
    along: Axis2,
}

impl SeamKey {
    fn new(from: EntityId, to: EntityId, along: Axis2) -> Self {
        let corners = if from <= to { (from, to) } else { (to, from) };
        Self { corners, along }
    }
}

/// What one face's synthesized cut has already written.
///
/// Both halves are confined to a single face because the cut is: another
/// face's cut is a different edge of the shell even where the two would land
/// on the same places.
#[derive(Debug)]
struct FaceCut {
    /// The support's periods, which say when two cut corners are one point.
    periods: [Option<f64>; 2],
    /// Corners written so far, each with the parameter point it stands at.
    corners: Vec<(Point2, EntityId)>,
    /// Stretches of cut written so far.
    seams: HashMap<SeamKey, WrittenSeam>,
}

impl FaceCut {
    fn of_face<P: Payload>(face: &Face<'_, P>) -> Self {
        let periods = match face.surface().periodicity() {
            SurfacePeriodicity::None => [None, None],
            SurfacePeriodicity::UPeriodic(u) => [Some(u), None],
            SurfacePeriodicity::VPeriodic(v) => [None, Some(v)],
            SurfacePeriodicity::UVPeriodic(u, v) => [Some(u), Some(v)],
        };
        Self {
            periods,
            corners: Vec::new(),
            seams: HashMap::new(),
        }
    }

    /// The `VERTEX_POINT` at a corner of the cut, shared by where it lands.
    ///
    /// A cut corner is not a cell, so there is no key to share it by — and it
    /// has to be shared, or a torus's four rectangle corners become four
    /// vertices where the shape has one and no reader sews the shell back up.
    /// What identifies it is its parameter point modulo the support's periods,
    /// which is exactly when two corners of the rectangle are one point of the
    /// surface. That is a statement about this cut, not about the model: real
    /// vertices are still shared by key and never by position.
    fn corner<P: Payload>(
        &mut self,
        builder: &mut InstanceBuilder,
        face: &Face<'_, P>,
        at: Point2,
    ) -> EntityId {
        if let Some(&(_, written)) = self
            .corners
            .iter()
            .find(|(existing, _)| self.same_corner(*existing, at))
        {
            return written;
        }
        let written = write_vertex_point(builder, face.surface().point_at(at.x, at.y));
        self.corners.push((at, written));
        written
    }

    /// Whether two parameter points are the same point of the support.
    fn same_corner(&self, first: Point2, second: Point2) -> bool {
        (0..2).all(|axis| {
            let gap = (second[axis] - first[axis]).abs();
            match self.periods[axis] {
                Some(period) => {
                    let folded = gap % period;
                    folded.min(period - folded) <= LINEAR_TOLERANCE
                }
                None => gap <= LINEAR_TOLERANCE,
            }
        })
    }
}

/// Writes one solid, returning its `MANIFOLD_SOLID_BREP`.
pub fn write_solid<P: Payload>(
    builder: &mut InstanceBuilder,
    gmap: &Model<P>,
    key: SolidKey,
) -> Result<EntityId, StepError> {
    let solid = gmap
        .solid(key)
        .ok_or(TopologyError::UnknownSolid { solid: key })?;

    // One cache for the whole solid, not one per shell: a void's shell is
    // disjoint from the outer one, but nothing says a later solid-wide edit
    // keeps it that way, and sharing by key is correct either way.
    let mut cache = ExportCache::default();
    let outer = write_shell(builder, gmap, &solid.outer_shell(), &mut cache)?;

    let voids = solid.inner_shells().unwrap_or_default();
    if voids.is_empty() {
        return Ok(builder.add_entity(&entities::ManifoldSolidBrep { outer }));
    }

    // A void's faces already point into it — away from the material, the same
    // way the outer shell's point away from it — so the shell is named in its
    // own direction and nothing is flipped on the way out.
    let mut oriented = Vec::with_capacity(voids.len());
    for shell in &voids {
        let closed_shell_element = write_shell(builder, gmap, shell, &mut cache)?;
        oriented.push(builder.add_entity(&entities::OrientedClosedShell {
            closed_shell_element,
            orientation: true,
        }));
    }
    Ok(builder.add_entity(&entities::BrepWithVoids {
        outer,
        voids: oriented,
    }))
}

/// Writes one shell of a solid as a `CLOSED_SHELL`.
fn write_shell<P: Payload>(
    builder: &mut InstanceBuilder,
    gmap: &Model<P>,
    shell: &ShellRef<'_, P>,
    cache: &mut ExportCache,
) -> Result<EntityId, StepError> {
    let mut cfs_faces = Vec::new();
    for face in shell.faces() {
        cfs_faces.push(write_face(builder, gmap, &face, cache)?);
    }
    Ok(builder.add_entity(&entities::ClosedShell { cfs_faces }))
}

fn write_face<P: Payload>(
    builder: &mut InstanceBuilder,
    gmap: &Model<P>,
    face: &Face<'_, P>,
    cache: &mut ExportCache,
) -> Result<EntityId, StepError> {
    let seamed = SeamedFace::of_face(face)?;
    let same_sense = face_same_sense(face)?;
    let surface = write_surface(builder, face.surface())?;

    // The cut is shared within one face and never beyond it: it is a property
    // of this face's own domain, so another face's cut is a different edge even
    // where the two would land on the same corners.
    let mut cut = FaceCut::of_face(face);
    let mut bounds = Vec::with_capacity(seamed.bounds.len());
    for bound in &seamed.bounds {
        let edge_loop = write_edge_loop(builder, gmap, face, bound, cache, &mut cut)?;
        bounds.push(builder.add_entity(&entities::FaceBound {
            bound: edge_loop,
            // The loop is already written in the face's own traversal
            // direction, so the bound never needs to flip it; the walk
            // direction lives entirely in each ORIENTED_EDGE.
            orientation: true,
            outer: bound.outer,
        }));
    }

    Ok(builder.add_entity(&entities::AdvancedFace {
        bounds,
        face_geometry: surface,
        same_sense,
    }))
}

/// Reports whether the face's outward normal agrees with its surface's.
///
/// This is exactly what `ADVANCED_FACE.same_sense` states, and NGK can answer
/// it without storing anything: [`Face::normal_at`] is the support normal
/// flipped whenever the boundary winds the other way, so comparing the two is
/// the same question asked twice.
fn face_same_sense<P: Payload>(face: &Face<'_, P>) -> Result<bool, TopologyError> {
    // The sign the comparison yields is the boundary's winding, which belongs
    // to the face rather than to a parameter — so any `(u, v)` where the
    // support has a normal at all answers for the whole face, and one that has
    // none is what the refusal below is for.
    let (u, v) = (0.0, 0.0);
    let agreement = face.normal_at(u, v).dot(&face.surface().normal_at(u, v));
    if agreement == 0.0 || !agreement.is_finite() {
        return Err(TopologyError::UnreadableFaceSense { face: face.key() });
    }
    Ok(agreement > 0.0)
}

fn write_edge_loop<P: Payload>(
    builder: &mut InstanceBuilder,
    gmap: &Model<P>,
    face: &Face<'_, P>,
    bound: &SeamedBound,
    cache: &mut ExportCache,
    cut: &mut FaceCut,
) -> Result<EntityId, StepError> {
    let corners = bound_corners(builder, gmap, face, bound, cache, cut)?;

    let mut oriented = Vec::with_capacity(bound.edges.len());
    for (index, element) in bound.edges.iter().enumerate() {
        let (edge_element, forward) = match element {
            SeamedEdge::Real { dart } => write_real_edge(builder, gmap, *dart, cache)?,
            SeamedEdge::Synthetic { from, to } => write_seam(
                builder,
                face,
                *from,
                *to,
                corners[index],
                corners[(index + 1) % corners.len()],
                cut,
            )?,
        };
        oriented.push(builder.add_entity(&entities::OrientedEdge {
            edge_element,
            orientation: forward,
        }));
    }

    Ok(builder.add_entity(&entities::EdgeLoop {
        edge_list: oriented,
    }))
}

/// The `VERTEX_POINT` the walk stands on before each element of a bound.
///
/// A real edge names its own corner, so a stretch of cut that meets one takes
/// the corner from it rather than from a position — which is what keeps a seam
/// welded to the rim it actually ends on. Only a corner between two stretches
/// of cut has nothing to take it from, and that is a place the map genuinely
/// has no vertex for: it is written from the surface.
fn bound_corners<P: Payload>(
    builder: &mut InstanceBuilder,
    gmap: &Model<P>,
    face: &Face<'_, P>,
    bound: &SeamedBound,
    cache: &mut ExportCache,
    cut: &mut FaceCut,
) -> Result<Vec<EntityId>, StepError> {
    let count = bound.edges.len();
    let mut corners = Vec::with_capacity(count);
    for (index, element) in bound.edges.iter().enumerate() {
        let corner = match element {
            SeamedEdge::Real { dart } => {
                let (start, _) = edge_corners(gmap, *dart)?;
                write_corner(builder, gmap, start, cache)?
            }
            SeamedEdge::Synthetic { from, .. } => match &bound.edges[(index + count - 1) % count] {
                SeamedEdge::Real { dart } => {
                    let (_, end) = edge_corners(gmap, *dart)?;
                    write_corner(builder, gmap, end, cache)?
                }
                SeamedEdge::Synthetic { .. } => cut.corner(builder, face, *from),
            },
        };
        corners.push(corner);
    }
    Ok(corners)
}

/// The corners an oriented edge view runs from and to.
///
/// An edge that closes on itself leaves and arrives at the same corner, which
/// `EDGE_CURVE` spells by naming it twice -- whether that corner is a vertex
/// the edge passes through or the place its own curve closes.
fn edge_corners<P: Payload>(
    gmap: &Model<P>,
    dart: Dart,
) -> Result<(Corner, Corner), TopologyError> {
    let edge = Edge::from_dart(gmap, dart).ok_or(TopologyError::UnregisteredEdge { dart })?;
    match edge {
        Edge::Bounded(bounded) => {
            let (start, end) = bounded.vertices();
            Ok((Corner::Vertex(start.key()), Corner::Vertex(end.key())))
        }
        Edge::Marked(marked) => {
            let corner = Corner::Vertex(marked.corner().key());
            Ok((corner, corner))
        }
        Edge::Unmarked(unmarked) => {
            let corner = Corner::Closure(unmarked.key());
            Ok((corner, corner))
        }
    }
}

/// Writes the `EDGE_CURVE` for an edge of the map, shared by key.
///
/// The instance is written in the edge's *default* direction rather than from
/// whichever face reached it first, which is what makes it shareable: both
/// faces then describe the same directed curve and disagree only in their own
/// `ORIENTED_EDGE.orientation`.
fn write_real_edge<P: Payload>(
    builder: &mut InstanceBuilder,
    gmap: &Model<P>,
    dart: Dart,
    cache: &mut ExportCache,
) -> Result<(EntityId, bool), StepError> {
    let edge = Edge::from_dart(gmap, dart).ok_or(TopologyError::UnregisteredEdge { dart })?;
    let key = edge.key();

    let edge_element = match cache.edges.get(&key) {
        Some(&written) => written,
        None => {
            let written = write_edge_curve(builder, gmap, key, cache)?;
            cache.edges.insert(key, written);
            written
        }
    };

    // Which way this loop walks the edge is a combinatorial fact, not a
    // geometric one: comparing corners would say nothing about an edge that
    // closes on itself, whose two ends are the same vertex.
    let forward = gmap.edge_orientation_at_dart(key, dart) == Orientation::Same;
    Ok((edge_element, forward))
}

fn write_edge_curve<P: Payload>(
    builder: &mut InstanceBuilder,
    gmap: &Model<P>,
    key: EdgeKey,
    cache: &mut ExportCache,
) -> Result<EntityId, StepError> {
    let edge = Edge::new(gmap, key);
    let (start, end) = edge_corners(gmap, edge.dart())?;

    let start_id = write_corner(builder, gmap, start, cache)?;
    let end_id = write_corner(builder, gmap, end, cache)?;

    let geometry = edge
        .curve()
        .ok_or(TopologyError::MissingCurve { edge: key })?;
    let curve = write_curve(builder, geometry)?;

    // An edge stores no interval, so which way the support runs between the
    // two corners is derived: the span they bound increases exactly when the
    // curve agrees with start → end.
    let interval = edge
        .parameter_interval()
        .ok_or(TopologyError::MissingCurve { edge: key })?;
    let same_sense = interval.start <= interval.end;

    let edge_curve = builder.add_entity(&entities::EdgeCurve {
        edge_start: start_id,
        edge_end: end_id,
        edge_geometry: curve,
        same_sense,
    });

    Ok(edge_curve)
}

/// Writes one stretch of a synthesized cut, shared with the other stretch that
/// describes it.
///
/// A cut is walked from both sides of the domain, and both sides are the same
/// edge of the shell — so the second one to arrive reuses the first's instance
/// and only says that it runs the other way.
fn write_seam<P: Payload>(
    builder: &mut InstanceBuilder,
    face: &Face<'_, P>,
    from: Point2,
    to: Point2,
    start: EntityId,
    end: EntityId,
    cut: &mut FaceCut,
) -> Result<(EntityId, bool), StepError> {
    let along = varying_axis(from, to).ok_or(TopologyError::UnwritableSeam { face: face.key() })?;
    let key = SeamKey::new(start, end, along);
    let increasing = along.of(to) > along.of(from);
    if let Some(written) = cut.seams.get(&key) {
        return Ok((written.curve, written.increasing == increasing));
    }

    let surface = face.surface();
    let curve =
        iso_curve(surface, from, to).ok_or(TopologyError::UnwritableSeam { face: face.key() })?;
    let geometry = write_curve(builder, &curve)?;
    let span = curve.interval_between(
        surface.point_at(from.x, from.y),
        surface.point_at(to.x, to.y),
    );

    let edge_curve = builder.add_entity(&entities::EdgeCurve {
        edge_start: start,
        edge_end: end,
        edge_geometry: geometry,
        same_sense: span.start <= span.end,
    });

    cut.seams.insert(
        key,
        WrittenSeam {
            curve: edge_curve,
            increasing,
        },
    );
    Ok((edge_curve, true))
}

/// Which parameter a cut runs along, when exactly one of them varies.
fn varying_axis(from: Point2, to: Point2) -> Option<Axis2> {
    let (du, dv) = ((to.x - from.x).abs(), (to.y - from.y).abs());
    match (du <= LINEAR_TOLERANCE, dv <= LINEAR_TOLERANCE) {
        (true, true) | (false, false) => None,
        (false, true) => Some(Axis2::U),
        (true, false) => Some(Axis2::V),
    }
}

/// Writes a `VERTEX_POINT`, shared by the cell the corner stands for.
///
/// Shared by *cell*, never by position: two corners of a solid that happen to
/// coincide are still two corners, and merging them would weld the shape.
fn write_corner<P: Payload>(
    builder: &mut InstanceBuilder,
    gmap: &Model<P>,
    corner: Corner,
    cache: &mut ExportCache,
) -> Result<EntityId, TopologyError> {
    match corner {
        Corner::Vertex(vertex) => {
            if let Some(&existing) = cache.vertices.get(&vertex) {
                return Ok(existing);
            }
            let position = gmap
                .vertex_attr(vertex)
                .map(|attr| attr.point)
                .ok_or(TopologyError::MissingVertexPoint { vertex })?;
            let id = write_vertex_point(builder, position);
            cache.vertices.insert(vertex, id);
            Ok(id)
        }
        Corner::Closure(edge) => {
            if let Some(&existing) = cache.closures.get(&edge) {
                return Ok(existing);
            }
            // Through `point_at_dart` rather than off the curve here, so that
            // where a vertex-free edge closes is decided in one place.
            let position = gmap
                .point_at_dart(Edge::new(gmap, edge).dart())
                .ok_or(TopologyError::MissingCurve { edge })?;
            let id = write_vertex_point(builder, position);
            cache.closures.insert(edge, id);
            Ok(id)
        }
    }
}

/// Writes a `VERTEX_POINT` at `position`, sharing nothing.
fn write_vertex_point(builder: &mut InstanceBuilder, position: Point3) -> EntityId {
    let vertex_geometry = write_point(builder, position);
    builder.add_entity(&entities::VertexPoint { vertex_geometry })
}
