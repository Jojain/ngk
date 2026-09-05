//! The 0-removal pass: fusing the two edges that meet at a shape-free vertex.
//!
//! A vertex carries no shape when exactly two edges meet there and their curves
//! continue each other on one support. Removing it fuses those edges, so the
//! pass rebuilds the fused curve and the parameter curve every incident face
//! keeps for it.

use std::collections::HashSet;

use crate::builders::removal::{MergedCell, is_removable, remove_cell_staged};
use crate::geometry::{Curve, Curve2, Point3, PointCoincidence};
use crate::topology::gmap::{Cell0, Dart, Dim, GMap};
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, FaceKey, VertexKey};
use crate::topology::{TopologyEdit, TopologyEditError};

use super::super::errors::HealingError;
use super::super::options::HealingOptions;
use super::super::predicates::curve::reversed;
use super::super::predicates::{boundary_pcurve, join_curves};
use super::super::report::{HealedCell, HealingReport, SkipReason};
use super::{boundary_dart, edge_key, incident_faces};

/// Offers every scoped vertex to the 0-removal operation.
pub(in crate::healing) fn run<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    options: &HealingOptions,
    report: &mut HealingReport,
) -> Result<(), HealingError> {
    for key in super::scoped_vertices(g.map(), options)? {
        if g.map().vertex_attr(key).is_none() {
            continue;
        }
        match plan(g.map(), key, options) {
            Ok(fusion) => apply(g, fusion, report)?,
            Err(reason) => report.skip(HealedCell::Vertex(key), reason),
        }
    }
    Ok(())
}

/// Everything the fusion needs, resolved before the topology changes.
struct VertexFusion {
    vertex: VertexKey,
    dart: Dart,
    survivor: EdgeKey,
    survivor_dart: Dart,
    curve: Curve,
    /// Rebuilt parameter curves, keyed by their pre-removal boundary dart.
    pcurves: Vec<(FaceKey, Dart, Curve2)>,
}

/// Decides whether the vertex carries shape, and builds the fused geometry.
fn plan<P: Payload>(
    g: &GMap<P>,
    vertex: VertexKey,
    options: &HealingOptions,
) -> Result<VertexFusion, SkipReason> {
    let attr = g.vertex_attr(vertex).ok_or(SkipReason::Unregistered)?;
    let dart = attr.dart;
    let through = attr.point;
    if !is_removable(g, dart, Dim::Zero) {
        return Err(SkipReason::NotRemovable);
    }

    let cell = g
        .orbit(dart, g.orbit_indices(Dim::Zero))
        .collect::<HashSet<_>>();
    let mut incident = Vec::new();
    for d in g.incident_cells(dart, Dim::Zero, Dim::One) {
        let edge = edge_key(g, d).ok_or(SkipReason::Unregistered)?;
        if !incident.contains(&edge) {
            incident.push(edge);
        }
    }
    let [first, second] = incident[..] else {
        return Err(SkipReason::NotBetweenTwoCells);
    };
    // Matches the survivor rule of `remove_cell_staged`, so the fused geometry
    // is built in the direction the surviving identity will keep.
    let (survivor, consumed) = if first <= second {
        (first, second)
    } else {
        (second, first)
    };

    let survivor_dart = surviving_dart(g, survivor, &cell).ok_or(SkipReason::NotRemovable)?;
    let consumed_dart = surviving_dart(g, consumed, &cell).ok_or(SkipReason::NotRemovable)?;
    let start = vertex_point(g, survivor_dart).ok_or(SkipReason::MissingGeometry)?;
    let end = vertex_point(g, consumed_dart).ok_or(SkipReason::MissingGeometry)?;
    if start.coincides(end, options.linear_tolerance) {
        return Err(SkipReason::WouldCloseEdge);
    }

    let survivor_curve = &g
        .edge_attr(survivor)
        .ok_or(SkipReason::MissingGeometry)?
        .curve;
    let consumed_curve = &g
        .edge_attr(consumed)
        .ok_or(SkipReason::MissingGeometry)?
        .curve;
    let curve = join_curves(
        survivor_curve,
        consumed_curve,
        start,
        through,
        end,
        options.linear_tolerance,
        options.angular_tolerance,
    )
    .ok_or(SkipReason::CurvesNotJoinable)?;

    let pcurves = fused_pcurves(
        g,
        &cell,
        FusedBoundary {
            survivor,
            survivor_dart,
            consumed_dart,
            start,
            end,
            curve: &curve,
        },
        options.linear_tolerance,
    )?;

    Ok(VertexFusion {
        vertex,
        dart,
        survivor,
        survivor_dart,
        curve,
        pcurves,
    })
}

/// The fused edge as each incident face will see it.
struct FusedBoundary<'a> {
    survivor: EdgeKey,
    survivor_dart: Dart,
    consumed_dart: Dart,
    start: Point3,
    end: Point3,
    curve: &'a Curve,
}

/// Rebuilds the parameter curve of the fused edge for every incident face.
///
/// A face traverses the fused edge from `start` when it already traversed the
/// surviving edge away from the vanishing vertex, and from `end` otherwise. The
/// rebuilt curve is keyed on whichever dart carries that direction, matching
/// the convention the profile builders use.
fn fused_pcurves<P: Payload>(
    g: &GMap<P>,
    cell: &HashSet<Dart>,
    fused: FusedBoundary<'_>,
    linear: f64,
) -> Result<Vec<(FaceKey, Dart, Curve2)>, SkipReason> {
    let mut pcurves = Vec::new();
    for face in incident_faces(g, fused.survivor_dart) {
        let attr = g.face_attr(face).ok_or(SkipReason::Unregistered)?;
        let boundary = boundary_dart(g, face, fused.survivor).ok_or(SkipReason::Unregistered)?;
        let carries_pcurve = attr.pcurves.contains_key(&boundary)
            || attr.pcurves.contains_key(&g.alpha(Dim::Zero, boundary));
        if !carries_pcurve {
            continue;
        }

        let forward = !cell.contains(&boundary);
        let (key, oriented, start, end) = if forward {
            (boundary, fused.curve.clone(), fused.start, fused.end)
        } else {
            let key = surviving_dart_in_face(g, fused.consumed_dart, face, cell)
                .ok_or(SkipReason::Unregistered)?;
            let oriented = reversed(fused.curve).ok_or(SkipReason::PcurveNotJoinable)?;
            (key, oriented, fused.end, fused.start)
        };
        let pcurve = boundary_pcurve(&attr.surface, &oriented, start, end, linear)
            .ok_or(SkipReason::PcurveNotJoinable)?;
        pcurves.push((face, key, pcurve));
    }
    Ok(pcurves)
}

/// Removes the vertex and writes the fused geometry onto the surviving edge.
fn apply<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    fusion: VertexFusion,
    report: &mut HealingReport,
) -> Result<(), HealingError> {
    let removal = remove_cell_staged(g, fusion.dart, Dim::Zero)?;
    let MergedCell::Edges { survivor, consumed } = removal.merged else {
        return Err(TopologyEditError::MissingLineageAttribute {
            key: crate::topology::EditKey::Edge(fusion.survivor),
        }
        .into());
    };
    debug_assert_eq!(survivor, fusion.survivor, "survivor rules must agree");

    let dart = removal
        .remap(fusion.survivor_dart)
        .expect("a dart outside the removed vertex must survive");
    let attr = g.edge_attr_mut_unchecked(survivor);
    attr.dart = dart;
    attr.curve = fusion.curve;

    let fused = g
        .map()
        .orbit(dart, g.map().orbit_indices(Dim::One))
        .collect::<HashSet<_>>();
    for (face, boundary, pcurve) in fusion.pcurves {
        let Some(boundary) = removal.remap(boundary) else {
            continue;
        };
        let attr = g.face_attr_mut_unchecked(face);
        attr.pcurves.retain(|dart, _| !fused.contains(dart));
        attr.pcurves.insert(boundary, pcurve);
    }

    report.removed_vertices.push(fusion.vertex);
    report.fused_edges.push((survivor, consumed));
    Ok(())
}

/// Returns the lowest dart of `edge` that the vertex removal will keep.
fn surviving_dart<P: Payload>(g: &GMap<P>, edge: EdgeKey, cell: &HashSet<Dart>) -> Option<Dart> {
    let dart = g.edge_attr(edge)?.dart;
    g.orbit(dart, g.orbit_indices(Dim::One))
        .filter(|d| !cell.contains(d))
        .min()
}

/// Returns the surviving dart of an edge orbit that belongs to `face`.
fn surviving_dart_in_face<P: Payload>(
    g: &GMap<P>,
    dart: Dart,
    face: FaceKey,
    cell: &HashSet<Dart>,
) -> Option<Dart> {
    g.orbit(dart, g.orbit_indices(Dim::One)).find(|&d| {
        !cell.contains(&d) && g.cell_key::<crate::topology::gmap::Cell2>(d) == Some(face)
    })
}

/// Returns the position of the vertex containing `dart`.
fn vertex_point<P: Payload>(g: &GMap<P>, dart: Dart) -> Option<Point3> {
    g.attribute::<Cell0>(dart).map(|attr| attr.point)
}
