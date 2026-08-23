use std::collections::{HashMap, HashSet};

use crate::builders::edges::add_edge_staged;
use crate::builders::errors::ChamferError;
use crate::builders::faces::{
    FaceImprint, add_face_staged, add_polygon_staged, split_face_by_imprints_staged,
};
use crate::builders::profiles::curve_pcurve;
use crate::geometry::{
    Curve, Curve2, LINEAR_TOLERANCE, Line2, Point2, Point3, RuledSurface, Surface,
};
use crate::topology::attributes::{FaceAttr, VertexAttr};
use crate::topology::gmap::{Cell0, Cell1, Dart, Dim, GMap};
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, FaceKey, ProfileKey, VertexKey};
use crate::topology::{IsolatedDart, TopologyEdit};

/// Orientation of a dart relative to the directed edge that owns it.
#[derive(Clone, Copy, PartialEq, Eq)]
enum CornerRole {
    IncomingEnd,
    OutgoingStart,
}

/// Chamfers a topology selection in place.
///
/// A standalone profile vertex mutates its line-only 2D corner. Edge and vertex
/// selections attached to faces trim an existing solid shell and sew in new
/// chamfer faces. Passing a profile chamfers all of its corners when it is
/// standalone, or its boundary edges when it belongs to a solid face.
///
/// Solid-edge chamfers support straight planar edges and NURBS edges produced
/// by extruding a planar profile. Solid-vertex chamfers currently require a
/// manifold trihedral corner with straight edges and planar faces. The complete
/// edit is transactional: any invalid target, distance, trim, or sew restores
/// the original map.
///
/// Singular edge and vertex keys are normalized to one-item selections, so the
/// singular and plural call forms execute the same code path.
///
/// # Errors
///
/// Returns [`ChamferError`] when the distance is invalid, the selection is not
/// supported by the current geometry path, or the staged topology cannot be
/// split and sewn consistently.
pub fn chamfer<P: Payload, T: Into<ChamferTarget>>(
    g: &mut GMap<P>,
    target: T,
    distance: f64,
) -> Result<(), ChamferError> {
    g.transaction(|g| {
        validate_distance(distance)?;

        match target.into() {
            ChamferTarget::Edges(edges) => {
                for edge in edges {
                    chamfer_solid_edge(g, edge, distance)?;
                }
                Ok(())
            }
            ChamferTarget::Profile(profile) => chamfer_profile(g, profile, distance),
            ChamferTarget::Vertices(vertices) => {
                for vertex in vertices {
                    chamfer_vertex(g, vertex, distance)?;
                }
                Ok(())
            }
        }
    })
}

/// Replaces one standalone-profile 0-cell with two offset vertices and a new
/// edge between them.
fn chamfer_profile_corner<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    vertex_key: VertexKey,
    distance: f64,
) -> Result<(), ChamferError> {
    let vertex_dart = g
        .vertex(vertex_key)
        .ok_or(ChamferError::MissingChamferVertex { vertex: vertex_key })?
        .dart;
    let (incoming_end, outgoing_start) = profile_corner_darts(g, vertex_dart)?;
    let vertex = vertex_point(g, incoming_end)?;
    let previous = vertex_point(g, g.alpha(Dim::Zero, incoming_end))?;
    let next = vertex_point(g, g.alpha(Dim::Zero, outgoing_start))?;

    let incoming_edge = line_edge_dart(g, incoming_end)?;
    let outgoing_edge = line_edge_dart(g, outgoing_start)?;
    let incoming_offset = offset_point(incoming_edge, vertex, previous, distance)?;
    let outgoing_offset = offset_point(outgoing_edge, vertex, next, distance)?;
    let corner_key = vertex_key;

    // Alpha1 joins the two edge-end occurrences into the original corner.
    // Unlinking it splits that 0-cell before the replacement edge is inserted.
    g.unlink(Dim::One, incoming_end)
        .map_err(ChamferError::from)?;
    g.vertex_attr_mut(corner_key)
        .expect("validated chamfer vertex must remain registered")
        .dart = incoming_end;
    let outgoing_key = g.add_vertex_split_from(
        corner_key,
        VertexAttr::new(outgoing_start, vertex, P::V::default()),
    );
    g.vertex_attr_mut(corner_key)
        .expect("validated chamfer vertex must remain registered")
        .point = incoming_offset;
    g.vertex_attr_mut(outgoing_key)
        .expect("validated chamfer vertex must remain registered")
        .point = outgoing_offset;
    reset_line_edge(g, incoming_end)?;
    reset_line_edge(g, outgoing_start)?;

    // The new edge closes the gap by alpha1-sewing its two endpoints to the
    // now-distinct incoming and outgoing profile vertices.
    let chamfer_edge = add_edge_staged(
        g,
        incoming_offset,
        outgoing_offset,
        Curve::line(incoming_offset, outgoing_offset),
    )
    .map_err(|_| ChamferError::ZeroLengthEdge { dart: incoming_end })?;
    let chamfer_start = g.edge_attr_unchecked(chamfer_edge).dart;
    let chamfer_end = g.alpha(Dim::Zero, chamfer_start);
    sew(g, incoming_end, chamfer_start)?;
    sew(g, chamfer_end, outgoing_start)?;

    Ok(())
}

/// A typed topology selection accepted by [`chamfer`].
pub enum ChamferTarget {
    /// One or more solid boundary edges.
    Edges(Vec<EdgeKey>),
    /// Every edge referenced by a profile.
    Profile(ProfileKey),
    /// One or more standalone-profile or solid-boundary vertices.
    Vertices(Vec<VertexKey>),
}

impl From<EdgeKey> for ChamferTarget {
    fn from(value: EdgeKey) -> Self {
        Self::Edges(vec![value])
    }
}

impl From<Vec<EdgeKey>> for ChamferTarget {
    fn from(value: Vec<EdgeKey>) -> Self {
        Self::Edges(value)
    }
}

impl<const N: usize> From<[EdgeKey; N]> for ChamferTarget {
    fn from(value: [EdgeKey; N]) -> Self {
        Self::Edges(value.into())
    }
}

impl From<ProfileKey> for ChamferTarget {
    fn from(value: ProfileKey) -> Self {
        Self::Profile(value)
    }
}

impl From<VertexKey> for ChamferTarget {
    fn from(value: VertexKey) -> Self {
        Self::Vertices(vec![value])
    }
}

impl From<Vec<VertexKey>> for ChamferTarget {
    fn from(value: Vec<VertexKey>) -> Self {
        Self::Vertices(value)
    }
}

impl<const N: usize> From<[VertexKey; N]> for ChamferTarget {
    fn from(value: [VertexKey; N]) -> Self {
        Self::Vertices(value.into())
    }
}

/// Geometry and incidence captured before a solid-edge chamfer mutates the map.
struct SolidEdgeChamfer {
    incident_faces: [FaceKey; 2],
    endpoint_faces: [FaceKey; 2],
    endpoints: [VertexKey; 2],
    face_offsets: [[Point3; 2]; 2],
    trim_curves: [Curve; 2],
    curved: bool,
}

/// Removes the four face patches surrounding a manifold edge and replaces
/// them with one planar or ruled chamfer face.
fn chamfer_solid_edge<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    edge: EdgeKey,
    distance: f64,
) -> Result<(), ChamferError> {
    let mut prepared = prepare_solid_edge_chamfer(g, edge, distance)?;
    orient_solid_edge_chamfer(g, &mut prepared);
    let mut patch_faces = HashSet::new();
    let mut section_edges = Vec::with_capacity(4);

    // First trim the two faces incident to the selected edge.
    for face_index in 0..2 {
        let face = prepared.incident_faces[face_index];
        let points = prepared.face_offsets[face_index];
        let imprint = if prepared.curved {
            chamfer_curve_imprint(g, edge, face, &prepared.trim_curves[face_index], points)?
        } else {
            planar_line_imprint(g, edge, face, points)?
        };
        let (patch, section) = split_chamfer_face(g, face, imprint, |g, candidate| {
            face_contains_edge(g, candidate, edge)
        })?;
        patch_faces.insert(patch);
        section_edges.push(section);
    }

    // Then trim the endpoint faces so all four sides of the replacement face
    // already exist as survivor boundary edges.
    for endpoint_index in 0..2 {
        let face = prepared.endpoint_faces[endpoint_index];
        let points = [
            prepared.face_offsets[0][endpoint_index],
            prepared.face_offsets[1][endpoint_index],
        ];
        let endpoint = prepared.endpoints[endpoint_index];
        let imprint = planar_line_imprint(g, edge, face, points)?;
        let (patch, section) = split_chamfer_face(g, face, imprint, |g, candidate| {
            face_contains_vertex(g, candidate, endpoint)
        })?;
        patch_faces.insert(patch);
        section_edges.push(section);
    }

    let corners = [
        prepared.face_offsets[0][0],
        prepared.face_offsets[0][1],
        prepared.face_offsets[1][1],
        prepared.face_offsets[1][0],
    ];
    let geometry = prepared.curved.then(|| CurvedChamferFace {
        base_curve: prepared.trim_curves[0].clone(),
        direction: prepared.face_offsets[1][0] - prepared.face_offsets[0][0],
    });
    replace_face_patch(g, &patch_faces, &section_edges, &corners, geometry)
}

/// Validates a solid edge and computes every trim curve and offset point
/// without changing topology.
fn prepare_solid_edge_chamfer<P: Payload>(
    g: &GMap<P>,
    edge: EdgeKey,
    distance: f64,
) -> Result<SolidEdgeChamfer, ChamferError> {
    let edge_view = g
        .edge(edge)
        .ok_or(ChamferError::MissingChamferEdge { edge })?;
    let edge_curve = edge_view
        .curve()
        .cloned()
        .ok_or(ChamferError::MissingEdgeCurve {
            dart: edge_view.dart(),
        })?;
    let curved = !is_linear_curve(Some(&edge_curve));
    if curved && !is_nurbs_curve(&edge_curve) {
        return Err(ChamferError::UnsupportedSolidChamferGeometry { edge });
    }
    let faces = edge_view.faces();
    if faces.len() != 2 {
        return Err(ChamferError::InvalidChamferEdgeIncidence {
            edge,
            count: faces.len(),
        });
    }
    let supported_incident_faces = faces.iter().all(|face| {
        matches!(face.surface(), Surface::Plane(_))
            || (curved && matches!(face.surface(), Surface::Ruled(_)))
    });
    if !supported_incident_faces {
        return Err(ChamferError::UnsupportedSolidChamferGeometry { edge });
    }

    let endpoints = [edge_view.start().key(), edge_view.end().key()];
    let incident_faces = [faces[0].key(), faces[1].key()];
    let mut endpoint_faces = [faces[0].key(); 2];
    let mut face_offsets = [[Point3::origin(); 2]; 2];

    let mut endpoint_points = [Point3::origin(); 2];
    for (endpoint_index, endpoint) in endpoints.iter().copied().enumerate() {
        let vertex = g.vertex_unchecked(endpoint);
        let other_faces = vertex
            .faces()
            .into_iter()
            .filter(|face| !incident_faces.contains(&face.key()))
            .collect::<Vec<_>>();
        if other_faces.len() != 1 || !matches!(other_faces[0].surface(), Surface::Plane(_)) {
            return Err(ChamferError::UnsupportedSolidChamferGeometry { edge });
        }
        endpoint_faces[endpoint_index] = other_faces[0].key();
        endpoint_points[endpoint_index] = *vertex
            .point()
            .ok_or(ChamferError::MissingVertexPoint { dart: vertex.dart })?;

        for (face_index, face) in faces.iter().enumerate() {
            let adjacent = face
                .edges()
                .into_iter()
                .find(|candidate| {
                    candidate.key() != edge
                        && (candidate.start().key() == endpoint
                            || candidate.end().key() == endpoint)
                })
                .ok_or(ChamferError::UnsupportedSolidChamferGeometry { edge })?;
            let endpoint_point = endpoint_points[endpoint_index];
            let neighbor = if adjacent.start().key() == endpoint {
                adjacent.end()
            } else {
                adjacent.start()
            };
            let neighbor_point = *neighbor.point().ok_or(ChamferError::MissingVertexPoint {
                dart: neighbor.dart,
            })?;
            face_offsets[face_index][endpoint_index] =
                offset_point(adjacent.dart(), endpoint_point, neighbor_point, distance)?;
        }
    }

    // A supported curved trim is a rigid translation of the selected NURBS
    // edge. Equal endpoint translations prove that a single translated curve
    // represents the full boundary exactly.
    let trim_curves = std::array::from_fn(|face_index| {
        let first_offset = face_offsets[face_index][0] - endpoint_points[0];
        edge_curve.translated(first_offset)
    });
    let [first_trim, second_trim] = trim_curves;
    let trim_curves = [
        first_trim.map_err(|_| ChamferError::UnsupportedSolidChamferGeometry { edge })?,
        second_trim.map_err(|_| ChamferError::UnsupportedSolidChamferGeometry { edge })?,
    ];
    for offsets in &face_offsets {
        let first_offset = offsets[0] - endpoint_points[0];
        let second_offset = offsets[1] - endpoint_points[1];
        if (first_offset - second_offset).norm() > LINEAR_TOLERANCE {
            return Err(ChamferError::UnsupportedSolidChamferGeometry { edge });
        }
    }

    Ok(SolidEdgeChamfer {
        incident_faces,
        endpoint_faces,
        endpoints,
        face_offsets,
        trim_curves,
        curved,
    })
}

/// Splits one face by an imprint and identifies both the patch to remove and
/// the section edge that remains on the survivor.
fn split_chamfer_face<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    face: FaceKey,
    imprint: FaceImprint,
    is_patch: impl Fn(&GMap<P>, FaceKey) -> bool,
) -> Result<(FaceKey, EdgeKey), ChamferError> {
    let splits = split_face_by_imprints_staged(g, face, &[imprint])
        .map_err(|_| ChamferError::ChamferFaceSplitFailed { face })?;
    let split = splits
        .into_iter()
        .next()
        .ok_or(ChamferError::ChamferFaceSplitFailed { face })?;
    let patch = [split.first, split.second]
        .into_iter()
        .find(|candidate| is_patch(g, *candidate))
        .ok_or(ChamferError::ChamferFaceSplitFailed { face })?;
    let section = split
        .section_edges
        .into_iter()
        .next()
        .ok_or(ChamferError::ChamferFaceSplitFailed { face })?;
    Ok((patch, section))
}

/// Builds synchronized model-space and UV-space line geometry for a planar
/// face split.
fn planar_line_imprint<P: Payload>(
    g: &GMap<P>,
    edge: EdgeKey,
    face: FaceKey,
    points: [Point3; 2],
) -> Result<FaceImprint, ChamferError> {
    let plane = match g.face(face).map(|face| face.surface().clone()) {
        Some(Surface::Plane(plane)) => plane,
        _ => return Err(ChamferError::UnsupportedSolidChamferGeometry { edge }),
    };
    Ok(FaceImprint::new(
        Curve::line(points[0], points[1]),
        Curve2::Line(Line2::new(
            plane.parameter_at(points[0]),
            plane.parameter_at(points[1]),
        )),
    ))
}

/// Builds the face pcurve for a translated curved chamfer boundary.
///
/// Planes use exact projection. Ruled surfaces recover endpoint parameters and
/// verify samples because the expected trim is an isoparametric straight line
/// in that surface's parameter space.
fn chamfer_curve_imprint<P: Payload>(
    g: &GMap<P>,
    edge: EdgeKey,
    face: FaceKey,
    curve: &Curve,
    points: [Point3; 2],
) -> Result<FaceImprint, ChamferError> {
    let surface = g
        .face(face)
        .map(|face| face.surface().clone())
        .ok_or(ChamferError::UnsupportedSolidChamferGeometry { edge })?;
    let pcurve = match &surface {
        Surface::Plane(plane) => curve_pcurve(curve, points[0], points[1], plane)
            .map_err(|_| ChamferError::UnsupportedSolidChamferGeometry { edge })?,
        Surface::Ruled(_) => {
            let start = surface
                .closest_parameter(points[0])
                .map_err(|_| ChamferError::UnsupportedSolidChamferGeometry { edge })?;
            let end = surface
                .closest_parameter(points[1])
                .map_err(|_| ChamferError::UnsupportedSolidChamferGeometry { edge })?;
            let pcurve = Curve2::Line(Line2::new(start, end));
            for parameter in [0.0, 0.25, 0.5, 0.75, 1.0] {
                let uv = pcurve.point_at(parameter);
                if (surface.point_at(uv.x, uv.y) - curve.point_at(parameter)).norm()
                    > 10.0 * LINEAR_TOLERANCE
                {
                    return Err(ChamferError::UnsupportedSolidChamferGeometry { edge });
                }
            }
            pcurve
        }
        _ => return Err(ChamferError::UnsupportedSolidChamferGeometry { edge }),
    };
    Ok(FaceImprint::new(curve.clone(), pcurve))
}

fn face_contains_edge<P: Payload>(g: &GMap<P>, face: FaceKey, edge: EdgeKey) -> bool {
    g.face(face)
        .is_some_and(|face| face.edges().iter().any(|candidate| candidate.key() == edge))
}

fn face_contains_vertex<P: Payload>(g: &GMap<P>, face: FaceKey, vertex: VertexKey) -> bool {
    g.face(face).is_some_and(|face| {
        face.vertices()
            .iter()
            .any(|candidate| candidate.key() == vertex)
    })
}

/// Orders the two incident-face trims so the replacement face points toward
/// the sum of the original outward normals.
fn orient_solid_edge_chamfer<P: Payload>(g: &GMap<P>, chamfer: &mut SolidEdgeChamfer) {
    let corners = [
        chamfer.face_offsets[0][0],
        chamfer.face_offsets[0][1],
        chamfer.face_offsets[1][1],
        chamfer.face_offsets[1][0],
    ];
    let edge_direction = corners[1] - corners[0];
    let across = corners[3] - corners[0];
    let candidate_normal = edge_direction.cross(&across);
    let outward = chamfer
        .incident_faces
        .into_iter()
        .filter_map(|face| g.face(face))
        .map(|face| *face.normal_at(0.0, 0.0))
        .sum::<nalgebra::Vector3<f64>>();
    if candidate_normal.dot(&outward) < 0.0 {
        chamfer.incident_faces.swap(0, 1);
        chamfer.face_offsets.swap(0, 1);
        chamfer.trim_curves.swap(0, 1);
    }
}

/// Dispatches a profile to either the batched solid-rim algorithm or the
/// sequential standalone-corner algorithm.
fn chamfer_profile<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    profile: ProfileKey,
    distance: f64,
) -> Result<(), ChamferError> {
    let profile_view = g
        .profile(profile)
        .ok_or(ChamferError::UnsupportedChamferTarget)?;
    let edges = profile_view.edges();
    if edges.iter().any(|edge| !edge.faces().is_empty()) {
        return chamfer_solid_profile(g, profile, distance);
    }

    let closed = edges
        .first()
        .zip(edges.last())
        .is_some_and(|(first, last)| first.start().key() == last.end().key());
    let corner_vertices = edges
        .iter()
        .take(edges.len().saturating_sub((!closed) as usize))
        .map(|edge| edge.end().key())
        .collect::<Vec<_>>();
    for vertex in corner_vertices {
        chamfer_profile_corner(g, vertex, distance)?;
    }
    Ok(())
}

/// Interprets a domain vertex by incidence: a vertex without faces is a 2D
/// profile corner; a vertex with faces is a solid-boundary selection.
fn chamfer_vertex<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    vertex: VertexKey,
    distance: f64,
) -> Result<(), ChamferError> {
    let is_standalone_profile_vertex = g
        .vertex(vertex)
        .ok_or(ChamferError::MissingChamferVertex { vertex })?
        .faces()
        .is_empty();
    if is_standalone_profile_vertex {
        chamfer_profile_corner(g, vertex, distance)
    } else {
        chamfer_solid_vertex(g, vertex, distance)
    }
}

/// Complete immutable plan for replacing one solid face rim with a bevel ring.
///
/// Capturing all adjacent edge data first is essential: chamfering one edge at
/// a time would invalidate the keys and incidences needed by its neighbors.
struct SolidProfileChamfer {
    target_face: FaceKey,
    edges: Vec<EdgeKey>,
    side_faces: Vec<FaceKey>,
    lower_corners: Vec<Point3>,
    inset_corners: Vec<Point3>,
    target_normal: nalgebra::Vector3<f64>,
    side_normals: Vec<nalgebra::Vector3<f64>>,
}

/// Replaces a complete planar outer profile with an inset cap and one chamfer
/// face per profile edge.
fn chamfer_solid_profile<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    profile: ProfileKey,
    distance: f64,
) -> Result<(), ChamferError> {
    let prepared = prepare_solid_profile_chamfer(g, profile, distance)?;
    let mut patch_faces = HashSet::from([prepared.target_face]);
    let mut section_edges = Vec::with_capacity(prepared.edges.len());

    // Split every side face below the selected rim. The target cap plus the
    // upper side strips form one connected patch to remove.
    for index in 0..prepared.edges.len() {
        let edge = prepared.edges[index];
        let face = prepared.side_faces[index];
        let next = (index + 1) % prepared.edges.len();
        let points = [prepared.lower_corners[index], prepared.lower_corners[next]];
        let imprint = planar_line_imprint(g, edge, face, points)?;
        let (patch, section) = split_chamfer_face(g, face, imprint, |g, candidate| {
            face_contains_edge(g, candidate, edge)
        })?;
        patch_faces.insert(patch);
        section_edges.push(section);
    }

    // Removing the complete patch before adding replacements avoids the
    // side-effects of sequentially chamfering adjacent GMap cells.
    let boundary_darts = remove_face_patch(g, &patch_faces, &section_edges)?;
    add_profile_chamfer_faces(g, &prepared, &boundary_darts)
}

/// Validates the current first-pass solid-profile domain and computes its cap
/// inset and side-wall offsets without mutating topology.
///
/// This path currently accepts a closed, convex, straight-edged outer loop on
/// a planar cap, with one planar side face and one outgoing solid edge at each
/// rim vertex.
fn prepare_solid_profile_chamfer<P: Payload>(
    g: &GMap<P>,
    profile: ProfileKey,
    distance: f64,
) -> Result<SolidProfileChamfer, ChamferError> {
    let profile_view = g
        .profile(profile)
        .ok_or(ChamferError::UnsupportedChamferTarget)?;
    let profile_edges = profile_view.edges();
    if profile_edges.len() < 3
        || profile_edges
            .last()
            .zip(profile_edges.first())
            .is_none_or(|(last, first)| last.end().key() != first.start().key())
        || profile_edges
            .iter()
            .any(|edge| !is_linear_curve(edge.curve()))
    {
        return Err(ChamferError::UnsupportedChamferTarget);
    }

    // A profile is its own alpha0/alpha1 component; identify the face that uses
    // that exact component rather than choosing an arbitrary incident face.
    let target_face = profile_edges[0]
        .faces()
        .into_iter()
        .find(|face| face.loops().iter().any(|loop_| loop_.key() == profile))
        .ok_or(ChamferError::UnsupportedChamferTarget)?;
    if target_face.loops().len() != 1
        || target_face.outer_loop().key() != profile
        || !matches!(target_face.surface(), Surface::Plane(_))
    {
        return Err(ChamferError::UnsupportedChamferTarget);
    }
    let target_face_key = target_face.key();
    let target_normal = *g
        .face(target_face_key)
        .expect("validated target face must remain registered")
        .normal_at(0.0, 0.0);
    let plane = match target_face.surface() {
        Surface::Plane(plane) => plane.clone(),
        _ => unreachable!("the target face was validated as planar"),
    };
    let edge_keys = profile_edges
        .iter()
        .map(|edge| edge.key())
        .collect::<Vec<_>>();
    let selected_edges = edge_keys.iter().copied().collect::<HashSet<_>>();
    let mut side_faces = Vec::with_capacity(edge_keys.len());
    let mut side_normals = Vec::with_capacity(edge_keys.len());
    let mut lower_corners = Vec::with_capacity(edge_keys.len());
    let mut polygon = Vec::with_capacity(edge_keys.len());

    for edge in &profile_edges {
        let faces = edge.faces();
        if faces.len() != 2 || !faces.iter().any(|face| face.key() == target_face_key) {
            return Err(ChamferError::InvalidChamferEdgeIncidence {
                edge: edge.key(),
                count: faces.len(),
            });
        }
        let side_face = faces
            .into_iter()
            .find(|face| face.key() != target_face_key)
            .ok_or(ChamferError::UnsupportedSolidChamferGeometry { edge: edge.key() })?;
        if !matches!(side_face.surface(), Surface::Plane(_)) {
            return Err(ChamferError::UnsupportedSolidChamferGeometry { edge: edge.key() });
        }
        let side_face_key = side_face.key();
        side_normals.push(*g.face_unchecked(side_face_key).normal_at(0.0, 0.0));
        side_faces.push(side_face_key);

        let vertex = edge.start();
        let point = *vertex
            .point()
            .ok_or(ChamferError::MissingVertexPoint { dart: vertex.dart })?;
        polygon.push(plane.parameter_at(point));
        // The one non-profile edge leads away from the cap and supplies the
        // lower corner of each bevel face at the requested edge distance.
        let outside_edges = vertex
            .edges()
            .into_iter()
            .filter(|candidate| !selected_edges.contains(&candidate.key()))
            .collect::<Vec<_>>();
        if outside_edges.len() != 1 || !is_linear_curve(outside_edges[0].curve()) {
            return Err(ChamferError::UnsupportedSolidVertexChamferGeometry {
                vertex: vertex.key(),
            });
        }
        let outside = &outside_edges[0];
        let neighbor = if outside.start().key() == vertex.key() {
            outside.end()
        } else {
            outside.start()
        };
        let neighbor_point = *neighbor.point().ok_or(ChamferError::MissingVertexPoint {
            dart: neighbor.dart,
        })?;
        lower_corners.push(offset_point(
            outside.dart(),
            point,
            neighbor_point,
            distance,
        )?);
    }

    let inset_uv =
        inset_convex_polygon(&polygon, distance).ok_or(ChamferError::UnsupportedChamferTarget)?;
    let inset_corners = inset_uv
        .into_iter()
        .map(|point| plane.point_at(point.x, point.y))
        .collect();
    Ok(SolidProfileChamfer {
        target_face: target_face_key,
        edges: edge_keys,
        side_faces,
        lower_corners,
        inset_corners,
        target_normal,
        side_normals,
    })
}

/// Intersects consecutive inward-offset support lines of a convex polygon.
///
/// Returns `None` for degenerate, parallel, inverted, or non-convex results.
fn inset_convex_polygon(points: &[Point2], distance: f64) -> Option<Vec<Point2>> {
    let area = signed_polygon_area(points);
    if area.abs() <= LINEAR_TOLERANCE {
        return None;
    }
    let winding = area.signum();
    let count = points.len();
    let mut offset_origins = Vec::with_capacity(count);
    let mut directions = Vec::with_capacity(count);
    for index in 0..count {
        let direction = points[(index + 1) % count] - points[index];
        let length = direction.norm();
        if length <= LINEAR_TOLERANCE {
            return None;
        }
        let tangent = direction / length;
        let inward = nalgebra::Vector2::new(-tangent.y, tangent.x) * winding;
        offset_origins.push(points[index] + inward * distance);
        directions.push(tangent);
    }

    // Each inset corner is the intersection of the preceding and following
    // offset lines, preserving the source loop's winding.
    let mut inset = Vec::with_capacity(count);
    for index in 0..count {
        let previous = (index + count - 1) % count;
        let denominator = cross2(directions[previous], directions[index]);
        if denominator.abs() <= LINEAR_TOLERANCE {
            return None;
        }
        let parameter = cross2(
            offset_origins[index] - offset_origins[previous],
            directions[index],
        ) / denominator;
        inset.push(offset_origins[previous] + directions[previous] * parameter);
    }

    let inset_area = signed_polygon_area(&inset);
    let convex = (0..count).all(|index| {
        let first = inset[(index + 1) % count] - inset[index];
        let second = inset[(index + 2) % count] - inset[(index + 1) % count];
        cross2(first, second) * winding > LINEAR_TOLERANCE
    });
    (inset_area * area > LINEAR_TOLERANCE && convex).then_some(inset)
}

fn signed_polygon_area(points: &[Point2]) -> f64 {
    (0..points.len())
        .map(|index| {
            let next = (index + 1) % points.len();
            points[index].x * points[next].y - points[next].x * points[index].y
        })
        .sum::<f64>()
        * 0.5
}

fn cross2(first: nalgebra::Vector2<f64>, second: nalgebra::Vector2<f64>) -> f64 {
    first.x * second.y - first.y * second.x
}

/// Adds the inset cap and bevel ring, then alpha2-sews every matching boundary
/// into the surviving shell.
fn add_profile_chamfer_faces<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    prepared: &SolidProfileChamfer,
    boundary_darts: &[Dart],
) -> Result<(), ChamferError> {
    let top_profile = add_polygon_staged(g, &prepared.inset_corners);
    add_face_staged(g, top_profile).map_err(|_| ChamferError::UnsupportedChamferTarget)?;
    let top_darts = g
        .profile_unchecked(top_profile)
        .darts()
        .step_by(2)
        .collect::<Vec<_>>();
    let mut chamfer_darts = Vec::with_capacity(prepared.edges.len());

    // Create all faces before sewing so each diagonal can be matched against
    // the next bevel face without traversal changing under us.
    for index in 0..prepared.edges.len() {
        let next = (index + 1) % prepared.edges.len();
        let mut corners = vec![
            prepared.inset_corners[index],
            prepared.inset_corners[next],
            prepared.lower_corners[next],
            prepared.lower_corners[index],
        ];
        let candidate_normal = (corners[1] - corners[0]).cross(&(corners[3] - corners[0]));
        let outward = prepared.target_normal + prepared.side_normals[index];
        if candidate_normal.dot(&outward) < 0.0 {
            corners.reverse();
        }
        let profile = add_polygon_staged(g, &corners);
        add_face_staged(g, profile).map_err(|_| ChamferError::UnsupportedChamferTarget)?;
        chamfer_darts.push(
            g.profile_unchecked(profile)
                .darts()
                .step_by(2)
                .collect::<Vec<_>>(),
        );
    }

    // Each bevel quad is sewn to the lower survivor, the inset cap, and the
    // following bevel quad along its shared diagonal.
    for index in 0..prepared.edges.len() {
        sew_matching_boundary(g, boundary_darts[index], &chamfer_darts[index])?;
        sew_matching_boundary(g, top_darts[index], &chamfer_darts[index])?;
        let next = (index + 1) % prepared.edges.len();
        let diagonal = find_boundary_dart(
            g,
            &chamfer_darts[index],
            prepared.inset_corners[next],
            prepared.lower_corners[next],
        )?;
        sew_matching_boundary(g, diagonal, &chamfer_darts[next])?;
    }
    Ok(())
}

/// Finds the geometrically coincident candidate edge, orients its dart to the
/// boundary start vertex, and alpha2-sews the two face boundaries.
fn sew_matching_boundary<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    boundary: Dart,
    candidates: &[Dart],
) -> Result<(), ChamferError> {
    let start = vertex_point(g, boundary)?;
    let end = vertex_point(g, g.alpha(Dim::Zero, boundary))?;
    let candidate = find_boundary_dart(g, candidates, start, end)?;
    let candidate_start = vertex_point(g, candidate)?;
    let oriented = if (candidate_start - start).norm() <= LINEAR_TOLERANCE {
        candidate
    } else {
        g.alpha(Dim::Zero, candidate)
    };
    g.sew(Dim::Two, oriented, boundary)
        .map_err(|_| ChamferError::SewFailed {
            dim: Dim::Two,
            first: oriented,
            second: boundary,
        })
}

/// Finds a candidate boundary dart whose unordered geometric endpoints match
/// the requested segment within linear tolerance.
fn find_boundary_dart<P: Payload>(
    g: &GMap<P>,
    candidates: &[Dart],
    start: Point3,
    end: Point3,
) -> Result<Dart, ChamferError> {
    candidates
        .iter()
        .copied()
        .find(|dart| {
            let candidate_start = vertex_point(g, *dart).ok();
            let candidate_end = vertex_point(g, g.alpha(Dim::Zero, *dart)).ok();
            matches!((candidate_start, candidate_end), (Some(first), Some(second)) if
                ((first - start).norm() <= LINEAR_TOLERANCE
                    && (second - end).norm() <= LINEAR_TOLERANCE)
                || ((first - end).norm() <= LINEAR_TOLERANCE
                    && (second - start).norm() <= LINEAR_TOLERANCE))
        })
        .ok_or(ChamferError::UnsupportedChamferTarget)
}

/// Removes the three corner patches around a trihedral vertex and replaces
/// them with one outward-oriented triangular chamfer face.
fn chamfer_solid_vertex<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    vertex: VertexKey,
    distance: f64,
) -> Result<(), ChamferError> {
    let vertex_view = g
        .vertex(vertex)
        .ok_or(ChamferError::MissingChamferVertex { vertex })?;
    let edges = vertex_view.edges();
    let faces = vertex_view.faces();
    if edges.len() != 3
        || faces.len() != 3
        || edges.iter().any(|edge| !is_linear_curve(edge.curve()))
        || faces
            .iter()
            .any(|face| !matches!(face.surface(), Surface::Plane(_)))
    {
        return Err(ChamferError::UnsupportedSolidVertexChamferGeometry { vertex });
    }
    let vertex_point = *vertex_view
        .point()
        .ok_or(ChamferError::MissingVertexPoint {
            dart: vertex_view.dart,
        })?;
    let edge_keys = edges.iter().map(|edge| edge.key()).collect::<Vec<_>>();
    let face_keys = faces.iter().map(|face| face.key()).collect::<Vec<_>>();
    let mut offsets = HashMap::new();
    for edge in &edges {
        let neighbor = if edge.start().key() == vertex {
            edge.end()
        } else {
            edge.start()
        };
        let neighbor_point = *neighbor.point().ok_or(ChamferError::MissingVertexPoint {
            dart: neighbor.dart,
        })?;
        offsets.insert(
            edge.key(),
            offset_point(edge.dart(), vertex_point, neighbor_point, distance)?,
        );
    }

    let mut patch_faces = HashSet::new();
    let mut section_edges = Vec::with_capacity(3);
    for face in face_keys.iter().copied() {
        let face_edges = g
            .face_unchecked(face)
            .edges()
            .into_iter()
            .filter(|edge| edge_keys.contains(&edge.key()))
            .map(|edge| edge.key())
            .collect::<Vec<_>>();
        if face_edges.len() != 2 {
            return Err(ChamferError::UnsupportedSolidVertexChamferGeometry { vertex });
        }
        let points = [offsets[&face_edges[0]], offsets[&face_edges[1]]];
        let reference_edge = face_edges[0];
        let imprint = planar_line_imprint(g, reference_edge, face, points)?;
        let (patch, section) = split_chamfer_face(g, face, imprint, |g, candidate| {
            face_contains_vertex(g, candidate, vertex)
        })?;
        patch_faces.insert(patch);
        section_edges.push(section);
    }

    let mut corners = edge_keys
        .iter()
        .map(|edge| offsets[edge])
        .collect::<Vec<_>>();
    let outward = face_keys
        .iter()
        .filter_map(|face| g.face(*face))
        .map(|face| *face.normal_at(0.0, 0.0))
        .sum::<nalgebra::Vector3<f64>>();
    if (corners[1] - corners[0])
        .cross(&(corners[2] - corners[0]))
        .dot(&outward)
        < 0.0
    {
        corners.reverse();
    }
    replace_face_patch(g, &patch_faces, &section_edges, &corners, None)
}

fn is_linear_curve(curve: Option<&Curve>) -> bool {
    match curve {
        Some(Curve::Line(_)) => true,
        Some(Curve::Bounded(curve)) => matches!(curve.inner(), Curve::Line(_)),
        _ => false,
    }
}

fn is_nurbs_curve(curve: &Curve) -> bool {
    match curve {
        Curve::Nurbs(_) => true,
        Curve::Bounded(curve) => is_nurbs_curve(curve.inner()),
        _ => false,
    }
}

/// Geometry needed to build a ruled chamfer face from one NURBS boundary.
struct CurvedChamferFace {
    base_curve: Curve,
    direction: nalgebra::Vector3<f64>,
}

/// Removes a connected face patch, creates one replacement face, and sews it
/// to every surviving section edge.
fn replace_face_patch<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    patch_faces: &HashSet<FaceKey>,
    section_edges: &[EdgeKey],
    corners: &[Point3],
    curved_geometry: Option<CurvedChamferFace>,
) -> Result<(), ChamferError> {
    let boundary_darts = remove_face_patch(g, patch_faces, section_edges)?;
    let profile = add_polygon_staged(g, corners);
    match curved_geometry {
        Some(geometry) => add_curved_chamfer_face(g, profile, corners, geometry)?,
        None => add_face_staged(g, profile).map_err(|_| ChamferError::UnsupportedChamferTarget)?,
    };
    let candidates = g
        .profile_unchecked(profile)
        .darts()
        .step_by(2)
        .collect::<Vec<_>>();
    for boundary in boundary_darts {
        sew_matching_boundary(g, boundary, &candidates)?;
    }
    Ok(())
}

/// Detaches and deletes a connected set of faces while preserving the section
/// edges that bound the surviving shell.
///
/// The returned darts are remapped survivor-side representatives, ready for
/// alpha2 sewing after compacting the removed isolated darts.
fn remove_face_patch<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    patch_faces: &HashSet<FaceKey>,
    section_edges: &[EdgeKey],
) -> Result<Vec<Dart>, ChamferError> {
    let patch_darts = patch_faces
        .iter()
        .flat_map(|face| {
            let root = g.face_attr_unchecked(*face).outer_loop;
            g.orbit(root, g.orbit_indices(Dim::Two)).collect::<Vec<_>>()
        })
        .collect::<HashSet<_>>();
    let mut boundary_darts = section_edges
        .iter()
        .map(|edge| {
            g.edge_unchecked(*edge)
                .darts()
                .find_map(|dart| {
                    let opposite = g.alpha(Dim::Two, dart);
                    (patch_darts.contains(&dart) && !patch_darts.contains(&opposite))
                        .then_some(opposite)
                })
                .ok_or(ChamferError::MissingChamferEdge { edge: *edge })
        })
        .collect::<Result<Vec<_>, _>>()?;
    let survivor = *boundary_darts
        .first()
        .ok_or(ChamferError::UnsupportedChamferTarget)?;

    // A lower-dimensional cell is deleted only when its complete orbit lies in
    // the removed patch; section cells retain their existing domain identity.
    let removed_edges = g
        .iter_edges()
        .filter_map(|(key, attr)| {
            g.orbit(attr.dart, g.orbit_indices(Dim::One))
                .all(|dart| patch_darts.contains(&dart))
                .then_some(key)
        })
        .collect::<Vec<_>>();
    let removed_vertices = g
        .iter_vertices()
        .filter_map(|(key, attr)| {
            g.orbit(attr.dart, g.orbit_indices(Dim::Zero))
                .all(|dart| patch_darts.contains(&dart))
                .then_some(key)
        })
        .collect::<Vec<_>>();
    let removed_profiles = g
        .iter_profiles()
        .filter_map(|(key, attr)| patch_darts.contains(&attr.dart).then_some(key))
        .collect::<Vec<_>>();

    let boundary_vertices = boundary_darts
        .iter()
        .flat_map(|dart| [*dart, g.alpha(Dim::Zero, *dart)])
        .filter_map(|dart| g.cell_key::<Cell0>(dart).map(|key| (key, dart)))
        .collect::<HashMap<_, _>>();

    // Detach the patch from the survivor before removing any attributes or
    // darts, preserving valid representatives on the survivor side.
    for dart in patch_darts.iter().copied() {
        let opposite = g.alpha(Dim::Two, dart);
        if !patch_darts.contains(&opposite) && opposite != dart {
            g.unlink(Dim::Two, dart).map_err(ChamferError::from)?;
        }
    }

    for (edge, dart) in section_edges
        .iter()
        .copied()
        .zip(boundary_darts.iter().copied())
    {
        g.edge_attr_mut(edge)
            .ok_or(ChamferError::MissingChamferEdge { edge })?
            .dart = g.cell_representative(dart, Dim::One);
    }
    for (vertex, dart) in boundary_vertices {
        let representative = g.cell_representative(dart, Dim::Zero);
        if let Some(attr) = g.vertex_attr_mut(vertex) {
            attr.dart = representative;
        }
    }
    // Sheet and solid roots are arbitrary representative darts. Move any root
    // inside the removed patch to a survivor before compaction remaps darts.
    let sheet_roots = g
        .iter_sheets()
        .map(|(key, sheet)| (key, sheet.dart))
        .collect::<Vec<_>>();
    for (key, dart) in sheet_roots {
        if patch_darts.contains(&dart) {
            g.sheet_attr_mut_unchecked(key).dart = survivor;
        }
    }
    let solid_keys = g.iter_solids().map(|(key, _)| key).collect::<Vec<_>>();
    for solid in solid_keys {
        let attr = g.solid_attr_mut_unchecked(solid);
        if patch_darts.contains(&attr.outer_shell) {
            attr.outer_shell = survivor;
        }
        if let Some(inner_shells) = &mut attr.inner_shells {
            for shell in inner_shells {
                if patch_darts.contains(shell) {
                    *shell = survivor;
                }
            }
        }
    }

    for face in patch_faces {
        g.remove_face(*face);
    }
    for profile in removed_profiles {
        g.remove_profile(profile);
    }
    for edge in removed_edges {
        g.remove_edge(edge);
    }
    for vertex in removed_vertices {
        g.remove_vertex(vertex);
    }
    for dart in patch_darts.iter().copied() {
        for dim in [Dim::Zero, Dim::One, Dim::Two, Dim::Three] {
            if !g.is_free(dart, dim) {
                g.unlink(dim, dart).map_err(ChamferError::from)?;
            }
        }
    }
    let dart_remap =
        g.remove_isolated_darts(patch_darts.into_iter().map(IsolatedDart::new).collect());
    for dart in &mut boundary_darts {
        *dart = dart_remap[dart];
    }
    Ok(boundary_darts)
}

/// Registers a four-edge ruled face whose opposite boundaries are translated
/// copies of the selected NURBS edge.
fn add_curved_chamfer_face<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    profile: ProfileKey,
    corners: &[Point3],
    geometry: CurvedChamferFace,
) -> Result<FaceKey, ChamferError> {
    let edges = g
        .profile_unchecked(profile)
        .edges()
        .into_iter()
        .map(|edge| (edge.key(), edge.dart()))
        .collect::<Vec<_>>();
    if edges.len() != 4 || corners.len() != 4 {
        return Err(ChamferError::UnsupportedChamferTarget);
    }

    let opposite_curve = geometry
        .base_curve
        .translated(geometry.direction)
        .map_err(|_| ChamferError::UnsupportedChamferTarget)?;
    let reversed_opposite = Curve::Nurbs(
        opposite_curve
            .to_nurbs()
            .map_err(|_| ChamferError::UnsupportedChamferTarget)?
            .reversed(),
    );
    let boundary_curves = [
        geometry.base_curve.clone(),
        Curve::line(corners[1], corners[2]),
        reversed_opposite,
        Curve::line(corners[3], corners[0]),
    ];
    for ((edge, _), curve) in edges.iter().zip(boundary_curves) {
        g.edge_attr_mut(*edge)
            .ok_or(ChamferError::MissingChamferEdge { edge: *edge })?
            .curve = curve;
    }

    let interval = geometry
        .base_curve
        .parameters_between(corners[0], corners[1]);
    // The ruled surface uses the base-curve parameter as `u` and translation
    // fraction as `v`, so its four pcurves form a unit-height parameter strip.
    let uv = [
        (
            nalgebra::Point2::new(interval.start, 0.0),
            nalgebra::Point2::new(interval.end, 0.0),
        ),
        (
            nalgebra::Point2::new(interval.end, 0.0),
            nalgebra::Point2::new(interval.end, 1.0),
        ),
        (
            nalgebra::Point2::new(interval.end, 1.0),
            nalgebra::Point2::new(interval.start, 1.0),
        ),
        (
            nalgebra::Point2::new(interval.start, 1.0),
            nalgebra::Point2::new(interval.start, 0.0),
        ),
    ];
    let pcurves = edges
        .iter()
        .zip(uv)
        .map(|((_, dart), (start, end))| (*dart, Curve2::Line(Line2::new(start, end))))
        .collect::<HashMap<_, _>>();
    let loop_dart = g.profile_attr_unchecked(profile).dart;
    Ok(g.add_face(FaceAttr::with_pcurves(
        Surface::Ruled(RuledSurface::new(geometry.base_curve, geometry.direction)),
        P::F::default(),
        loop_dart,
        Vec::new(),
        pcurves,
    )))
}

fn validate_distance(distance: f64) -> Result<(), ChamferError> {
    if distance.is_finite() && distance > 0.0 {
        Ok(())
    } else {
        Err(ChamferError::InvalidDistance { distance })
    }
}

/// Resolves the two alpha1-linked dart occurrences of a profile vertex and
/// returns them in incoming-then-outgoing order.
fn profile_corner_darts<P: Payload>(
    g: &GMap<P>,
    vertex_dart: Dart,
) -> Result<(Dart, Dart), ChamferError> {
    let linked = g.alpha(Dim::One, vertex_dart);
    if linked == vertex_dart {
        return Err(ChamferError::EndpointVertex { dart: vertex_dart });
    }

    let role = corner_role(g, vertex_dart)?;
    let linked_role = corner_role(g, linked)?;
    match (role, linked_role) {
        (CornerRole::IncomingEnd, CornerRole::OutgoingStart) => Ok((vertex_dart, linked)),
        (CornerRole::OutgoingStart, CornerRole::IncomingEnd) => Ok((linked, vertex_dart)),
        _ => Err(ChamferError::AmbiguousProfileVertex { dart: vertex_dart }),
    }
}

/// Classifies a profile vertex occurrence from its position on the directed
/// edge attribute.
fn corner_role<P: Payload>(g: &GMap<P>, dart: Dart) -> Result<CornerRole, ChamferError> {
    let edge_dart = line_edge_dart(g, dart)?;
    if edge_dart == dart {
        Ok(CornerRole::OutgoingStart)
    } else if g.alpha(Dim::Zero, edge_dart) == dart {
        Ok(CornerRole::IncomingEnd)
    } else {
        Err(ChamferError::AmbiguousProfileVertex { dart })
    }
}

/// Returns the stored orientation dart when `dart` belongs to a straight edge.
fn line_edge_dart<P: Payload>(g: &GMap<P>, dart: Dart) -> Result<Dart, ChamferError> {
    let attr = g
        .attribute::<Cell1>(dart)
        .ok_or(ChamferError::MissingEdgeCurve { dart })?;
    match &attr.curve {
        Curve::Line(_) => Ok(attr.dart),
        Curve::Bounded(curve) if matches!(curve.inner(), Curve::Line(_)) => Ok(attr.dart),
        _ => Err(ChamferError::UnsupportedEdgeCurve { dart: attr.dart }),
    }
}

/// Moves from a vertex toward its neighbor by the requested edge distance.
fn offset_point(
    edge_dart: Dart,
    vertex: Point3,
    neighbor: Point3,
    distance: f64,
) -> Result<Point3, ChamferError> {
    let direction = neighbor - vertex;
    let edge_length = direction.norm();
    if edge_length <= LINEAR_TOLERANCE {
        return Err(ChamferError::ZeroLengthEdge { dart: edge_dart });
    }
    if distance >= edge_length - LINEAR_TOLERANCE {
        return Err(ChamferError::DistanceTooLarge {
            dart: edge_dart,
            distance,
            edge_length,
        });
    }

    Ok(vertex + direction / edge_length * distance)
}

/// Rebuilds a line curve after one of its endpoint vertex positions changed.
fn reset_line_edge<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    dart: Dart,
) -> Result<(), ChamferError> {
    let edge_dart = line_edge_dart(g, dart)?;
    let start = vertex_point(g, edge_dart)?;
    let end = vertex_point(g, g.alpha(Dim::Zero, edge_dart))?;
    let edge_key = g
        .cell_key::<Cell1>(edge_dart)
        .ok_or(ChamferError::MissingEdgeCurve { dart: edge_dart })?;
    let attr = g
        .edge_attr_mut(edge_key)
        .expect("validated chamfer edge must remain registered");
    attr.curve = Curve::line(start, end);
    Ok(())
}

fn vertex_point<P: Payload>(g: &GMap<P>, dart: Dart) -> Result<Point3, ChamferError> {
    g.attribute::<Cell0>(dart)
        .map(|attr| attr.point)
        .ok_or(ChamferError::MissingVertexPoint { dart })
}

/// Alpha1-sews two standalone profile edge-end occurrences.
fn sew<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    first: Dart,
    second: Dart,
) -> Result<(), ChamferError> {
    g.sew(Dim::One, first, second)
        .map_err(|_| ChamferError::SewFailed {
            dim: Dim::One,
            first,
            second,
        })
}
