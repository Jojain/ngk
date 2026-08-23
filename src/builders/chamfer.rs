use std::collections::{HashMap, HashSet};

use crate::builders::edges::add_edge_staged;
use crate::builders::errors::ChamferError;
use crate::builders::faces::{
    FaceImprint, add_face_staged, add_polygon_staged, split_face_by_imprints_staged,
};
use crate::geometry::{Curve, Curve2, LINEAR_TOLERANCE, Line2, Point3, Surface};
use crate::topology::attributes::VertexAttr;
use crate::topology::gmap::{Cell0, Cell1, Dart, Dim, GMap};
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, FaceKey, ProfileKey, VertexKey};
use crate::topology::{IsolatedDart, TopologyEdit};

#[derive(Clone, Copy, PartialEq, Eq)]
enum CornerRole {
    IncomingEnd,
    OutgoingStart,
}

/// Chamfers a topology selection in place and returns no created handles.
///
/// A profile-corner dart mutates a line-only 2D profile. An edge, vertex, or
/// collection of either trims an existing solid shell and sews in new chamfer
/// faces. Passing a profile chamfers all of its corners when it is standalone,
/// or its boundary edges when it belongs to a solid face.
///
/// The initial solid implementation supports straight edges, planar faces, and
/// manifold trihedral corners. The complete edit is transactional: any invalid
/// target, distance, trim, or sew restores the original map.
pub fn chamfer<P: Payload, T: Into<ChamferTarget>>(
    g: &mut GMap<P>,
    target: T,
    distance: f64,
) -> Result<(), ChamferError> {
    g.transaction(|g| {
        validate_distance(distance)?;

        match target.into() {
            ChamferTarget::ProfileVertex(vertex_dart) => {
                chamfer_profile_corner(g, vertex_dart, distance)
            }
            ChamferTarget::Edge(edge) => chamfer_solid_edge(g, edge, distance),
            ChamferTarget::Edges(edges) => {
                for edge in edges {
                    chamfer_solid_edge(g, edge, distance)?;
                }
                Ok(())
            }
            ChamferTarget::Profile(profile) => chamfer_profile(g, profile, distance),
            ChamferTarget::Vertex(vertex) => chamfer_solid_vertex(g, vertex, distance),
            ChamferTarget::Vertices(vertices) => {
                for vertex in vertices {
                    chamfer_solid_vertex(g, vertex, distance)?;
                }
                Ok(())
            }
        }
    })
}

fn chamfer_profile_corner<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    vertex_dart: Dart,
    distance: f64,
) -> Result<(), ChamferError> {
    let (incoming_end, outgoing_start) = profile_corner_darts(g, vertex_dart)?;
    let vertex = vertex_point(g, incoming_end)?;
    let previous = vertex_point(g, g.alpha(Dim::Zero, incoming_end))?;
    let next = vertex_point(g, g.alpha(Dim::Zero, outgoing_start))?;

    let incoming_edge = line_edge_dart(g, incoming_end)?;
    let outgoing_edge = line_edge_dart(g, outgoing_start)?;
    let incoming_offset = offset_point(incoming_edge, vertex, previous, distance)?;
    let outgoing_offset = offset_point(outgoing_edge, vertex, next, distance)?;
    let corner_key = g
        .cell_key::<Cell0>(incoming_end)
        .ok_or(ChamferError::MissingVertexPoint { dart: incoming_end })?;

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
    /// One corner occurrence in a standalone profile.
    ProfileVertex(Dart),
    /// One solid boundary edge.
    Edge(EdgeKey),
    /// Several solid boundary edges.
    Edges(Vec<EdgeKey>),
    /// Every edge referenced by a profile.
    Profile(ProfileKey),
    /// One solid boundary vertex.
    Vertex(VertexKey),
    /// Several solid boundary vertices.
    Vertices(Vec<VertexKey>),
}

impl From<Dart> for ChamferTarget {
    fn from(value: Dart) -> Self {
        Self::ProfileVertex(value)
    }
}

impl From<EdgeKey> for ChamferTarget {
    fn from(value: EdgeKey) -> Self {
        Self::Edge(value)
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
        Self::Vertex(value)
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

struct SolidEdgeChamfer {
    incident_faces: [FaceKey; 2],
    endpoint_faces: [FaceKey; 2],
    endpoints: [VertexKey; 2],
    face_offsets: [[Point3; 2]; 2],
}

fn chamfer_solid_edge<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    edge: EdgeKey,
    distance: f64,
) -> Result<(), ChamferError> {
    let prepared = prepare_solid_edge_chamfer(g, edge, distance)?;
    let mut patch_faces = HashSet::new();
    let mut section_edges = Vec::with_capacity(4);

    for face_index in 0..2 {
        let face = prepared.incident_faces[face_index];
        let points = prepared.face_offsets[face_index];
        let (patch, section) = split_chamfer_face(g, edge, face, points, |g, candidate| {
            face_contains_edge(g, candidate, edge)
        })?;
        patch_faces.insert(patch);
        section_edges.push(section);
    }

    for endpoint_index in 0..2 {
        let face = prepared.endpoint_faces[endpoint_index];
        let points = [
            prepared.face_offsets[0][endpoint_index],
            prepared.face_offsets[1][endpoint_index],
        ];
        let endpoint = prepared.endpoints[endpoint_index];
        let (patch, section) = split_chamfer_face(g, edge, face, points, |g, candidate| {
            face_contains_vertex(g, candidate, endpoint)
        })?;
        patch_faces.insert(patch);
        section_edges.push(section);
    }

    let mut corners = [
        prepared.face_offsets[0][0],
        prepared.face_offsets[0][1],
        prepared.face_offsets[1][1],
        prepared.face_offsets[1][0],
    ];
    orient_chamfer_corners(g, prepared.incident_faces, &mut corners);
    replace_face_patch(g, &patch_faces, &section_edges, &corners)
}

fn prepare_solid_edge_chamfer<P: Payload>(
    g: &GMap<P>,
    edge: EdgeKey,
    distance: f64,
) -> Result<SolidEdgeChamfer, ChamferError> {
    let edge_view = g
        .edge(edge)
        .ok_or(ChamferError::MissingChamferEdge { edge })?;
    if !is_linear_curve(edge_view.curve()) {
        return Err(ChamferError::UnsupportedSolidChamferGeometry { edge });
    }
    let faces = edge_view.faces();
    if faces.len() != 2 {
        return Err(ChamferError::InvalidChamferEdgeIncidence {
            edge,
            count: faces.len(),
        });
    }
    if faces
        .iter()
        .any(|face| !matches!(face.surface(), Surface::Plane(_)))
    {
        return Err(ChamferError::UnsupportedSolidChamferGeometry { edge });
    }

    let endpoints = [edge_view.start().key(), edge_view.end().key()];
    let incident_faces = [faces[0].key(), faces[1].key()];
    let mut endpoint_faces = [faces[0].key(); 2];
    let mut face_offsets = [[Point3::origin(); 2]; 2];

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
            let endpoint_point = *vertex
                .point()
                .ok_or(ChamferError::MissingVertexPoint { dart: vertex.dart })?;
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

    Ok(SolidEdgeChamfer {
        incident_faces,
        endpoint_faces,
        endpoints,
        face_offsets,
    })
}

fn split_chamfer_face<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    edge: EdgeKey,
    face: FaceKey,
    points: [Point3; 2],
    is_patch: impl Fn(&GMap<P>, FaceKey) -> bool,
) -> Result<(FaceKey, EdgeKey), ChamferError> {
    let plane = match g.face_attr(face).map(|attr| &attr.surface) {
        Some(Surface::Plane(plane)) => plane,
        _ => return Err(ChamferError::UnsupportedSolidChamferGeometry { edge }),
    };
    let imprint = FaceImprint::new(
        Curve::line(points[0], points[1]),
        Curve2::Line(Line2::new(
            plane.parameter_at(points[0]),
            plane.parameter_at(points[1]),
        )),
    );
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

fn orient_chamfer_corners<P: Payload>(
    g: &GMap<P>,
    incident_faces: [FaceKey; 2],
    corners: &mut [Point3; 4],
) {
    let edge_direction = corners[1] - corners[0];
    let across = corners[3] - corners[0];
    let candidate_normal = edge_direction.cross(&across);
    let outward = incident_faces
        .into_iter()
        .filter_map(|face| g.face(face))
        .map(|face| *face.normal_at(0.0, 0.0))
        .sum::<nalgebra::Vector3<f64>>();
    if candidate_normal.dot(&outward) < 0.0 {
        corners.reverse();
    }
}

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
        let edge_keys = edges.iter().map(|edge| edge.key()).collect::<Vec<_>>();
        for edge in edge_keys {
            chamfer_solid_edge(g, edge, distance)?;
        }
        return Ok(());
    }

    let closed = edges
        .first()
        .zip(edges.last())
        .is_some_and(|(first, last)| first.start().key() == last.end().key());
    let corner_darts = edges
        .iter()
        .take(edges.len().saturating_sub((!closed) as usize))
        .map(|edge| edge.end().dart)
        .collect::<Vec<_>>();
    for dart in corner_darts {
        chamfer_profile_corner(g, dart, distance)?;
    }
    Ok(())
}

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
        let (patch, section) =
            split_chamfer_face(g, reference_edge, face, points, |g, candidate| {
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
    replace_face_patch(g, &patch_faces, &section_edges, &corners)
}

fn is_linear_curve(curve: Option<&Curve>) -> bool {
    match curve {
        Some(Curve::Line(_)) => true,
        Some(Curve::Bounded(curve)) => matches!(curve.inner(), Curve::Line(_)),
        _ => false,
    }
}

fn replace_face_patch<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    patch_faces: &HashSet<FaceKey>,
    section_edges: &[EdgeKey],
    corners: &[Point3],
) -> Result<(), ChamferError> {
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

    let profile = add_polygon_staged(g, corners);
    let face = add_face_staged(g, profile).map_err(|_| ChamferError::UnsupportedChamferTarget)?;
    let new_edges = g.face_unchecked(face).edges();
    let sew_pairs = boundary_darts
        .into_iter()
        .map(|boundary| {
            let boundary_start = vertex_point(g, boundary)?;
            let boundary_end = vertex_point(g, g.alpha(Dim::Zero, boundary))?;
            let edge = new_edges
                .iter()
                .find(|edge| {
                    let start = edge.start().point().copied();
                    let end = edge.end().point().copied();
                    matches!((start, end), (Some(start), Some(end)) if
                        ((start - boundary_start).norm() <= LINEAR_TOLERANCE
                            && (end - boundary_end).norm() <= LINEAR_TOLERANCE)
                        || ((start - boundary_end).norm() <= LINEAR_TOLERANCE
                            && (end - boundary_start).norm() <= LINEAR_TOLERANCE))
                })
                .ok_or(ChamferError::UnsupportedChamferTarget)?;
            let dart = if (*edge.start().point().expect("polygon edge has geometry")
                - boundary_start)
                .norm()
                <= LINEAR_TOLERANCE
            {
                edge.dart()
            } else {
                g.alpha(Dim::Zero, edge.dart())
            };
            Ok((dart, boundary))
        })
        .collect::<Result<Vec<_>, ChamferError>>()?;
    for (new_edge, boundary) in sew_pairs {
        g.sew(Dim::Two, new_edge, boundary)
            .map_err(|_| ChamferError::SewFailed {
                dim: Dim::Two,
                first: new_edge,
                second: boundary,
            })?;
    }
    Ok(())
}

fn validate_distance(distance: f64) -> Result<(), ChamferError> {
    if distance.is_finite() && distance > 0.0 {
        Ok(())
    } else {
        Err(ChamferError::InvalidDistance { distance })
    }
}

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
