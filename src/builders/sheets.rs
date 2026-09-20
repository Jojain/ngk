use crate::geometry::TrimmedCurve2;
use crate::geometry::parameter::Fraction;
use std::collections::HashMap;

use nalgebra::Vector3;

use crate::builders::errors::ExtrudeError;
use crate::geometry::{
    Curve, LINEAR_TOLERANCE, Plane, Point2, Point3, Rigid, RuledSurface, Surface,
};
use crate::model::Model;
use crate::topology::ModelEdit;
use crate::topology::attributes::{EdgeAttr, FaceAttr, ProfileAttr, SheetAttr, VertexAttr};
use crate::topology::closed::Closeable;
use crate::topology::edge::Edge;
use crate::topology::gmap::{Dart, Dim};
use crate::topology::payload::{DefaultPayload, Payload};
use crate::topology::shape_keys::{EdgeKey, ProfileKey, SheetKey, VertexKey};
use crate::topology::vertex::Vertex;

/// Adds an extruded profile to the given model.
///
/// Returns the generated sheet key. Its stored dart belongs to the translated
/// copy of the input edge.
///
/// # Panics
///
/// Panics if `profile_key` does not identify a registered profile.
pub fn add_extruded_profile<P: DefaultPayload>(
    g: &mut Model<P>,
    profile_key: ProfileKey,
    direction: Vector3<f64>,
) -> Result<SheetKey, ExtrudeError> {
    g.transaction(|edit| {
        if direction.norm_squared() <= LINEAR_TOLERANCE * LINEAR_TOLERANCE {
            return Err(ExtrudeError::ZeroDirection);
        }

        let profile = edit.profile_unchecked(profile_key);
        let profile_dart = profile.dart;
        let is_closed = profile.is_closed();
        let edge_darts = profile
            .edges()
            .into_iter()
            .map(|edge| edge.dart())
            .collect::<Vec<_>>();
        let mut faces = Vec::with_capacity(edge_darts.len());
        let mut translated_dart = None;

        for edge_dart in edge_darts {
            let extruded_face = extrude_edge(edit, edge_dart, direction)?;
            if edge_dart == profile_dart {
                translated_dart = Some(extruded_face.translated_start);
            } else if edit.alpha(Dim::Zero, edge_dart) == profile_dart {
                translated_dart = Some(extruded_face.translated_end);
            }
            faces.push(extruded_face);
        }

        sew_extruded_faces(edit, &faces, is_closed)?;
        let translated_dart =
            translated_dart.expect("profile dart must belong to one of its profile edges");

        Ok(edit.add_sheet(SheetAttr::new(translated_dart)))
    })
}

fn extrude_edge<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    edge_dart: Dart,
    direction: Vector3<f64>,
) -> Result<ExtrudedFace, ExtrudeError> {
    let edge = Edge::from_dart(edit, edge_dart)
        .ok_or(ExtrudeError::MissingEdgeCurve { dart: edge_dart })?;
    // The ends of the section being swept. A closed edge sweeps a wall whose two
    // ends are the same point, which is exactly the wrapping case.
    let section = edge.trimmed_curve();
    let (start, end) = (
        section.point_at(Fraction::new(0.0)),
        section.point_at(Fraction::new(1.0)),
    );
    let curve = edge.curve();

    let corners = [start, end, end + direction, start + direction];
    let surface_data = extruded_edge_surface(edge.dart(), curve, start, end, direction)?;
    add_extruded_edge_face(edit, corners, surface_data)
}

fn sew_extruded_faces<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    faces: &[ExtrudedFace],
    close_ring: bool,
) -> Result<(), ExtrudeError> {
    for i in 0..faces.len().saturating_sub(1) {
        sew_adjacent_sweep_edges(edit, faces[i].end_side, faces[i + 1].start_side)?;
    }

    if close_ring && !faces.is_empty() {
        sew_adjacent_sweep_edges(edit, faces[faces.len() - 1].end_side, faces[0].start_side)?;
    }

    Ok(())
}

struct ExtrudedFace {
    start_side: Dart,
    end_side: Dart,
    translated_start: Dart,
    translated_end: Dart,
}

struct ExtrudedSurface {
    surface: Surface,
    uv: [Point2; 4],
    boundary_curves: [Curve; 4],
}

fn extruded_edge_surface(
    dart: Dart,
    curve: &Curve,
    start: Point3,
    end: Point3,
    direction: Vector3<f64>,
) -> Result<ExtrudedSurface, ExtrudeError> {
    match curve {
        Curve::Line(_) => {
            let surface = lateral_plane(dart, start, end, direction)?;
            let translated_curve = curve.moved(&Rigid::translation(direction));
            let uv = [
                plane_uv(&surface, start),
                plane_uv(&surface, end),
                plane_uv(&surface, end + direction),
                plane_uv(&surface, start + direction),
            ];
            Ok(ExtrudedSurface {
                surface: Surface::Plane(surface),
                uv,
                boundary_curves: [
                    curve.clone(),
                    Curve::line(end, end + direction),
                    translated_curve,
                    Curve::line(start + direction, start),
                ],
            })
        }
        Curve::Circle(_) | Curve::Ellipse(_) | Curve::Nurbs(_) => {
            let interval = curve.interval_between(start, end);
            let translated_curve = curve.moved(&Rigid::translation(direction));
            Ok(ExtrudedSurface {
                surface: Surface::Ruled(RuledSurface::new(curve.clone(), direction)),
                uv: [
                    Point2::new(interval.start.value(), 0.0),
                    Point2::new(interval.end.value(), 0.0),
                    Point2::new(interval.end.value(), 1.0),
                    Point2::new(interval.start.value(), 1.0),
                ],
                boundary_curves: [
                    curve.clone(),
                    Curve::line(end, end + direction),
                    translated_curve,
                    Curve::line(start + direction, start),
                ],
            })
        }
    }
}

fn add_extruded_edge_face<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    corners: [Point3; 4],
    surface_data: ExtrudedSurface,
) -> Result<ExtrudedFace, ExtrudeError> {
    let darts: Vec<Dart> = (0..8).map(|_| edit.add_dart()).collect();

    for i in 0..4 {
        sew(edit, Dim::Zero, darts[2 * i], darts[2 * i + 1])?;
    }
    for i in 0..4 {
        sew(
            edit,
            Dim::One,
            darts[2 * i + 1],
            darts[(2 * i + 2) % darts.len()],
        )?;
    }

    for i in 0..4 {
        let dart = edit.cell_representative(darts[2 * i], Dim::Zero);
        edit.add_vertex(VertexAttr::new(dart, corners[i]));
    }

    for i in 0..4 {
        let edge_dart = darts[2 * i];
        edit.add_edge(EdgeAttr::new(
            edge_dart,
            surface_data.boundary_curves[i].clone(),
        ));
    }

    edit.add_profile(ProfileAttr::new(darts[0]));
    edit.add_face(FaceAttr::with_pcurves(
        surface_data.surface,
        darts[0],
        Vec::new(),
        quad_pcurves(&surface_data.uv, &darts),
    ));

    Ok(ExtrudedFace {
        start_side: darts[7],
        end_side: darts[2],
        translated_start: darts[5],
        translated_end: darts[4],
    })
}

fn sew_adjacent_sweep_edges<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    survivor: Dart,
    removed: Dart,
) -> Result<(), ExtrudeError> {
    let merge = alpha2_sweep_merge(edit, survivor, removed)?;
    edit.sew(Dim::Two, survivor, removed)
        .map_err(|_| ExtrudeError::SewFailed {
            dim: Dim::Two,
            first: survivor,
            second: removed,
        })?;
    if merge.survivor_edge != merge.removed_edge {
        edit.merge_edges_into(merge.survivor_edge, merge.removed_edge);
    }
    if merge.survivor_start != merge.removed_start {
        edit.merge_vertices_into(merge.survivor_start, merge.removed_start);
    }
    if merge.survivor_end != merge.removed_end {
        edit.merge_vertices_into(merge.survivor_end, merge.removed_end);
    }
    Ok(())
}

struct Alpha2SweepMerge {
    survivor_edge: EdgeKey,
    removed_edge: EdgeKey,
    survivor_start: VertexKey,
    removed_start: VertexKey,
    survivor_end: VertexKey,
    removed_end: VertexKey,
}

fn alpha2_sweep_merge<P: Payload>(
    g: &Model<P>,
    survivor: Dart,
    removed: Dart,
) -> Result<Alpha2SweepMerge, ExtrudeError> {
    let survivor_edge =
        Edge::from_dart(g, survivor).ok_or(ExtrudeError::MissingEdgeCurve { dart: survivor })?;
    let removed_edge =
        Edge::from_dart(g, removed).ok_or(ExtrudeError::MissingEdgeCurve { dart: removed })?;

    // A dart-level lookup, not an endpoint one: sewing two closed circles gives
    // the same vertex at both ends of each, and that is what has to reconcile.
    let vertex_at = |dart: Dart| {
        Vertex::from_dart(g, dart)
            .map(|vertex| vertex.key())
            .ok_or(ExtrudeError::MissingVertexPoint { dart })
    };
    Ok(Alpha2SweepMerge {
        survivor_edge: survivor_edge.key(),
        removed_edge: removed_edge.key(),
        survivor_start: vertex_at(survivor)?,
        removed_start: vertex_at(removed)?,
        survivor_end: vertex_at(g.alpha(Dim::Zero, survivor))?,
        removed_end: vertex_at(g.alpha(Dim::Zero, removed))?,
    })
}

fn sew<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    dim: Dim,
    first: Dart,
    second: Dart,
) -> Result<(), ExtrudeError> {
    edit.sew(dim, first, second)
        .map_err(|_| ExtrudeError::SewFailed { dim, first, second })
}

fn lateral_plane(
    dart: Dart,
    start: Point3,
    end: Point3,
    direction: Vector3<f64>,
) -> Result<Plane, ExtrudeError> {
    let edge = end - start;
    if edge.norm_squared() <= LINEAR_TOLERANCE * LINEAR_TOLERANCE {
        return Err(ExtrudeError::ZeroLengthEdge { dart });
    }
    if edge.cross(&direction).norm_squared() <= LINEAR_TOLERANCE * LINEAR_TOLERANCE {
        return Err(ExtrudeError::DegenerateSweep { dart });
    }
    Ok(Plane::from_xy(start, edge, direction))
}

fn quad_pcurves(uv: &[Point2; 4], darts: &[Dart]) -> HashMap<Dart, TrimmedCurve2> {
    let mut pcurves = HashMap::with_capacity(4);
    for i in 0..4 {
        pcurves.insert(
            darts[2 * i],
            TrimmedCurve2::segment(uv[i], uv[(i + 1) % uv.len()]),
        );
    }
    pcurves
}

fn plane_uv(surface: &Plane, point: Point3) -> Point2 {
    let v = point - surface.origin();
    Point2::new(v.dot(&surface.x_dir()), v.dot(&surface.y_dir()))
}

#[cfg(test)]
mod tests {
    use std::collections::VecDeque;

    use nalgebra::Vector3;

    use crate::builders::faces::add_polygon;
    use crate::builders::sheets::add_extruded_profile;
    use crate::geometry::{LINEAR_TOLERANCE, Point3, PointCoincidence};
    use crate::model::{Cell0, Model};
    use crate::modeling::sweep::extrude_profile;
    use crate::tessellate::{TessellateOpts, face::tessellate_face_key};
    use crate::topology::StandardPayload;
    use crate::topology::edge::Edge;
    use crate::topology::gmap::{Dart, Dim};
    #[test]
    fn extrude_closed_profile_builds_one_lateral_face_per_edge() {
        let mut source = Model::<StandardPayload>::new();
        let profile_key = add_polygon(
            &mut source,
            &[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
        );

        let shape = extrude_profile(
            source.profile_unchecked(profile_key),
            Vector3::new(0.0, 0.0, 2.0),
        )
        .unwrap();
        let (g, sheet_key) = shape.into_model();

        assert!(g.sheet(sheet_key).is_some());
        assert_eq!(g.iter_faces().count(), 4);
        assert_eq!(g.iter_edges().count(), 16);
        assert_eq!(g.iter_vertices().count(), 12);

        for (face, attr) in g.iter_faces() {
            assert_eq!(attr.pcurves.len(), 4);
            let mesh = tessellate_face_key(&g, face, TessellateOpts::default())
                .expect("extruded face should tessellate");
            assert!(!mesh.positions.is_empty());
            assert!(!mesh.indices.is_empty());
        }
    }

    #[test]
    fn add_extruded_profile_key_uses_translated_edge_as_default_dart() {
        let mut source = Model::<StandardPayload>::new();
        let profile_key = add_polygon(
            &mut source,
            &[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
            ],
        );
        let source_dart_count = source.dart_count();
        let direction = Vector3::new(0.0, 0.0, 2.0);

        let sheet_key = add_extruded_profile(&mut source, profile_key, direction).unwrap();
        let translated_dart = source.sheet_attr_unchecked(sheet_key).dart();

        assert!(
            translated_dart.id() >= source_dart_count,
            "returned dart should belong to generated extrusion topology"
        );
        let translated_edge = Edge::from_dart(&source, translated_dart)
            .expect("translated dart should belong to an edge");
        let (translated_start, translated_end) = translated_edge.bounded_unchecked().vertices();
        let start = *translated_start.point();
        let end = *translated_end.point();

        assert!((start.z - 2.0).abs() <= LINEAR_TOLERANCE);
        assert!((end.z - 2.0).abs() <= LINEAR_TOLERANCE);
    }

    #[test]
    fn extrude_closed_square_preserves_gmap_and_corner_connectivity() {
        let mut source = Model::<StandardPayload>::new();
        let profile_key = add_polygon(
            &mut source,
            &[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
        );

        let shape = extrude_profile(
            source.profile_unchecked(profile_key),
            Vector3::new(0.0, 0.0, 2.0),
        )
        .unwrap();
        let sheet = shape.sheet();
        assert_eq!(sheet.darts().count(), 32);
        let g = shape.model();

        assert_valid_gmap(g);
        assert_orientable_gmap(g);
        assert_square_sweep_alpha2_seams_are_not_twisted(g);
        assert_alpha1_links_shared_corners(g);
        assert_alpha2_links_matching_edges(g);

        for (_, face) in g.iter_faces() {
            let loop_darts = g
                .orbit(
                    face.outer_unchecked(),
                    vec![Dim::Zero.index(), Dim::One.index()],
                )
                .collect::<Vec<_>>();
            assert_eq!(
                loop_darts.len(),
                8,
                "each extruded square side should be a quad face with 8 darts"
            );
        }
    }

    fn assert_square_sweep_alpha2_seams_are_not_twisted(g: &Model<StandardPayload>) {
        let expected_pairs = [
            (10, 23),
            (11, 22),
            (18, 31),
            (19, 30),
            (26, 39),
            (27, 38),
            (34, 15),
            (35, 14),
        ];

        for (first, second) in expected_pairs {
            let first = Dart::new(first);
            let second = Dart::new(second);
            assert_eq!(
                g.alpha(Dim::Two, first),
                second,
                "sweep alpha2 seam should preserve vertex side for {first:?}"
            );
            assert_eq!(
                g.alpha(Dim::Two, second),
                first,
                "sweep alpha2 seam should be symmetric for {second:?}"
            );
        }
    }

    fn assert_orientable_gmap(g: &Model<StandardPayload>) {
        let mut colors = vec![None; g.dart_count()];

        for start in g.darts() {
            if colors[start.id()].is_some() {
                continue;
            }

            colors[start.id()] = Some(false);
            let mut queue = VecDeque::from([start]);

            while let Some(dart) = queue.pop_front() {
                let color = colors[dart.id()].expect("queued darts should be colored");
                for i in 0..g.dimension() {
                    let dim = Dim::from_index(i);
                    let linked = g.alpha(dim, dart);
                    if linked == dart {
                        continue;
                    }

                    let expected = !color;
                    match colors[linked.id()] {
                        Some(actual) => assert_eq!(
                            actual, expected,
                            "orientability violation: alpha{i} links same-orientation darts {dart:?} and {linked:?}"
                        ),
                        None => {
                            colors[linked.id()] = Some(expected);
                            queue.push_back(linked);
                        }
                    }
                }
            }
        }
    }

    fn assert_valid_gmap(g: &Model<StandardPayload>) {
        for dart in g.darts() {
            for i in 0..g.dimension() {
                let dim = Dim::from_index(i);
                let linked = g.alpha(dim, dart);
                assert!(
                    linked.id() < g.dart_count(),
                    "alpha{i}({dart:?}) points outside the dart set: {linked:?}"
                );
                assert_eq!(
                    g.alpha(dim, linked),
                    dart,
                    "alpha{i} must be an involution at dart {dart:?}"
                );
            }

            for i in 0..g.dimension() {
                for j in i + 2..g.dimension() {
                    let dim_i = Dim::from_index(i);
                    let dim_j = Dim::from_index(j);
                    let twice =
                        g.alpha(dim_i, g.alpha(dim_j, g.alpha(dim_i, g.alpha(dim_j, dart))));
                    assert_eq!(
                        twice, dart,
                        "alpha{i} o alpha{j} must be an involution at dart {dart:?}"
                    );
                }
            }
        }
    }

    fn assert_alpha1_links_shared_corners(g: &Model<StandardPayload>) {
        for id in 0..g.dart_count() {
            let dart = Dart::new(id);
            let linked = g.alpha(Dim::One, dart);
            if linked == dart {
                continue;
            }
            let p0 = vertex_point(g, dart);
            let p1 = vertex_point(g, linked);
            assert!(
                p0.coincides(p1, LINEAR_TOLERANCE),
                "alpha1 should connect darts with the same corner point: {dart:?} at {p0:?}, {linked:?} at {p1:?}"
            );
        }
    }

    fn assert_alpha2_links_matching_edges(g: &Model<StandardPayload>) {
        for id in 0..g.dart_count() {
            let dart = Dart::new(id);
            let linked = g.alpha(Dim::Two, dart);
            if linked == dart {
                continue;
            }
            let edge = edge_points(g, dart);
            let linked_edge = edge_points(g, linked);
            assert!(
                same_undirected_edge(edge, linked_edge),
                "alpha2 should sew matching geometric edges: {dart:?} {edge:?}, {linked:?} {linked_edge:?}"
            );
        }
    }

    fn edge_points(g: &Model<StandardPayload>, dart: Dart) -> (Point3, Point3) {
        (
            vertex_point(g, dart),
            vertex_point(g, g.alpha(Dim::Zero, dart)),
        )
    }

    fn vertex_point(g: &Model<StandardPayload>, dart: Dart) -> Point3 {
        g.attribute_unchecked::<Cell0>(dart).point
    }

    fn same_undirected_edge(a: (Point3, Point3), b: (Point3, Point3)) -> bool {
        (a.0.coincides(b.0, LINEAR_TOLERANCE) && a.1.coincides(b.1, LINEAR_TOLERANCE))
            || (a.0.coincides(b.1, LINEAR_TOLERANCE) && a.1.coincides(b.0, LINEAR_TOLERANCE))
    }
}
