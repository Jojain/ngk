use std::collections::HashMap;

use nalgebra::Vector3;

use crate::geometry::{
    Curve, Curve2, Line, Line2, NurbsError, Plane, Point2, Point3, RuledSurface, Surface,
};
use crate::topology::StandardPayload;
use crate::topology::attributes::{EdgeAttr, FaceAttr, VertexAttr};
use crate::topology::closed::Closeable;
use crate::topology::edge::Edge;
use crate::topology::gmap::{Dart, Dim, GMap};
use crate::topology::payload::Payload;
use crate::topology::profile::Profile;
use crate::topology::shape::Shape;
use crate::topology::shape_keys::FaceKey;

#[derive(Debug, Clone, PartialEq)]
pub enum ExtrudeError {
    EmptyProfile,
    MissingVertexPoint { dart: Dart },
    MissingEdgeCurve { dart: Dart },
    ZeroDirection,
    ZeroLengthEdge { dart: Dart },
    DegenerateSweep { dart: Dart },
    SewFailed { dim: Dim, first: Dart, second: Dart },
    CurveTranslationFailed { dart: Dart, source: NurbsError },
}


pub fn extrude<P: Payload>(
    profile: Profile<'_, P>,
    direction: Vector3<f64>,
) -> Result<Shape<FaceKey, P>, ExtrudeError> {
    if direction.norm_squared() <= f64::EPSILON {
        return Err(ExtrudeError::ZeroDirection);
    }

    let mut g = GMap::<P>::new();
    let mut faces = Vec::new();

    for edge in profile.edges() {
        faces.push(extrude_edge(&mut g, &edge, direction)?);
    }

    sew_extruded_faces(&mut g, &faces, profile.is_closed())?;

    let first_face = faces
        .first()
        .map(|face| face.key)
        .ok_or(ExtrudeError::EmptyProfile)?;

    Ok(Shape::new(g, first_face))
}

fn extrude_edge<P: Payload>(
    g: &mut GMap<P>,
    edge: &Edge<'_, P>,
    direction: Vector3<f64>,
) -> Result<ExtrudedFace, ExtrudeError> {
    let start = *edge
        .start()
        .point()
        .ok_or(ExtrudeError::MissingVertexPoint { dart: edge.dart })?;
    let end = *edge.end().point().ok_or(ExtrudeError::MissingVertexPoint {
        dart: edge.end().dart,
    })?;
    let curve = edge
        .curve()
        .ok_or(ExtrudeError::MissingEdgeCurve { dart: edge.dart })?;

    let corners = [start, end, end + direction, start + direction];
    let surface_data = extruded_edge_surface(edge.dart, curve, start, end, direction)?;
    add_extruded_edge_face(g, corners, surface_data)
}

fn sew_extruded_faces<P: Payload>(
    g: &mut GMap<P>,
    faces: &[ExtrudedFace],
    close_ring: bool,
) -> Result<(), ExtrudeError> {
    for i in 0..faces.len().saturating_sub(1) {
        sew_adjacent_sweep_edges(g, faces[i].end_side, faces[i + 1].start_side)?;
    }

    if close_ring && faces.len() > 1 {
        sew_adjacent_sweep_edges(g, faces[faces.len() - 1].end_side, faces[0].start_side)?;
    }

    Ok(())
}

struct ExtrudedFace {
    key: FaceKey,
    start_side: Dart,
    end_side: Dart,
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
            let translated_curve = curve
                .translated(direction)
                .map_err(|source| ExtrudeError::CurveTranslationFailed { dart, source })?;
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
                    Curve::Line(Line::new(end, end + direction)),
                    translated_curve,
                    Curve::Line(Line::new(start + direction, start)),
                ],
            })
        }
        Curve::Circle(_) | Curve::Nurbs(_) => {
            let (u0, u1) = curve_parameters(curve, start, end);
            let translated_curve = curve
                .translated(direction)
                .map_err(|source| ExtrudeError::CurveTranslationFailed { dart, source })?;
            Ok(ExtrudedSurface {
                surface: Surface::Ruled(RuledSurface::new(curve.clone(), direction)),
                uv: [
                    Point2::new(u0, 0.0),
                    Point2::new(u1, 0.0),
                    Point2::new(u1, 1.0),
                    Point2::new(u0, 1.0),
                ],
                boundary_curves: [
                    curve.clone(),
                    Curve::Line(Line::new(end, end + direction)),
                    translated_curve,
                    Curve::Line(Line::new(start + direction, start)),
                ],
            })
        }
    }
}

fn add_extruded_edge_face<P: Payload>(
    g: &mut GMap<P>,
    corners: [Point3; 4],
    surface_data: ExtrudedSurface,
) -> Result<ExtrudedFace, ExtrudeError> {
    let darts: Vec<Dart> = (0..8).map(|_| g.add_dart()).collect();

    for i in 0..4 {
        sew(g, Dim::Zero, darts[2 * i], darts[2 * i + 1])?;
    }
    for i in 0..4 {
        sew(
            g,
            Dim::One,
            darts[2 * i + 1],
            darts[(2 * i + 2) % darts.len()],
        )?;
    }

    for i in 0..4 {
        let dart = g.cell_representative(darts[2 * i], Dim::Zero);
        g.add_vertex(VertexAttr::new(dart, corners[i], P::V::default()));
    }

    for i in 0..4 {
        let edge_dart = g.cell_representative(darts[2 * i], Dim::One);
        g.add_edge(EdgeAttr::new(
            edge_dart,
            surface_data.boundary_curves[i].clone(),
            P::E::default(),
        ));
    }

    let key = g.add_face(FaceAttr::with_pcurves(
        surface_data.surface,
        P::F::default(),
        darts[0],
        Vec::new(),
        quad_pcurves(&surface_data.uv, &darts),
    ));

    Ok(ExtrudedFace {
        key,
        start_side: darts[7],
        end_side: darts[2],
    })
}

fn sew_adjacent_sweep_edges<P: Payload>(
    g: &mut GMap<P>,
    a: Dart,
    b: Dart,
) -> Result<(), ExtrudeError> {
    sew(g, Dim::Two, a, b)
}

fn sew<P: Payload>(
    g: &mut GMap<P>,
    dim: Dim,
    first: Dart,
    second: Dart,
) -> Result<(), ExtrudeError> {
    g.sew(dim, first, second)
        .map_err(|_| ExtrudeError::SewFailed { dim, first, second })
}

fn lateral_plane(
    dart: Dart,
    start: Point3,
    end: Point3,
    direction: Vector3<f64>,
) -> Result<Plane, ExtrudeError> {
    let edge = end - start;
    if edge.norm_squared() <= f64::EPSILON {
        return Err(ExtrudeError::ZeroLengthEdge { dart });
    }
    if edge.cross(&direction).norm_squared() <= f64::EPSILON {
        return Err(ExtrudeError::DegenerateSweep { dart });
    }
    Ok(Plane::from_xy(start, edge, direction))
}

fn curve_parameters(curve: &Curve, start: Point3, end: Point3) -> (f64, f64) {
    match curve {
        Curve::Line(_) | Curve::Circle(_) => (curve.param_at(start), curve.param_at(end)),
        Curve::Nurbs(nurbs) => nurbs.domain(),
    }
}

fn quad_pcurves(uv: &[Point2; 4], darts: &[Dart]) -> HashMap<Dart, Curve2> {
    let mut pcurves = HashMap::with_capacity(4);
    for i in 0..4 {
        pcurves.insert(
            darts[2 * i],
            Curve2::Line(Line2::new(uv[i], uv[(i + 1) % uv.len()])),
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

    use super::extrude;
    use crate::builders::add_polygon;
    use crate::geometry::Point3;
    use crate::tessellate::{TessellateOpts, face::tessellate_face};
    use crate::topology::StandardPayload;
    use crate::topology::gmap::{Cell0, Dart, Dim, GMap};
    use crate::topology::profile::Profile;

    #[test]
    fn extrude_closed_profile_builds_one_lateral_face_per_edge() {
        let mut source = GMap::<StandardPayload>::new();
        let loop_dart = add_polygon(
            &mut source,
            &[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
        );

        let shape = extrude(
            Profile::new(&source, loop_dart),
            Vector3::new(0.0, 0.0, 2.0),
        )
        .unwrap();
        let (g, first_face) = shape.into_map();

        assert!(g.face(first_face).is_some());
        assert_eq!(g.iter_faces().count(), 4);
        assert_eq!(g.iter_edges().count(), 16);
        assert_eq!(g.iter_vertices().count(), 16);

        for (face, attr) in g.iter_faces() {
            assert_eq!(attr.pcurves.len(), 4);
            let mesh = tessellate_face(&g, face, TessellateOpts::default())
                .expect("extruded face should tessellate");
            assert!(!mesh.positions.is_empty());
            assert!(!mesh.indices.is_empty());
        }
    }

    #[test]
    fn extrude_closed_square_preserves_gmap_and_corner_connectivity() {
        let mut source = GMap::<StandardPayload>::new();
        let loop_dart = add_polygon(
            &mut source,
            &[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
        );

        let shape = extrude(
            Profile::new(&source, loop_dart),
            Vector3::new(0.0, 0.0, 2.0),
        )
        .unwrap();
        let g= shape.map();

        assert_valid_gmap(&g);
        assert_orientable_gmap(&g);
        assert_square_sweep_alpha2_seams_are_not_twisted(&g);
        assert_alpha1_links_shared_corners(&g);
        assert_alpha2_links_matching_edges(&g);

        for (_, face) in g.iter_faces() {
            let loop_darts = g
                .orbit(face.outer_loop, vec![Dim::Zero.index(), Dim::One.index()])
                .collect::<Vec<_>>();
            assert_eq!(
                loop_darts.len(),
                8,
                "each extruded square side should be a quad face with 8 darts"
            );
        }
    }

    fn assert_square_sweep_alpha2_seams_are_not_twisted(g: &GMap<StandardPayload>) {
        let expected_pairs = [
            (2, 15),
            (3, 14),
            (10, 23),
            (11, 22),
            (18, 31),
            (19, 30),
            (26, 7),
            (27, 6),
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

    fn assert_orientable_gmap(g: &GMap<StandardPayload>) {
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

    fn assert_valid_gmap(g: &GMap<StandardPayload>) {
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
                    let twice = g.alpha(
                        dim_i,
                        g.alpha(dim_j, g.alpha(dim_i, g.alpha(dim_j, dart))),
                    );
                    assert_eq!(
                        twice, dart,
                        "alpha{i} o alpha{j} must be an involution at dart {dart:?}"
                    );
                }
            }
        }
    }

    fn assert_alpha1_links_shared_corners(g: &GMap<StandardPayload>) {
        for id in 0..g.dart_count() {
            let dart = Dart::new(id);
            let linked = g.alpha(Dim::One, dart);
            if linked == dart {
                continue;
            }
            let p0 = vertex_point(g, dart);
            let p1 = vertex_point(g, linked);
            assert!(
                same_point(p0, p1),
                "alpha1 should connect darts with the same corner point: {dart:?} at {p0:?}, {linked:?} at {p1:?}"
            );
        }
    }

    fn assert_alpha2_links_matching_edges(g: &GMap<StandardPayload>) {
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

    fn edge_points(g: &GMap<StandardPayload>, dart: Dart) -> (Point3, Point3) {
        (vertex_point(g, dart), vertex_point(g, g.alpha(Dim::Zero, dart)))
    }

    fn vertex_point(g: &GMap<StandardPayload>, dart: Dart) -> Point3 {
        g.attribute::<Cell0>(dart)
            .expect("test map should have vertex attributes on every dart")
            .point
    }

    fn same_undirected_edge(a: (Point3, Point3), b: (Point3, Point3)) -> bool {
        (same_point(a.0, b.0) && same_point(a.1, b.1))
            || (same_point(a.0, b.1) && same_point(a.1, b.0))
    }

    fn same_point(a: Point3, b: Point3) -> bool {
        (a - b).norm_squared() <= 1e-18
    }
}
