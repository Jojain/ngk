use nalgebra::Vector3;

use crate::{
    Payload,
    builders::errors::ExtrudeError,
    geometry::{
        Curve, Curve2, LINEAR_TOLERANCE, Line2, Plane, Point2, Point3, RuledSurface, Surface,
    },
    topology::{
        Dart, SolidAttr,
        attributes::{EdgeAttr, FacetAttr},
        edge::Edge,
        face::Face,
        gmap::{Dim, GMap, MergeTopology},
        profile::Profile,
        shape::{FaceTag, Shape},
        shape_keys::{FaceKey, SolidKey},
        sheet::Sheet,
    },
};

pub fn translate_face<P: Payload>(
    face: &Face<'_, P>,
    direction: Vector3<f64>,
) -> Result<Shape<FaceTag, P>, ExtrudeError> {
    if direction.norm_squared() <= LINEAR_TOLERANCE * LINEAR_TOLERANCE {
        return Err(ExtrudeError::ZeroDirection);
    }

    let (mut translated, translated_dart) = face.isolate();

    let vertex_keys = translated
        .iter_vertices()
        .map(|(key, _)| key)
        .collect::<Vec<_>>();
    for key in vertex_keys {
        let vertex = translated
            .vertex_attr_mut(key)
            .expect("collected vertex key must remain valid");
        vertex.point += direction;
    }

    let edge_keys = translated
        .iter_edges()
        .map(|(key, _)| key)
        .collect::<Vec<_>>();
    for key in edge_keys {
        let edge = translated
            .edge_attr_mut(key)
            .expect("collected edge key must remain valid");
        edge.curve = edge.curve.translated(direction).map_err(|source| {
            ExtrudeError::CurveTranslationFailed {
                dart: edge.dart,
                source,
            }
        })?;
    }

    let translated_face_key = translated
        .face_key_at(translated_dart)
        .expect("isolating a face must preserve its face attribute");
    let translated_facet_key = translated
        .face_attr(translated_face_key)
        .expect("isolated face key must remain valid")
        .facet;
    let translated_face = translated
        .facet_attr_mut(translated_facet_key)
        .expect("isolated facet key must remain valid");
    translated_face.surface = translated_face
        .surface
        .translated(direction)
        .map_err(|source| ExtrudeError::SurfaceTranslationFailed {
            dart: translated_dart,
            source,
        })?;

    Ok(Shape::new(translated, translated_face_key))
}

pub fn add_extruded_face<P: Payload>(
    g: &mut GMap<P>,
    face_key: FaceKey,
    direction: Vector3<f64>,
) -> Result<SolidKey, ExtrudeError> {
    let bot_face = g
        .face(face_key)
        .ok_or(ExtrudeError::MissingFace { dart: face_key })?;
    let top_face = translate_face(&bot_face, direction)?;

    let top_face_dart = g.merge(top_face.face());
    let top_face_key = g
        .face_key_at(top_face_dart)
        .expect("merged top face should preserve its face attribute");
    let bottom_loop_darts =
        face_loop_darts(&g.face(face_key).expect("bottom face must remain valid"));
    let top_loop_darts =
        face_loop_darts(&g.face(top_face_key).expect("top face must remain valid"));
    orient_extruded_caps(g, face_key, top_face_key, direction);

    let mut shell_representative = None;
    for (bottom_loop_dart, top_loop_dart) in bottom_loop_darts.into_iter().zip(top_loop_darts) {
        let extruded = sew_extruded_loop(g, bottom_loop_dart, top_loop_dart, direction)?;
        shell_representative.get_or_insert(extruded);
    }

    let shell_representative =
        shell_representative.expect("a face should have at least one outer loop");
    orient_shell_faces_outward(g, shell_representative);
    let solid = g.add_solid(SolidAttr::new(P::S::default(), shell_representative, None));
    Ok(solid)
}

fn orient_extruded_caps<P: Payload>(
    g: &mut GMap<P>,
    bottom_face: FaceKey,
    top_face: FaceKey,
    direction: Vector3<f64>,
) {
    let Some(bottom_normal_dot_direction) = g
        .face(bottom_face)
        .map(|face| face.normal_at(0.0, 0.0).dot(&direction))
    else {
        return;
    };

    if bottom_normal_dot_direction > LINEAR_TOLERANCE {
        reverse_face_winding(g, bottom_face);
    } else if bottom_normal_dot_direction < -LINEAR_TOLERANCE {
        reverse_face_winding(g, top_face);
    }
}

fn face_loop_darts<P: Payload>(face: &Face<'_, P>) -> Vec<Dart> {
    let mut loops = Vec::with_capacity(1 + face.inner_loops().len());
    loops.push(face.outer_loop().dart);
    loops.extend(face.inner_loops().into_iter().map(|loop_| loop_.dart));
    loops
}

fn orient_shell_faces_outward<P: Payload>(g: &mut GMap<P>, shell: Dart) {
    let face_keys = Sheet::new(g, shell)
        .faces()
        .into_iter()
        .map(|face| face.key())
        .collect::<Vec<_>>();
    let points = face_keys
        .iter()
        .filter_map(|key| g.face(*key))
        .flat_map(|face| face.vertices())
        .filter_map(|vertex| vertex.point().copied())
        .collect::<Vec<_>>();
    if points.is_empty() {
        return;
    }
    let center = points
        .iter()
        .map(|point| point.coords)
        .sum::<Vector3<f64>>()
        / points.len() as f64;
    let flips = face_keys
        .into_iter()
        .filter(|key| {
            let Some(face) = g.face(*key) else {
                return false;
            };
            let uvs = face
                .outer_loop()
                .edges()
                .into_iter()
                .filter_map(|edge| face.pcurve(edge.dart))
                .flat_map(|pcurve| pcurve.sample(1))
                .collect::<Vec<_>>();
            if uvs.is_empty() {
                return false;
            }
            let uv = uvs
                .iter()
                .map(|point| point.coords)
                .sum::<nalgebra::Vector2<f64>>()
                / uvs.len() as f64;
            let face_center = face.point_at(uv.x, uv.y).coords;
            let dot = face.normal_at(0.0, 0.0).dot(&(face_center - center));
            dot < 0.0
        })
        .collect::<Vec<_>>();
    for face in flips {
        g.reverse_face(face);
    }
}

fn reverse_face_winding<P: Payload>(g: &mut GMap<P>, face: FaceKey) {
    g.reverse_face(face);
}

fn sew_extruded_loop<P: Payload>(
    g: &mut GMap<P>,
    bottom_loop_dart: Dart,
    top_loop_dart: Dart,
    direction: Vector3<f64>,
) -> Result<Dart, ExtrudeError> {
    let bottom_edges = Profile::new(g, bottom_loop_dart)
        .edges()
        .into_iter()
        .map(|edge| edge.dart)
        .collect::<Vec<_>>();
    let top_edges = Profile::new(g, top_loop_dart)
        .edges()
        .into_iter()
        .map(|edge| edge.dart)
        .collect::<Vec<_>>();
    let laterals = bottom_edges
        .iter()
        .copied()
        .zip(top_edges.iter().copied())
        .map(|(bottom_edge, top_edge)| {
            let prepared = prepare_lateral_face(g, bottom_edge, top_edge, direction)?;
            let topology = add_lateral_face_topology(g)?;
            add_lateral_face_attributes(g, &topology, &prepared);
            Ok(ExtrudedFaceLateral {
                topology,
                vertical_start: prepared.end,
                vertical_end: prepared.end + direction,
            })
        })
        .collect::<Result<Vec<_>, ExtrudeError>>()?;

    for pair in laterals.windows(2) {
        sew(
            g,
            Dim::Two,
            pair[0].topology.end_vertical,
            pair[1].topology.start_vertical,
        )?;
    }
    if let (Some(first), Some(last)) = (laterals.first(), laterals.last()) {
        sew(
            g,
            Dim::Two,
            last.topology.end_vertical,
            first.topology.start_vertical,
        )?;
    }

    for ((bottom_edge, top_edge), lateral) in bottom_edges.iter().zip(top_edges).zip(&laterals) {
        sew(g, Dim::Two, lateral.topology.bottom_edge, *bottom_edge)?;
        sew(g, Dim::Two, lateral.topology.top_edge, top_edge)?;
    }

    for lateral in &laterals {
        g.add_edge(EdgeAttr::new(
            lateral.topology.end_vertical,
            Curve::line(lateral.vertical_start, lateral.vertical_end),
            P::E::default(),
        ));
    }

    let representative = laterals
        .first()
        .map(|lateral| lateral.topology.bottom_edge)
        .expect("a loop should have at least one lateral face");
    Ok(g.cell_representative(representative, Dim::Three))
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

struct PreparedLateralFace {
    end: Point3,
    surface: Surface,
    uv: [Point2; 4],
}

struct ExtrudedFaceLateral {
    topology: LateralFaceTopology,
    vertical_start: Point3,
    vertical_end: Point3,
}

struct LateralFaceTopology {
    loop_dart: Dart,
    bottom_edge: Dart,
    top_edge: Dart,
    start_vertical: Dart,
    end_vertical: Dart,
    darts: [Dart; 8],
}

fn prepare_lateral_face<P: Payload>(
    g: &GMap<P>,
    bottom_edge: Dart,
    _top_edge: Dart,
    direction: Vector3<f64>,
) -> Result<PreparedLateralFace, ExtrudeError> {
    let bottom_edge = Edge::new(g, bottom_edge);
    let edge_dart = bottom_edge.dart;
    let start = *bottom_edge
        .start()
        .point()
        .ok_or(ExtrudeError::MissingVertexPoint { dart: edge_dart })?;
    let end = *bottom_edge
        .end()
        .point()
        .ok_or(ExtrudeError::MissingVertexPoint { dart: edge_dart })?;
    let curve = bottom_edge
        .curve()
        .ok_or(ExtrudeError::MissingEdgeCurve { dart: edge_dart })?;
    let surface = lateral_face_surface(edge_dart, curve, start, end, direction)?;
    let uv = lateral_face_uv(&surface, curve, start, end, direction);

    Ok(PreparedLateralFace { end, uv, surface })
}

fn add_lateral_face_topology<P: Payload>(
    g: &mut GMap<P>,
) -> Result<LateralFaceTopology, ExtrudeError> {
    let darts = std::array::from_fn(|_| g.add_dart());

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

    Ok(LateralFaceTopology {
        loop_dart: darts[0],
        bottom_edge: darts[0],
        top_edge: darts[5],
        start_vertical: darts[7],
        end_vertical: darts[2],
        darts,
    })
}

fn add_lateral_face_attributes<P: Payload>(
    g: &mut GMap<P>,
    topology: &LateralFaceTopology,
    prepared: &PreparedLateralFace,
) {
    g.add_face(
        topology.loop_dart,
        Vec::new(),
        FacetAttr::with_pcurves(
            prepared.surface.clone(),
            P::F::default(),
            quad_pcurves(&prepared.uv, &topology.darts),
        ),
    );
}

fn lateral_face_surface(
    dart: Dart,
    curve: &Curve,
    start: Point3,
    end: Point3,
    direction: Vector3<f64>,
) -> Result<Surface, ExtrudeError> {
    match curve {
        Curve::Line(_) => Ok(Surface::Plane(lateral_plane(dart, start, end, direction)?)),
        Curve::Bounded(_) if is_linear_curve(curve) => {
            Ok(Surface::Plane(lateral_plane(dart, start, end, direction)?))
        }
        Curve::Circle(_) | Curve::Nurbs(_) | Curve::Bounded(_) => {
            Ok(Surface::Ruled(RuledSurface::new(curve.clone(), direction)))
        }
    }
}

fn lateral_face_uv(
    surface: &Surface,
    curve: &Curve,
    start: Point3,
    end: Point3,
    direction: Vector3<f64>,
) -> [Point2; 4] {
    match surface {
        Surface::Plane(plane) => [
            plane_uv(plane, start),
            plane_uv(plane, end),
            plane_uv(plane, end + direction),
            plane_uv(plane, start + direction),
        ],
        Surface::Ruled(_) => {
            let interval = curve.parameters_between(start, end);
            [
                Point2::new(interval.start, 0.0),
                Point2::new(interval.end, 0.0),
                Point2::new(interval.end, 1.0),
                Point2::new(interval.start, 1.0),
            ]
        }
        _ => unreachable!("lateral_face_surface only creates plane or ruled surfaces"),
    }
}

fn is_linear_curve(curve: &Curve) -> bool {
    match curve {
        Curve::Line(_) => true,
        Curve::Bounded(bounded) => matches!(bounded.inner(), Curve::Line(_)),
        _ => false,
    }
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

fn plane_uv(surface: &Plane, point: Point3) -> Point2 {
    let v = point - surface.origin();
    Point2::new(v.dot(&surface.x_dir()), v.dot(&surface.y_dir()))
}

fn quad_pcurves(uv: &[Point2; 4], darts: &[Dart; 8]) -> std::collections::HashMap<Dart, Curve2> {
    let mut pcurves = std::collections::HashMap::with_capacity(4);
    for i in 0..4 {
        pcurves.insert(
            darts[2 * i],
            Curve2::Line(Line2::new(uv[i], uv[(i + 1) % uv.len()])),
        );
    }
    pcurves
}
