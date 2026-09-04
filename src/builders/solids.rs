use nalgebra::Vector3;

use crate::{
    Payload,
    builders::errors::ExtrudeError,
    builders::faces::reverse_face_winding,
    geometry::{
        Curve, Curve2, LINEAR_TOLERANCE, Line2, Plane, Point2, Point3, RuledSurface, Surface,
    },
    topology::{
        Dart, SheetAttr, SolidAttr, TopologyEdit,
        attributes::{EdgeAttr, FaceAttr, ProfileAttr},
        edge::Edge,
        face::Face,
        gmap::{Cell2, Dim, GMap, MergeTopology},
        profile::Profile,
        shape::{FaceTag, Shape},
        shape_keys::{FaceKey, SolidKey},
    },
};

/// Returns an isolated copy of `face` translated by `direction`.
///
/// Vertex positions, edge curves, and the supporting surface are translated;
/// face pcurves remain unchanged because they use the face's local parameter
/// space. The source map is not modified.
///
/// Returns an error for a zero direction or when curve or surface geometry
/// cannot be translated.
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
    let edge_keys = translated
        .iter_edges()
        .map(|(key, _)| key)
        .collect::<Vec<_>>();
    let translated_face_key = *translated.attribute_unchecked::<Cell2>(translated_dart);
    translated.transaction(|edit| {
        for key in vertex_keys {
            edit.vertex_attr_mut_unchecked(key).point += direction;
        }

        for key in edge_keys {
            let edge = edit.edge_attr_mut_unchecked(key);
            edge.curve = edge.curve.translated(direction).map_err(|source| {
                ExtrudeError::CurveTranslationFailed {
                    dart: edge.dart,
                    source,
                }
            })?;
        }

        let translated_face = edit.face_attr_mut_unchecked(translated_face_key);
        translated_face.surface =
            translated_face
                .surface
                .translated(direction)
                .map_err(|source| ExtrudeError::SurfaceTranslationFailed {
                    dart: translated_face.outer_loop,
                    source,
                })?;
        Ok::<_, ExtrudeError>(())
    })?;

    Ok(Shape::new(translated, translated_face_key))
}

/// Extrudes an existing face into a solid along `direction`.
///
/// The source face becomes one cap, a translated copy becomes the opposite cap,
/// and one lateral face is added for every edge of the outer and inner boundary
/// loops. Cap winding is adjusted so the resulting shell faces outward.
///
/// Returns an error when the face is missing, the direction is zero, required
/// boundary geometry is absent, or a lateral face is degenerate or cannot be
/// sewn into the shell.
pub fn add_extruded_face<P: Payload>(
    g: &mut GMap<P>,
    face_key: FaceKey,
    direction: Vector3<f64>,
) -> Result<SolidKey, ExtrudeError> {
    g.transaction(|g| add_extruded_face_staged(g, face_key, direction))
}

/// Builds translated caps and lateral faces, then registers the staged solid.
fn add_extruded_face_staged<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    face_key: FaceKey,
    direction: Vector3<f64>,
) -> Result<SolidKey, ExtrudeError> {
    let bot_face = g
        .face_attr(face_key)
        .map(|attr| attr.face(g))
        .ok_or(ExtrudeError::MissingFace { dart: face_key })?;
    let top_face = translate_face(&bot_face, direction)?;
    let mut bottom_loop_darts = Vec::with_capacity(1 + bot_face.inner_loops().len());
    bottom_loop_darts.push(bot_face.outer_loop().dart);
    bottom_loop_darts.extend(bot_face.inner_loops().into_iter().map(|loop_| loop_.dart));

    let top_face_dart = g.merge(top_face.face());
    let top_face_key = *g.attribute_unchecked::<Cell2>(top_face_dart);
    let top_face_attr = g.face_attr_unchecked(top_face_key);
    let mut top_loop_darts = Vec::with_capacity(1 + top_face_attr.inner_loops.len());
    top_loop_darts.push(top_face_attr.outer_loop);
    top_loop_darts.extend(top_face_attr.inner_loops.iter().copied());

    orient_extruded_caps(g, face_key, top_face_key, direction);

    for (bottom_loop_dart, top_loop_dart) in bottom_loop_darts.into_iter().zip(top_loop_darts) {
        sew_extruded_loop(g, bottom_loop_dart, top_loop_dart, direction)?;
    }

    // The shell dart is contextual: unlike a cell representative, it must retain
    // the outward orientation established for the bottom cap.
    let outer_shell = g.face_attr_unchecked(face_key).outer_loop;
    if g.sheet_key(outer_shell).is_none() {
        g.add_sheet(SheetAttr::new(outer_shell, P::Sheet::default()));
    }
    let solid = g.add_solid(SolidAttr::new(P::S::default(), outer_shell, None));
    Ok(solid)
}

fn orient_extruded_caps<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    bottom_face: FaceKey,
    top_face: FaceKey,
    direction: Vector3<f64>,
) {
    let Some(bottom_normal_dot_direction) = g
        .face_attr(bottom_face)
        .map(|attr| face_normal_dot_direction(g, attr, direction))
    else {
        return;
    };

    if bottom_normal_dot_direction > LINEAR_TOLERANCE {
        reverse_face_winding(g, bottom_face);
    } else if bottom_normal_dot_direction < -LINEAR_TOLERANCE {
        reverse_face_winding(g, top_face);
    }
}

fn face_normal_dot_direction<P: Payload>(
    g: &GMap<P>,
    face: &FaceAttr<P::F>,
    direction: Vector3<f64>,
) -> f64 {
    face.face(g).normal_at(0.0, 0.0).dot(&direction)
}

fn sew_extruded_loop<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    bottom_loop_dart: Dart,
    top_loop_dart: Dart,
    direction: Vector3<f64>,
) -> Result<Dart, ExtrudeError> {
    let bottom_edges = Profile::from_dart(g, bottom_loop_dart)
        .expect("bottom loop must have a registered profile")
        .edges()
        .into_iter()
        .map(|edge| edge.dart())
        .collect::<Vec<_>>();
    let top_edges = Profile::from_dart(g, top_loop_dart)
        .expect("top loop must have a registered profile")
        .edges()
        .into_iter()
        .map(|edge| edge.dart())
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
    g: &mut TopologyEdit<'_, P>,
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
    let bottom_edge_view = Edge::from_dart(g, bottom_edge)
        .ok_or(ExtrudeError::MissingEdgeCurve { dart: bottom_edge })?;
    let edge_dart = bottom_edge_view.dart();
    let start = *bottom_edge_view
        .start()
        .point()
        .ok_or(ExtrudeError::MissingVertexPoint { dart: edge_dart })?;
    let end = *bottom_edge_view
        .end()
        .point()
        .ok_or(ExtrudeError::MissingVertexPoint { dart: edge_dart })?;
    let curve = bottom_edge_view
        .curve()
        .ok_or(ExtrudeError::MissingEdgeCurve { dart: edge_dart })?;
    let surface = lateral_face_surface(edge_dart, curve, start, end, direction)?;
    let uv = lateral_face_uv(&surface, curve, start, end, direction);

    Ok(PreparedLateralFace { end, uv, surface })
}

fn add_lateral_face_topology<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
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
    g: &mut TopologyEdit<'_, P>,
    topology: &LateralFaceTopology,
    prepared: &PreparedLateralFace,
) {
    g.add_profile(ProfileAttr::new(topology.loop_dart, P::Profile::default()));
    g.add_face(FaceAttr::with_pcurves(
        prepared.surface.clone(),
        P::F::default(),
        topology.loop_dart,
        Vec::new(),
        quad_pcurves(&prepared.uv, &topology.darts),
    ));
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
