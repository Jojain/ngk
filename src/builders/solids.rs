use nalgebra::Vector3;

use crate::{
    Payload,
    builders::{errors::ExtrudeError, sheets::add_extruded_profile_boundaries},
    topology::{
        Dart, SolidAttr,
        face::Face,
        gmap::{Cell2, Dim, GMap, MergeTopology},
        profile::Profile,
        shape::{FaceShape, Shape},
        shape_keys::{FaceKey, SolidKey},
    },
};

pub fn translate_face<P: Payload>(
    face: &Face<'_, P>,
    direction: Vector3<f64>,
) -> Result<Shape<FaceShape, P>, ExtrudeError> {
    if direction.norm_squared() <= f64::EPSILON {
        return Err(ExtrudeError::ZeroDirection);
    }

    let (mut translated, translated_dart) = face.isolate();

    let vertex_keys = translated
        .iter_vertices()
        .map(|(key, _)| key)
        .collect::<Vec<_>>();
    for key in vertex_keys {
        let vertex = translated
            .vertex_mut(key)
            .expect("collected vertex key must remain valid");
        vertex.point += direction;
    }

    let edge_keys = translated
        .iter_edges()
        .map(|(key, _)| key)
        .collect::<Vec<_>>();
    for key in edge_keys {
        let edge = translated
            .edge_mut(key)
            .expect("collected edge key must remain valid");
        edge.curve = edge.curve.translated(direction).map_err(|source| {
            ExtrudeError::CurveTranslationFailed {
                dart: edge.dart,
                source,
            }
        })?;
    }

    let translated_face_key = *translated
        .attribute::<Cell2>(translated_dart)
        .expect("isolating a face must preserve its face attribute");
    let translated_face = translated
        .face_mut(translated_face_key)
        .expect("isolated face key must remain valid");
    translated_face.surface = translated_face
        .surface
        .translated(direction)
        .map_err(|source| ExtrudeError::SurfaceTranslationFailed {
            dart: translated_face.outer_loop,
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
        .map(|attr| attr.face(g))
        .ok_or(ExtrudeError::MissingFace { dart: face_key })?;
    let top_face = translate_face(&bot_face, direction)?;
    let mut bottom_loop_darts = Vec::with_capacity(1 + bot_face.inner_loops().len());
    bottom_loop_darts.push(bot_face.outer_loop().dart);
    bottom_loop_darts.extend(bot_face.inner_loops().into_iter().map(|loop_| loop_.dart));

    let top_face_dart = g.merge(top_face.face());
    let top_face_key = *g
        .attribute::<Cell2>(top_face_dart)
        .expect("merged top face should preserve its face attribute");
    let top_face_attr = g
        .face(top_face_key)
        .expect("merged top face key should remain valid");
    let mut top_loop_darts = Vec::with_capacity(1 + top_face_attr.inner_loops.len());
    top_loop_darts.push(top_face_attr.outer_loop);
    top_loop_darts.extend(top_face_attr.inner_loops.iter().copied());

    let mut shell_representative = None;
    for (bottom_loop_dart, top_loop_dart) in bottom_loop_darts.into_iter().zip(top_loop_darts) {
        let extruded = sew_extruded_loop(g, bottom_loop_dart, top_loop_dart, direction)?;
        shell_representative.get_or_insert(extruded);
    }

    let shell_representative =
        shell_representative.expect("a face should have at least one outer loop");
    let solid = g.add_solid(SolidAttr::new(P::S::default(), shell_representative, None));
    Ok(solid)
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
    let extruded = add_extruded_profile_boundaries(g, bottom_loop_dart, direction)?;

    for (&cap_edge, &side_edge) in bottom_edges.iter().zip(extruded.bottom_edges.iter()) {
        sew(g, Dim::Two, side_edge, cap_edge)?;
    }
    for (&cap_edge, &side_edge) in top_edges.iter().zip(extruded.top_edges.iter()) {
        sew(g, Dim::Two, side_edge, cap_edge)?;
    }

    Ok(g.cell_representative(extruded.bottom_edges[0], Dim::Three))
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
