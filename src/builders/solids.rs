use nalgebra::Vector3;

use crate::{
    Payload,
    builders::{errors::ExtrudeError, sheets::add_extruded_profile_boundaries},
    topology::{
        SolidAttr,
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
    let outer_loop_dart = bot_face.outer_loop().dart;
    let bottom_edges = Profile::new(g, outer_loop_dart)
        .edges()
        .into_iter()
        .map(|edge| edge.dart)
        .collect::<Vec<_>>();

    let top_face_dart = g.merge(top_face.face());
    let top_edges = Profile::new(g, top_face_dart)
        .edges()
        .into_iter()
        .map(|edge| edge.dart)
        .collect::<Vec<_>>();
    let extruded = add_extruded_profile_boundaries(g, outer_loop_dart, direction)?;

    for (&cap_edge, &side_edge) in bottom_edges.iter().zip(extruded.bottom_edges.iter()) {
        sew(g, Dim::Two, side_edge, cap_edge)?;
    }
    for (&cap_edge, &side_edge) in top_edges.iter().zip(extruded.top_edges.iter()) {
        sew(g, Dim::Two, side_edge, cap_edge)?;
    }

    let shell_representative = g.cell_representative(extruded.bottom_edges[0], Dim::Three);
    let solid = g.add_solid(SolidAttr::new(P::S::default(), shell_representative, None));
    Ok(solid)
}

fn sew<P: Payload>(
    g: &mut GMap<P>,
    dim: Dim,
    first: crate::topology::Dart,
    second: crate::topology::Dart,
) -> Result<(), ExtrudeError> {
    g.sew(dim, first, second)
        .map_err(|_| ExtrudeError::SewFailed { dim, first, second })
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::translate_face;
    use crate::builders::profiles::add_polygon;
    use crate::geometry::{Plane, Point3, Surface};
    use crate::topology::attributes::FaceAttr;
    use crate::topology::face::Face;
    use crate::topology::gmap::GMap;
    use crate::topology::payload::StandardPayload;

    #[test]
    fn translate_face_copies_face_into_translated_map() {
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
        let face_key = source.add_face(FaceAttr::new(
            Surface::Plane(Plane::from_xy(
                Point3::new(0.0, 0.0, 0.0),
                Vector3::x(),
                Vector3::y(),
            )),
            (),
            loop_dart,
            Vec::new(),
        ));
        let face = source
            .face(face_key)
            .map(|attr| Face::new(&source, attr))
            .expect("source face should exist");

        let translated = translate_face(&face, Vector3::new(0.0, 0.0, 2.0)).unwrap();

        assert_eq!(translated.map().dart_count(), 8);
        assert_eq!(translated.map().iter_faces().count(), 1);
        assert!(
            translated
                .map()
                .iter_vertices()
                .all(|(_, attr)| (attr.point.z - 2.0).abs() <= f64::EPSILON)
        );
        assert!(
            source
                .iter_vertices()
                .all(|(_, attr)| attr.point.z.abs() <= f64::EPSILON)
        );

        match translated.face().surface() {
            Surface::Plane(plane) => {
                assert!((plane.origin().z - 2.0).abs() <= f64::EPSILON);
            }
            _ => panic!("test face should remain planar"),
        }
    }
}
