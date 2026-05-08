use std::collections::HashMap;

use nalgebra::Vector3;

use crate::builders::profiles::{PolylineError, add_polyline};
use crate::geometry::{Curve, Line, Plane, Point3, Surface};
use crate::topology::FaceAttr;
use crate::topology::gmap::GMap;
use crate::topology::payload::StandardPayload;
use crate::topology::shape::{FaceShape, ProfileShape, Shape};

pub fn polyline(
    segments: &[(Point3, Point3, Curve)],
) -> Result<Shape<ProfileShape, StandardPayload>, PolylineError> {
    let mut g = GMap::new();
    let profile_dart = add_polyline(&mut g, segments)?;
    Ok(Shape::new(g, profile_dart))
}

/// Adds a square profile to the given GMap.
///
/// The corners are expected to be in the following order:
/// 0-----1
/// |     |
/// |     |
/// 3-----2
///
/// Returns the dart of the first corner.
pub fn square_loop(corners: &[Point3; 4]) -> Result<Shape<ProfileShape>, PolylineError> {
    let segments = [
        (
            corners[0],
            corners[1],
            Curve::Line(Line::new(corners[0], corners[1])),
        ),
        (
            corners[1],
            corners[2],
            Curve::Line(Line::new(corners[1], corners[2])),
        ),
        (
            corners[2],
            corners[3],
            Curve::Line(Line::new(corners[2], corners[3])),
        ),
        (
            corners[3],
            corners[0],
            Curve::Line(Line::new(corners[3], corners[0])),
        ),
    ];
    polyline(&segments)
}

pub fn square(corners: &[Point3; 4]) -> Result<Shape<FaceShape>, PolylineError> {
    let loop_shape = square_loop(corners)?;
    let (mut gmap, loop_dart) = loop_shape.into_map();
    let face = FaceAttr::with_pcurves(
        Surface::Plane(Plane::from_xy(Point3::origin(), Vector3::x(), Vector3::y())),
        (),
        loop_dart,
        Vec::new(),
        HashMap::new(),
    );
    let face_key = gmap.add_face(face);

    let shape = Shape::new(gmap, face_key);
    Ok(shape)
}
