use crate::builders::errors::FaceCreationError;
use crate::builders::faces::{
    add_annulus, add_circle, add_face, add_polygon, add_polygon_with_holes, add_rectangle,
    add_square,
};
use crate::geometry::{Plane, Point3};
use crate::topology::gmap::GMap;
use crate::topology::payload::StandardPayload;
use crate::topology::shape::{FaceTag, Shape};

pub fn rectangle(
    plane: Plane,
    x_size: f64,
    y_size: f64,
) -> Result<Shape<FaceTag, StandardPayload>, FaceCreationError> {
    let mut g = GMap::new();
    let face_key = add_rectangle(&mut g, plane, x_size, y_size)?;
    Ok(Shape::new(g, face_key))
}

pub fn square(
    plane: Plane,
    size: f64,
) -> Result<Shape<FaceTag, StandardPayload>, FaceCreationError> {
    let mut g = GMap::new();
    let handle = add_square(&mut g, plane, size)?;
    Ok(Shape::new(g, handle))
}

pub fn circle(
    plane: Plane,
    radius: f64,
) -> Result<Shape<FaceTag, StandardPayload>, FaceCreationError> {
    let mut g = GMap::new();
    let face_key = add_circle(&mut g, plane, radius)?;
    Ok(Shape::new(g, face_key))
}

pub fn annulus(
    plane: Plane,
    outer_radius: f64,
    inner_radius: f64,
) -> Result<Shape<FaceTag, StandardPayload>, FaceCreationError> {
    let mut g = GMap::new();
    let face_key = add_annulus(&mut g, plane, outer_radius, inner_radius)?;
    Ok(Shape::new(g, face_key))
}

pub fn polygon(points: &[Point3]) -> Result<Shape<FaceTag, StandardPayload>, FaceCreationError> {
    if points.len() < 3 {
        return Err(FaceCreationError::InvalidPolygon {
            point_count: points.len(),
        });
    }

    let mut g = GMap::new();
    let profile_key = add_polygon(&mut g, points);
    let loop_dart = g.profile_attr_unchecked(profile_key).dart;
    let face_key = add_face(&mut g, loop_dart)?;
    Ok(Shape::new(g, face_key))
}

pub fn polygon_with_holes(
    plane: Plane,
    outer: &[Point3],
    holes: &[&[Point3]],
) -> Result<Shape<FaceTag, StandardPayload>, FaceCreationError> {
    let mut g = GMap::new();
    let face_key = add_polygon_with_holes(&mut g, plane, outer, holes)?;
    Ok(Shape::new(g, face_key))
}
