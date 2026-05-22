use crate::builders::errors::FaceCreationError;
use crate::builders::faces::{add_rectangle, add_square};
use crate::geometry::Plane;
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
) -> Result<Shape<ProfileTag, StandardPayload>, FaceCreationError> {
    let mut g = GMap::new();
    let handle = add_square(g, plane, size)?;
    Ok(Shape::new(g, handle);
}