use nalgebra::{Unit, Vector3};

use crate::{
    StandardPayload,
    builders::solids::{add_extruded_face, add_sphere},
    geometry::{Frame, Plane},
    modeling::faces,
    topology::{
        gmap::GMap,
        shape::{FaceTag, Shape, SolidTag},
    },
};

pub use crate::modeling::errors::PrimitiveError;

fn validate_length(axis: &'static str, value: f64) -> Result<(), PrimitiveError> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err(PrimitiveError::InvalidSize { axis, value })
    }
}

/// Creates a block (rectangular prism) at the given frame with the specified dimensions.
pub fn block_at(
    frame: Frame,
    x_size: f64,
    y_size: f64,
    z_size: f64,
) -> Result<Shape<SolidTag, StandardPayload>, PrimitiveError> {
    validate_length("x", x_size)?;
    validate_length("y", y_size)?;
    validate_length("z", z_size)?;
    let direction = frame.z_dir.into_inner() * z_size;
    let plane = Plane::from_frame(frame);
    let base =
        faces::rectangle(plane, x_size, y_size).map_err(|_| PrimitiveError::FaceCreationFailed)?;
    let (mut g, face_key) = base.into_map();
    let solid_key = add_extruded_face(&mut g, face_key, direction)
        .map_err(|_| PrimitiveError::SolidCreationFailed)?;
    Ok(Shape::new(g, solid_key))
}

// Creates a block at the origin with the specified dimensions.
pub fn block(
    x_size: f64,
    y_size: f64,
    z_size: f64,
) -> Result<Shape<SolidTag, StandardPayload>, PrimitiveError> {
    block_at(Frame::xyz(), x_size, y_size, z_size)
}

/// Creates a cylinder at the given frame with the specified radius and height.
pub fn cylinder_at(
    frame: Frame,
    radius: f64,
    height: f64,
) -> Result<Shape<SolidTag, StandardPayload>, PrimitiveError> {
    validate_length("radius", radius)?;
    validate_length("height", height)?;
    let direction = frame.z_dir.into_inner() * height;
    let plane = Plane::from_frame(frame);
    let base = faces::circle(plane, radius).map_err(|_| PrimitiveError::FaceCreationFailed)?;
    let (mut g, face_key) = base.into_map();
    let solid_key = add_extruded_face(&mut g, face_key, direction)
        .map_err(|_| PrimitiveError::SolidCreationFailed)?;
    Ok(Shape::new(g, solid_key))
}

/// Creates a cylinder at the origin with the specified radius and height.
pub fn cylinder(
    radius: f64,
    height: f64,
) -> Result<Shape<SolidTag, StandardPayload>, PrimitiveError> {
    cylinder_at(Frame::xyz(), radius, height)
}

/// Creates a sphere centered at the given frame origin.
///
/// The frame's z-axis is the revolution axis and its x-axis fixes the
/// generating circle arc's meridian.
pub fn sphere_at(
    frame: Frame,
    radius: f64,
) -> Result<Shape<SolidTag, StandardPayload>, PrimitiveError> {
    validate_length("radius", radius)?;
    let mut g = GMap::new();
    let solid_key =
        add_sphere(&mut g, frame, radius).map_err(|_| PrimitiveError::SolidCreationFailed)?;
    Ok(Shape::new(g, solid_key))
}

/// Creates a sphere centered at the origin.
pub fn sphere(radius: f64) -> Result<Shape<SolidTag, StandardPayload>, PrimitiveError> {
    sphere_at(Frame::xyz(), radius)
}

/// Creates a solid by extruding the given face in the specified direction.
pub fn extruded(
    face: Shape<FaceTag, StandardPayload>,
    direction: Unit<Vector3<f64>>,
    distance: f64,
) -> Result<Shape<SolidTag, StandardPayload>, PrimitiveError> {
    validate_length("distance", distance)?;
    let direction = direction.into_inner() * distance;
    let (mut g, face_key) = face.into_map();
    let solid_key = add_extruded_face(&mut g, face_key, direction)
        .map_err(|_| PrimitiveError::SolidCreationFailed)?;
    Ok(Shape::new(g, solid_key))
}
