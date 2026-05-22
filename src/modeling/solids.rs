use nalgebra::Vector3;
use thiserror::Error;

use crate::{
    StandardPayload,
    builders::solids::add_extruded_face,
    geometry::Plane,
    modeling::faces,
    topology::shape::{Shape, SolidTag},
};

#[derive(Debug, Clone, Error, PartialEq)]
pub enum PrimitiveError {
    #[error("block {axis} size must be greater than 0, got {value}")]
    InvalidSize { axis: &'static str, value: f64 },
    #[error("failed to create the block base face")]
    FaceCreationFailed,
    #[error("failed to extrude the block base face")]
    SolidCreationFailed,
}

fn validate_size(axis: &'static str, value: f64) -> Result<(), PrimitiveError> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err(PrimitiveError::InvalidSize { axis, value })
    }
}

pub fn block(
    x_size: f64,
    y_size: f64,
    z_size: f64,
) -> Result<Shape<SolidTag, StandardPayload>, PrimitiveError> {
    validate_size("x", x_size)?;
    validate_size("y", y_size)?;
    validate_size("z", z_size)?;

    let base = faces::rectangle(Plane::xy(), x_size, y_size)
        .map_err(|_| PrimitiveError::FaceCreationFailed)?;
    let (mut g, face_key) = base.into_map();
    let direction = Vector3::new(0.0, 0.0, z_size);
    let solid_key = add_extruded_face(&mut g, face_key, direction)
        .map_err(|_| PrimitiveError::SolidCreationFailed)?;
    Ok(Shape::new(g, solid_key))
}
