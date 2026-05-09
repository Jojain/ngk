use std::fmt;

use nalgebra::Vector3;

use crate::{
    StandardPayload,
    builders::solids::add_extruded_face,
    geometry::Point3,
    modeling::profiles::square,
    topology::shape::{Shape, SolidShape},
};

#[derive(Debug, Clone, PartialEq)]
pub enum PrimitiveError {
    InvalidSize { axis: &'static str, value: f64 },
    FaceCreationFailed,
    SolidCreationFailed,
}

impl fmt::Display for PrimitiveError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidSize { axis, value } => {
                write!(f, "block {axis} size must be greater than 0, got {value}")
            }
            Self::FaceCreationFailed => write!(f, "failed to create the block base face"),
            Self::SolidCreationFailed => write!(f, "failed to extrude the block base face"),
        }
    }
}

impl std::error::Error for PrimitiveError {}

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
) -> Result<Shape<SolidShape, StandardPayload>, PrimitiveError> {
    validate_size("x", x_size)?;
    validate_size("y", y_size)?;
    validate_size("z", z_size)?;

    let corners = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(x_size, 0.0, 0.0),
        Point3::new(x_size, y_size, 0.0),
        Point3::new(0.0, y_size, 0.0),
    ];
    let square = square(&corners).map_err(|_| PrimitiveError::FaceCreationFailed)?;
    let (mut g, face_key) = square.into_map();
    let direction = Vector3::new(0.0, 0.0, z_size);
    let solid_key = add_extruded_face(&mut g, face_key, direction)
        .map_err(|_| PrimitiveError::SolidCreationFailed)?;
    Ok(Shape::new(g, solid_key))
}
