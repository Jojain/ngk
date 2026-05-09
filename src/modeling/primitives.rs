use nalgebra::Vector3;

use crate::{
    StandardPayload,
    builders::{
        profiles::{add_face, add_square},
        solids::add_extruded_face,
    },
    geometry::{Plane, Point3},
    modeling::profiles::square,
    topology::{
        closed::Closed,
        gmap::GMap,
        planar::PlanarLoop,
        profile::{Loop, Profile},
        shape::{Shape, SolidShape},
    },
};

#[derive(Debug, Clone, PartialEq)]
pub enum PrimitiveError {
    CubeCreationError,
}

pub fn cube(
    x_size: f64,
    y_size: f64,
    z_size: f64,
) -> Result<Shape<SolidShape, StandardPayload>, PrimitiveError> {
    let corners = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(x_size, 0.0, 0.0),
        Point3::new(x_size, y_size, 0.0),
        Point3::new(0.0, y_size, 0.0),
    ];
    let square = square(&corners).map_err(|e| PrimitiveError::CubeCreationError)?;
    let (mut g, face_key) = square.into_map();
    let direction = Vector3::new(0.0, 0.0, z_size);
    let solid_key = add_extruded_face(&mut g, face_key, direction)
        .map_err(|e| PrimitiveError::CubeCreationError)?;
    Ok(Shape::new(g, solid_key))
}
