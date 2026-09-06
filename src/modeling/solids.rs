use nalgebra::{Unit, Vector3};

use crate::{
    StandardPayload,
    builders::boolean::{BooleanError, BooleanOperation, BooleanOptions, boolean},
    builders::solids::{add_extruded_face, add_sphere},
    geometry::{Frame, Plane},
    modeling::faces,
    topology::{
        TopologyEditError,
        gmap::GMap,
        payload::Payload,
        shape::{FaceTag, Shape, SolidTag},
    },
};

#[cfg(feature = "python")]
use crate::topology::solid::Solid;

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

/// Consumes two owned solid shapes and fuses them into one owned solid.
pub fn fuse<P: Payload>(
    first: Shape<SolidTag, P>,
    second: Shape<SolidTag, P>,
) -> Result<Shape<SolidTag, P>, BooleanError> {
    combine_shapes(first, second, BooleanOperation::Union)
}

/// Consumes two owned solid shapes and subtracts `tool` from `target`.
pub fn cut<P: Payload>(
    target: Shape<SolidTag, P>,
    tool: Shape<SolidTag, P>,
) -> Result<Shape<SolidTag, P>, BooleanError> {
    combine_shapes(target, tool, BooleanOperation::Difference)
}

/// Consumes two owned solid shapes and returns their common volume.
pub fn intersect<P: Payload>(
    first: Shape<SolidTag, P>,
    second: Shape<SolidTag, P>,
) -> Result<Shape<SolidTag, P>, BooleanError> {
    combine_shapes(first, second, BooleanOperation::Intersection)
}

/// Moves two owned shapes into one working map and evaluates one Boolean.
fn combine_shapes<P: Payload>(
    target: Shape<SolidTag, P>,
    tool: Shape<SolidTag, P>,
    operation: BooleanOperation,
) -> Result<Shape<SolidTag, P>, BooleanError> {
    let (mut map, target) = target.into_map();
    let (tool_map, tool) = tool.into_map();
    let tool = map.transaction(|edit| {
        let dart = edit.merge(tool_map.solid_unchecked(tool));
        Ok::<_, TopologyEditError>(
            edit.solid_key(dart)
                .expect("copied tool solid must retain its registration"),
        )
    })?;
    let result = boolean(&mut map, target, tool, operation, BooleanOptions::default())?;
    Ok(Shape::new(map, result.solid))
}

/// Copies two borrowed solid views for language bindings and evaluates a Boolean.
#[cfg(feature = "python")]
pub(crate) fn combine_views<P: Payload>(
    first: Solid<'_, P>,
    second: Solid<'_, P>,
    operation: BooleanOperation,
) -> Result<Shape<SolidTag, P>, BooleanError> {
    let mut map = GMap::new();
    let (first, second) = map.transaction(|edit| {
        let first_dart = edit.merge(first);
        let second_dart = edit.merge(second);
        Ok::<_, TopologyEditError>((
            edit.solid_key(first_dart)
                .expect("copied first solid must retain its registration"),
            edit.solid_key(second_dart)
                .expect("copied second solid must retain its registration"),
        ))
    })?;
    let result = boolean(
        &mut map,
        first,
        second,
        operation,
        BooleanOptions::default(),
    )?;
    Ok(Shape::new(map, result.solid))
}
