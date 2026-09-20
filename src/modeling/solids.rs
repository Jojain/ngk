use nalgebra::{Unit, Vector3};

use crate::{
    StandardPayload,
    builders::boolean::{BooleanError, BooleanOperation, BooleanOptions, boolean},
    builders::errors::ExtrudeError,
    builders::solids::{
        BlockError, CylinderError, SphereBuildError, TorusBuildError, add_block, add_cylinder,
        add_extruded_face, add_sphere, add_torus,
    },
    geometry::Frame,
    model::Model,
    topology::{
        ModelEditError,
        payload::Payload,
        shape::{FaceTag, Shape, SolidTag},
    },
};

use crate::topology::solid::Solid;

/// Creates a block (rectangular prism) at the given frame with the specified dimensions.
pub fn block_at(
    frame: Frame,
    x_size: f64,
    y_size: f64,
    z_size: f64,
) -> Result<Shape<SolidTag>, BlockError> {
    block_at_with::<StandardPayload>(frame, x_size, y_size, z_size)
}

/// As [`block_at`], with the payload chosen by the caller.
pub fn block_at_with<P: Payload>(
    frame: Frame,
    x_size: f64,
    y_size: f64,
    z_size: f64,
) -> Result<Shape<SolidTag, P>, BlockError> {
    Shape::build(|model| {
        add_block(model, frame, x_size, y_size, z_size).map(|extrusion| extrusion.solid)
    })
}

/// Creates a block at the origin with the specified dimensions.
pub fn block(x_size: f64, y_size: f64, z_size: f64) -> Result<Shape<SolidTag>, BlockError> {
    block_with::<StandardPayload>(x_size, y_size, z_size)
}

/// As [`block`], with the payload chosen by the caller.
pub fn block_with<P: Payload>(
    x_size: f64,
    y_size: f64,
    z_size: f64,
) -> Result<Shape<SolidTag, P>, BlockError> {
    block_at_with::<P>(Frame::xyz(), x_size, y_size, z_size)
}

/// Creates a cylinder at the given frame with the specified radius and height.
pub fn cylinder_at(
    frame: Frame,
    radius: f64,
    height: f64,
) -> Result<Shape<SolidTag>, CylinderError> {
    cylinder_at_with::<StandardPayload>(frame, radius, height)
}

/// As [`cylinder_at`], with the payload chosen by the caller.
pub fn cylinder_at_with<P: Payload>(
    frame: Frame,
    radius: f64,
    height: f64,
) -> Result<Shape<SolidTag, P>, CylinderError> {
    Shape::build(|model| {
        add_cylinder(model, frame, radius, height).map(|extrusion| extrusion.solid)
    })
}

/// Creates a cylinder at the origin with the specified radius and height.
pub fn cylinder(radius: f64, height: f64) -> Result<Shape<SolidTag>, CylinderError> {
    cylinder_with::<StandardPayload>(radius, height)
}

/// As [`cylinder`], with the payload chosen by the caller.
pub fn cylinder_with<P: Payload>(
    radius: f64,
    height: f64,
) -> Result<Shape<SolidTag, P>, CylinderError> {
    cylinder_at_with::<P>(Frame::xyz(), radius, height)
}

/// Creates a sphere centered at the given frame origin.
///
/// The frame's z-axis is the revolution axis and its x-axis fixes the
/// generating circle arc's meridian.
pub fn sphere_at(frame: Frame, radius: f64) -> Result<Shape<SolidTag>, SphereBuildError> {
    sphere_at_with::<StandardPayload>(frame, radius)
}

/// As [`sphere_at`], with the payload chosen by the caller.
pub fn sphere_at_with<P: Payload>(
    frame: Frame,
    radius: f64,
) -> Result<Shape<SolidTag, P>, SphereBuildError> {
    Shape::build(|model| add_sphere(model, frame, radius).map(|solid| solid.solid))
}

/// Creates a sphere centered at the origin.
pub fn sphere(radius: f64) -> Result<Shape<SolidTag>, SphereBuildError> {
    sphere_with::<StandardPayload>(radius)
}

/// As [`sphere`], with the payload chosen by the caller.
pub fn sphere_with<P: Payload>(radius: f64) -> Result<Shape<SolidTag, P>, SphereBuildError> {
    sphere_at_with::<P>(Frame::xyz(), radius)
}

/// Creates a torus centered at the given frame origin.
///
/// The frame's z-axis is the revolution axis and its x-axis fixes where the
/// generating circle sits. `minor` is the tube's radius and `major` the distance
/// from the axis out to the tube's centre, so `minor` must stay under `major`.
pub fn torus_at(frame: Frame, major: f64, minor: f64) -> Result<Shape<SolidTag>, TorusBuildError> {
    torus_at_with::<StandardPayload>(frame, major, minor)
}

/// As [`torus_at`], with the payload chosen by the caller.
pub fn torus_at_with<P: Payload>(
    frame: Frame,
    major: f64,
    minor: f64,
) -> Result<Shape<SolidTag, P>, TorusBuildError> {
    Shape::build(|model| add_torus(model, frame, major, minor).map(|solid| solid.solid))
}

/// Creates a torus centered at the origin.
pub fn torus(major: f64, minor: f64) -> Result<Shape<SolidTag>, TorusBuildError> {
    torus_with::<StandardPayload>(major, minor)
}

/// As [`torus`], with the payload chosen by the caller.
pub fn torus_with<P: Payload>(
    major: f64,
    minor: f64,
) -> Result<Shape<SolidTag, P>, TorusBuildError> {
    torus_at_with::<P>(Frame::xyz(), major, minor)
}

/// Creates a solid by extruding the given face in the specified direction.
pub fn extruded<P: Payload>(
    face: Shape<FaceTag, P>,
    direction: Unit<Vector3<f64>>,
    distance: f64,
) -> Result<Shape<SolidTag, P>, ExtrudeError> {
    let direction = direction.into_inner() * distance;
    face.then(|model, face_key| {
        add_extruded_face(model, face_key, direction).map(|extrusion| extrusion.solid)
    })
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
    let (mut map, target) = target.into_model();
    let (tool_map, tool) = tool.into_model();
    let tool = map.transaction(|edit| {
        let handle = edit.merge(tool_map.solid_unchecked(tool));
        Ok::<_, ModelEditError>(
            edit.solid_key(handle)
                .expect("copied tool solid must retain its registration"),
        )
    })?;
    let result = boolean(&mut map, target, tool, operation, BooleanOptions::default())?;
    Ok(Shape::new(map, result.solid))
}

/// Copies two borrowed solid views for language bindings and evaluates a Boolean.
pub(crate) fn combine_views<P: Payload>(
    first: Solid<'_, P>,
    second: Solid<'_, P>,
    operation: BooleanOperation,
) -> Result<Shape<SolidTag, P>, BooleanError> {
    let mut map = Model::new();
    let (first, second) = map.transaction(|edit| {
        let first_handle = edit.merge(first);
        let second_handle = edit.merge(second);
        Ok::<_, ModelEditError>((
            edit.solid_key(first_handle)
                .expect("copied first solid must retain its registration"),
            edit.solid_key(second_handle)
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
