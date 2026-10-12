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
    modeling::import::import_tools,
    topology::{
        ModelEdit,
        payload::Payload,
        shape::{FaceTag, Shape, SolidTag},
        shape_keys::SolidKey,
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

/// Fuses every tool into `target`: `target ∪ t₁ ∪ … ∪ tₙ`.
///
/// Called as `fuse(target, [&tool])` for one tool. The result is built in
/// `target`'s model, kept whole; each tool contributes only its solid, copied
/// in. Only the final result must be one connected solid, so the tools may be
/// passed in any order.
pub fn fuse<'a, P: Payload>(
    target: Shape<SolidTag, P>,
    tools: impl IntoIterator<Item = &'a Shape<SolidTag, P>>,
) -> Result<Shape<SolidTag, P>, BooleanError> {
    combine(target, tools, BooleanOperation::Union)
}

/// Subtracts every tool from `target`: `target − (t₁ ∪ … ∪ tₙ)`.
///
/// Called as `cut(target, [&tool])` for one tool; see [`fuse`] for where the
/// result is built.
pub fn cut<'a, P: Payload>(
    target: Shape<SolidTag, P>,
    tools: impl IntoIterator<Item = &'a Shape<SolidTag, P>>,
) -> Result<Shape<SolidTag, P>, BooleanError> {
    combine(target, tools, BooleanOperation::Difference)
}

/// The volume `target` shares with every tool: `target ∩ t₁ ∩ … ∩ tₙ`.
///
/// Called as `intersect(target, [&tool])` for one tool; see [`fuse`] for where
/// the result is built.
pub fn intersect<'a, P: Payload>(
    target: Shape<SolidTag, P>,
    tools: impl IntoIterator<Item = &'a Shape<SolidTag, P>>,
) -> Result<Shape<SolidTag, P>, BooleanError> {
    combine(target, tools, BooleanOperation::Intersection)
}

/// Copies each tool's solid into `target`'s model and evaluates one Boolean
/// there.
fn combine<'a, P: Payload>(
    target: Shape<SolidTag, P>,
    tools: impl IntoIterator<Item = &'a Shape<SolidTag, P>>,
    operation: BooleanOperation,
) -> Result<Shape<SolidTag, P>, BooleanError> {
    let (mut model, target) = target.into_model();
    let tools = tools.into_iter().collect::<Vec<_>>();
    let tools = import_tools(
        &mut model,
        &tools,
        |a, b| std::ptr::eq(*a, *b),
        |edit, tool| copy_solid(edit, tool.solid()),
    )?;
    let result = boolean(
        &mut model,
        target,
        &tools,
        operation,
        BooleanOptions::default(),
    )?;
    Ok(Shape::new(model, result.solid))
}

/// Copies one solid, with the entities that bound it and nothing else of its model, and
/// returns its key in the model being edited.
pub(crate) fn copy_solid<P: Payload>(edit: &mut ModelEdit<'_, P>, solid: Solid<'_, P>) -> SolidKey {
    let handle = edit.merge(solid);
    edit.solid_key(handle)
        .expect("copied tool solid must retain its registration")
}
