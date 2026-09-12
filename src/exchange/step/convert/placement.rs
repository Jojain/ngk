//! Points, directions and frames.
//!
//! `Frame::from_xz(location, ref_direction, axis)` and STEP's
//! `AXIS2_PLACEMENT_3D(name, location, axis, ref_direction)` are the same
//! construction with the last two arguments swapped — both take the normal and
//! an in-plane reference and derive the third axis — so this mapping is exact
//! in both directions and needs no orthonormalization of its own.
//!
//! This is also where the document's length unit is applied, and where it is
//! deliberately *not*: a position is scaled, a direction is a ratio and is
//! left alone, and a vector's magnitude is a length and is scaled.

use nalgebra::{UnitVector3, Vector3};

use crate::geometry::{Frame, Point3};

use super::super::builder::InstanceBuilder;
use super::super::part21::EntityId;
use super::super::schema::entities;
use super::super::schema::resolver::{Origin, Resolver, SchemaError};

/// Writes a `CARTESIAN_POINT`, shared with every other point at this position.
pub fn write_point(builder: &mut InstanceBuilder, point: Point3) -> EntityId {
    builder.add_shared_entity(&entities::CartesianPoint {
        coordinates: [point.x, point.y, point.z],
    })
}

/// Writes a `DIRECTION`, shared with every other direction that equals it.
pub fn write_direction(builder: &mut InstanceBuilder, direction: UnitVector3<f64>) -> EntityId {
    builder.add_shared_entity(&entities::Direction {
        direction_ratios: [direction.x, direction.y, direction.z],
    })
}

/// Writes a `VECTOR`: a direction carrying a magnitude.
pub fn write_vector(
    builder: &mut InstanceBuilder,
    direction: UnitVector3<f64>,
    magnitude: f64,
) -> EntityId {
    let orientation = write_direction(builder, direction);
    builder.add_shared_entity(&entities::Vector {
        orientation,
        magnitude,
    })
}

/// Writes an `AXIS2_PLACEMENT_3D` for a frame, shared by value.
pub fn write_placement(builder: &mut InstanceBuilder, frame: &Frame) -> EntityId {
    let placement = placement_of(builder, frame);
    builder.add_shared_entity(&placement)
}

/// Writes an `AXIS2_PLACEMENT_3D` under a name of its own.
///
/// For a placement that is a *datum* rather than a value — a representation's
/// own origin — which should not become some coplanar face's surface position
/// just because the coordinates agree.
pub fn write_unshared_placement(builder: &mut InstanceBuilder, frame: &Frame) -> EntityId {
    let placement = placement_of(builder, frame);
    builder.add_entity(&placement)
}

fn placement_of(builder: &mut InstanceBuilder, frame: &Frame) -> entities::Axis2Placement3d {
    entities::Axis2Placement3d {
        location: write_point(builder, frame.origin),
        axis: Some(write_direction(builder, frame.z_dir)),
        ref_direction: Some(write_direction(builder, frame.x_dir)),
    }
}

/// Reads a `CARTESIAN_POINT`, converted to millimetres.
pub fn read_point(
    resolver: &Resolver<'_>,
    from: Origin,
    id: EntityId,
) -> Result<Point3, SchemaError> {
    let point = resolver.read::<entities::CartesianPoint<3>>(from, id)?;
    let [x, y, z] = point.coordinates;
    let scale = resolver.units().length;
    Ok(Point3::new(x * scale, y * scale, z * scale))
}

/// Reads a `DIRECTION`.
///
/// Unscaled: a direction is a ratio, so the document's length unit has
/// nothing to say about it. A `DIRECTION` is not required to be written
/// normalized, so it is normalized here.
pub fn read_direction(
    resolver: &Resolver<'_>,
    from: Origin,
    id: EntityId,
) -> Result<UnitVector3<f64>, SchemaError> {
    let direction = resolver.read::<entities::Direction<3>>(from, id)?;
    let [x, y, z] = direction.direction_ratios;
    let vector = Vector3::new(x, y, z);
    if vector.norm() == 0.0 {
        return Err(SchemaError::UnreadableUnit {
            origin: direction.origin,
            detail: "a direction with no length has no orientation".to_string(),
        });
    }
    Ok(UnitVector3::new_normalize(vector))
}

/// Reads an `AXIS2_PLACEMENT_3D` as a frame.
///
/// `ref_direction` need not be perpendicular to `axis`, and STEP takes its
/// component in the plane. [`Frame::from_xz`] does exactly that, which is why
/// nothing is orthonormalized here.
pub fn read_placement(
    resolver: &Resolver<'_>,
    from: Origin,
    id: EntityId,
) -> Result<Frame, SchemaError> {
    let placement = resolver.read::<entities::Axis2Placement3d>(from, id)?;
    let location = read_point(resolver, placement.origin, placement.location)?;

    let axis = match placement.axis {
        Some(axis) => read_direction(resolver, placement.origin, axis)?,
        None => Vector3::z_axis(),
    };
    let reference = match placement.ref_direction {
        Some(reference) => read_direction(resolver, placement.origin, reference)?,
        None => any_perpendicular(axis),
    };

    Ok(Frame::from_xz(location, reference, axis))
}

/// Returns some unit vector perpendicular to `axis`.
///
/// An unset `ref_direction` leaves the choice to the reader: the schema
/// constrains the placement to be well formed and says nothing more. Crossing
/// with whichever global axis `axis` leans on least keeps the result far from
/// degenerate, so the normalization that follows is well conditioned whatever
/// direction arrives.
fn any_perpendicular(axis: UnitVector3<f64>) -> UnitVector3<f64> {
    let least = if axis.x.abs() <= axis.y.abs() && axis.x.abs() <= axis.z.abs() {
        Vector3::x()
    } else if axis.y.abs() <= axis.z.abs() {
        Vector3::y()
    } else {
        Vector3::z()
    };
    UnitVector3::new_normalize(axis.cross(&least))
}
