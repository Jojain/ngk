//! Points, directions and frames.
//!
//! `Frame::from_xz(location, ref_direction, axis)` and STEP's
//! `AXIS2_PLACEMENT_3D(name, location, axis, ref_direction)` are the same
//! construction with the last two arguments swapped — both take the normal and
//! an in-plane reference and derive the third axis — so this mapping is exact
//! in both directions and needs no orthonormalization of its own.

use nalgebra::UnitVector3;

use crate::geometry::{Frame, Point3};

use super::super::builder::InstanceBuilder;
use super::super::part21::{EntityId, Record, Value};

/// Writes a `CARTESIAN_POINT`, shared with every other point at this position.
pub fn write_point(builder: &mut InstanceBuilder, point: Point3) -> EntityId {
    builder.add_shared(Record::new(
        "CARTESIAN_POINT",
        vec![
            Value::Text(String::new()),
            Value::List(vec![
                Value::Real(point.x),
                Value::Real(point.y),
                Value::Real(point.z),
            ]),
        ],
    ))
}

/// Writes a `DIRECTION`, shared with every other direction that equals it.
pub fn write_direction(builder: &mut InstanceBuilder, direction: UnitVector3<f64>) -> EntityId {
    builder.add_shared(Record::new(
        "DIRECTION",
        vec![
            Value::Text(String::new()),
            Value::List(vec![
                Value::Real(direction.x),
                Value::Real(direction.y),
                Value::Real(direction.z),
            ]),
        ],
    ))
}

/// Writes a `VECTOR`: a direction carrying a magnitude.
pub fn write_vector(
    builder: &mut InstanceBuilder,
    direction: UnitVector3<f64>,
    magnitude: f64,
) -> EntityId {
    let direction = write_direction(builder, direction);
    builder.add_shared(Record::new(
        "VECTOR",
        vec![
            Value::Text(String::new()),
            Value::Ref(direction),
            Value::Real(magnitude),
        ],
    ))
}

/// Writes an `AXIS2_PLACEMENT_3D` for a frame, shared by value.
///
/// STEP names the normal `axis` and the in-plane reference `ref_direction`,
/// in that order — the reverse of the order [`Frame::from_xz`] takes them.
pub fn write_placement(builder: &mut InstanceBuilder, frame: &Frame) -> EntityId {
    let record = placement_record(builder, frame);
    builder.add_shared(record)
}

/// Writes an `AXIS2_PLACEMENT_3D` under a name of its own.
///
/// For a placement that is a *datum* rather than a value — a representation's
/// own origin — which should not become some coplanar face's surface position
/// just because the coordinates agree.
pub fn write_unshared_placement(builder: &mut InstanceBuilder, frame: &Frame) -> EntityId {
    let record = placement_record(builder, frame);
    builder.add(record)
}

fn placement_record(builder: &mut InstanceBuilder, frame: &Frame) -> Record {
    let location = write_point(builder, frame.origin);
    let axis = write_direction(builder, frame.z_dir);
    let reference = write_direction(builder, frame.x_dir);
    Record::new(
        "AXIS2_PLACEMENT_3D",
        vec![
            Value::Text(String::new()),
            Value::Ref(location),
            Value::Ref(axis),
            Value::Ref(reference),
        ],
    )
}
