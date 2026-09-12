//! Points, directions and frames.
//!
//! `Frame::from_xz(location, ref_direction, axis)` and STEP's
//! `AXIS2_PLACEMENT_3D(name, location, axis, ref_direction)` are the same
//! construction with the last two arguments swapped — both take the normal and
//! an in-plane reference and derive the third axis — so this mapping is exact
//! in both directions and needs no orthonormalization of its own.

use nalgebra::{UnitVector3, Vector3};

use crate::geometry::{Frame, Point3};

use super::super::builder::InstanceBuilder;
use super::super::part21::{EntityId, Record, Value};
use super::super::schema::resolver::{Entity, Resolver, SchemaError};

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

/// Reads a `CARTESIAN_POINT`, converted to millimetres.
pub fn read_point(
    resolver: &Resolver<'_>,
    from: &Entity<'_>,
    id: EntityId,
) -> Result<Point3, SchemaError> {
    let point = resolver.follow_typed(from, id, "CARTESIAN_POINT")?;
    let [x, y, z] = point.reals::<3>(1)?;
    let scale = resolver.units().length;
    Ok(Point3::new(x * scale, y * scale, z * scale))
}

/// Reads a `DIRECTION`.
///
/// Unscaled, deliberately: a direction is a ratio, so the document's length
/// unit has nothing to say about it. A `DIRECTION` is not required to be
/// written normalized, so it is normalized here.
pub fn read_direction(
    resolver: &Resolver<'_>,
    from: &Entity<'_>,
    id: EntityId,
) -> Result<UnitVector3<f64>, SchemaError> {
    let direction = resolver.follow_typed(from, id, "DIRECTION")?;
    let [x, y, z] = direction.reals::<3>(1)?;
    let vector = Vector3::new(x, y, z);
    if vector.norm() == 0.0 {
        return Err(SchemaError::BadAttribute {
            id: direction.id,
            line: direction.line,
            keyword: direction.keyword().to_string(),
            index: 1,
            expected: "a direction with some length",
        });
    }
    Ok(UnitVector3::new_normalize(vector))
}

/// Reads an `AXIS2_PLACEMENT_3D` as a frame.
///
/// Both `axis` and `ref_direction` are optional in the schema. An unset axis
/// is the global z, and an unset reference is *any* direction perpendicular to
/// the axis — the schema says only that the placement must be well formed, so
/// picking one is the reader's job rather than a defect in the file.
///
/// `ref_direction` need not be perpendicular to `axis`, and STEP takes its
/// component in the plane. [`Frame::from_xz`] does exactly that, which is why
/// nothing is orthonormalized here.
pub fn read_placement(
    resolver: &Resolver<'_>,
    from: &Entity<'_>,
    id: EntityId,
) -> Result<Frame, SchemaError> {
    let placement = resolver.follow_typed(from, id, "AXIS2_PLACEMENT_3D")?;
    let location = read_point(resolver, &placement, placement.reference(1)?)?;

    let axis = match placement.optional_reference(2)? {
        Some(axis) => read_direction(resolver, &placement, axis)?,
        None => Vector3::z_axis(),
    };
    let reference = match placement.optional_reference(3)? {
        Some(reference) => read_direction(resolver, &placement, reference)?,
        None => any_perpendicular(axis),
    };

    Ok(Frame::from_xz(location, reference, axis))
}

/// Returns some unit vector perpendicular to `axis`.
///
/// Crossing with whichever global axis `axis` leans on least keeps the result
/// far from degenerate, so the normalization that follows is well conditioned
/// whatever direction arrives.
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
