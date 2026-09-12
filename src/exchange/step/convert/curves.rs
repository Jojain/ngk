//! Curves.
//!
//! Dispatch follows the crate's own analytic-first convention (D2): each
//! writer returns `Option<Result<..>>`, where `None` *declines* the curve and
//! lets the next writer try. Stage 6 appends a NURBS fallback that never
//! declines, at which point `UnsupportedCurve` becomes unreachable for
//! anything NGK can hold — and no call site here changes.

use crate::geometry::Curve;

use super::super::builder::InstanceBuilder;
use super::super::error::GeometryError;
use super::super::part21::{EntityId, Record, Value};
use super::placement::{write_point, write_vector};

/// Writes a curve, preferring its closed form.
pub fn write_curve(
    builder: &mut InstanceBuilder,
    curve: &Curve,
) -> Result<EntityId, GeometryError> {
    if let Some(written) = write_line(builder, curve) {
        return written;
    }
    Err(GeometryError::UnsupportedCurve { kind: kind(curve) })
}

/// Writes a `LINE`, or declines anything that is not one.
///
/// NGK's line is `origin + direction · (scale · t)` and STEP's is
/// `pnt + magnitude · dir · t`, so the two parameterizations agree exactly
/// once `magnitude` carries NGK's scale. Recovering that scale from the
/// curve's own `point_at` keeps it faithful without widening `Line`'s API.
fn write_line(
    builder: &mut InstanceBuilder,
    curve: &Curve,
) -> Option<Result<EntityId, GeometryError>> {
    let Curve::Line(line) = curve else {
        return None;
    };

    let base = line.point_at(0.0);
    let span = line.point_at(1.0) - base;
    let magnitude = span.norm();
    if magnitude == 0.0 {
        return Some(Err(GeometryError::DegenerateLine));
    }

    let point = write_point(builder, base);
    let vector = write_vector(builder, line.direction(), magnitude);
    Some(Ok(builder.add_shared(Record::new(
        "LINE",
        vec![
            Value::Text(String::new()),
            Value::Ref(point),
            Value::Ref(vector),
        ],
    ))))
}

/// Names a curve variant for an error message.
fn kind(curve: &Curve) -> &'static str {
    match curve {
        Curve::Line(_) => "Line",
        Curve::Circle(_) => "Circle",
        Curve::Ellipse(_) => "Ellipse",
        Curve::Nurbs(_) => "Nurbs",
    }
}
