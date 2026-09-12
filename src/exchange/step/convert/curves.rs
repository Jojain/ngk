//! Curves.
//!
//! Dispatch follows the crate's own analytic-first convention (D2): each
//! writer returns `Option<Result<..>>`, where `None` *declines* the curve and
//! lets the next writer try. Stage 6 appends a NURBS fallback that never
//! declines, at which point `UnsupportedCurve` becomes unreachable for
//! anything NGK can hold — and no call site here changes.

use crate::geometry::{Curve, Line};

use super::super::builder::InstanceBuilder;
use super::super::error::{GeometryError, StepError};
use super::super::part21::{EntityId, Record, Value};
use super::super::schema::resolver::{Entity, Resolver};
use super::placement::{read_direction, read_point, write_point, write_vector};

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

/// Reads a curve, preferring its closed form.
///
/// A `SURFACE_CURVE` or `SEAM_CURVE` is unwrapped first: both are a 3D curve
/// with parameter curves hung off it, and the pcurves are not read here. On a
/// plane they are redundant — the projection is exact and cheaper to redo than
/// to resolve — and on a curved support they are stage 4's business.
pub fn read_curve(
    resolver: &Resolver<'_>,
    from: &Entity<'_>,
    id: EntityId,
) -> Result<Curve, StepError> {
    let curve = resolver.follow(from, id)?;
    let curve = match unwrap_surface_curve(resolver, &curve)? {
        Some(unwrapped) => unwrapped,
        None => curve,
    };

    if let Some(read) = read_line(resolver, &curve) {
        return read;
    }
    Err(GeometryError::UnreadableCurve {
        keyword: curve.keyword().to_string(),
        id: curve.id,
        line: curve.line,
    }
    .into())
}

/// Follows a `SURFACE_CURVE` down to the 3D curve it describes.
fn unwrap_surface_curve<'a>(
    resolver: &Resolver<'a>,
    curve: &Entity<'a>,
) -> Result<Option<Entity<'a>>, StepError> {
    if !curve.is("SURFACE_CURVE") && !curve.is("SEAM_CURVE") && !curve.is("INTERSECTION_CURVE") {
        return Ok(None);
    }
    let geometry = curve.reference(1)?;
    Ok(Some(resolver.follow(curve, geometry)?))
}

/// Reads a `LINE`, or declines anything that is not one.
///
/// STEP's line is `pnt + magnitude · dir · t` and NGK's is
/// `origin + direction · (scale · t)`, so the parameterizations agree exactly
/// once `magnitude` becomes NGK's scale — which is what anchoring the line
/// through `pnt` and `pnt + magnitude · dir` does, since
/// [`Line::through`](crate::geometry::Line::through) puts `t = 1` at its end
/// point.
fn read_line(resolver: &Resolver<'_>, curve: &Entity<'_>) -> Option<Result<Curve, StepError>> {
    if !curve.is("LINE") {
        return None;
    }
    Some(read_line_inner(resolver, curve))
}

fn read_line_inner(resolver: &Resolver<'_>, curve: &Entity<'_>) -> Result<Curve, StepError> {
    let origin = read_point(resolver, curve, curve.reference(1)?)?;
    let vector = resolver.follow_typed(curve, curve.reference(2)?, "VECTOR")?;
    let direction = read_direction(resolver, &vector, vector.reference(1)?)?;
    let magnitude = resolver.units().to_mm(vector.real(2)?);
    if magnitude == 0.0 {
        return Err(GeometryError::DegenerateLine.into());
    }
    Ok(Curve::Line(Line::through(
        origin,
        origin + direction.into_inner() * magnitude,
    )))
}
