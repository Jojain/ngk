//! Curves.
//!
//! Dispatch follows the crate's analytic-first convention: each reader and
//! each writer returns `Option<Result<..>>`, where `None` *declines* the
//! curve and lets the next one try. A kind no entry claims is refused by
//! name rather than approximated, so a new curve type is one entry appended
//! to a chain and no call site here changes.

use crate::geometry::{Curve, Line};

use super::super::builder::InstanceBuilder;
use super::super::error::{GeometryError, StepError};
use super::super::part21::EntityId;
use super::super::schema::entities;
use super::super::schema::resolver::{Attributes, Origin, Resolver};
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

    let pnt = write_point(builder, base);
    let dir = write_vector(builder, line.direction(), magnitude);
    Some(Ok(builder.add_shared_entity(&entities::Line { pnt, dir })))
}

/// Reads a curve, preferring its closed form.
///
/// A `SURFACE_CURVE` — or its `SEAM_CURVE` and `INTERSECTION_CURVE` subtypes
/// — is unwrapped first: each is a 3D curve with parameter curves hung off
/// it, and the parameter curves are not read here. On a plane they are
/// redundant, since projecting the 3D curve is exact and cheaper than
/// resolving them.
pub fn read_curve(resolver: &Resolver<'_>, from: Origin, id: EntityId) -> Result<Curve, StepError> {
    let curve = resolver.attributes(from, id)?;
    let curve = match unwrap_surface_curve(resolver, &curve)? {
        Some(unwrapped) => unwrapped,
        None => curve,
    };

    if let Some(read) = read_line(resolver, &curve) {
        return read;
    }
    Err(GeometryError::UnreadableCurve {
        keyword: curve.keyword().to_string(),
        origin: curve.origin,
    }
    .into())
}

/// Follows a curve-on-surface down to the 3D curve it describes.
fn unwrap_surface_curve<'a>(
    resolver: &Resolver<'a>,
    curve: &Attributes<'a>,
) -> Result<Option<Attributes<'a>>, StepError> {
    let Some(surface_curve) = curve.decode::<entities::SurfaceCurve>() else {
        return Ok(None);
    };
    let surface_curve = surface_curve?;
    Ok(Some(
        resolver.attributes(curve.origin, surface_curve.curve_3d)?,
    ))
}

/// Reads a `LINE`, or declines anything that is not one.
fn read_line(resolver: &Resolver<'_>, curve: &Attributes<'_>) -> Option<Result<Curve, StepError>> {
    let line = curve.decode::<entities::Line>()?;
    Some(line.map_err(StepError::from).and_then(|line| {
        let origin = curve.origin;
        let base = read_point(resolver, origin, line.pnt)?;
        let vector = resolver.read::<entities::Vector>(origin, line.dir)?;
        let direction = read_direction(resolver, vector.origin, vector.orientation)?;
        let magnitude = resolver.units().to_mm(vector.magnitude);
        if magnitude == 0.0 {
            return Err(GeometryError::DegenerateLine.into());
        }
        Ok(Curve::Line(Line::through(
            base,
            base + direction.into_inner() * magnitude,
        )))
    }))
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
