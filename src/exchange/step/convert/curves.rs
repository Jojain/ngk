//! Curves.
//!
//! Dispatch follows the crate's analytic-first convention: each reader and
//! each writer returns `Option<Result<..>>`, where `None` *declines* the
//! curve and lets the next one try. A kind no entry claims is refused by
//! name rather than approximated, so a new curve type is one entry appended
//! to a chain and no call site here changes.

use crate::geometry::{Circle, Curve, Ellipse, Line, Plane};

use super::super::builder::InstanceBuilder;
use super::super::error::{GeometryError, StepError};
use super::super::part21::EntityId;
use super::super::schema::entities;
use super::super::schema::resolver::{Attributes, Origin, Resolver};
use super::placement::{
    read_direction, read_placement, read_point, write_placement, write_point, write_vector,
};

/// Writes a curve, preferring its closed form.
pub fn write_curve(
    builder: &mut InstanceBuilder,
    curve: &Curve,
) -> Result<EntityId, GeometryError> {
    if let Some(written) = write_line(builder, curve) {
        return written;
    }
    if let Some(written) = write_circle(builder, curve) {
        return written;
    }
    if let Some(written) = write_ellipse(builder, curve) {
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

/// Writes a `CIRCLE`, or declines anything that is not one.
///
/// Both parameterizations are the angle swept from the placement's reference
/// direction about its axis, counter-clockwise, so the parameter carries
/// across untouched and an arc keeps its own interval.
fn write_circle(
    builder: &mut InstanceBuilder,
    curve: &Curve,
) -> Option<Result<EntityId, GeometryError>> {
    let Curve::Circle(circle) = curve else {
        return None;
    };

    let position = write_placement(builder, &circle.plane().frame);
    Some(Ok(builder.add_shared_entity(&entities::Circle {
        position,
        radius: circle.radius(),
    })))
}

/// Writes an `ELLIPSE`, or declines anything that is not one.
///
/// `semi_axis_1` is the radius along the placement's reference direction,
/// which is NGK's `major_radius` whether or not it is the longer of the two —
/// the schema orders the two axes by direction, not by size, so writing them
/// by size would rotate the parameterization a quarter turn.
fn write_ellipse(
    builder: &mut InstanceBuilder,
    curve: &Curve,
) -> Option<Result<EntityId, GeometryError>> {
    let Curve::Ellipse(ellipse) = curve else {
        return None;
    };

    let position = write_placement(builder, ellipse.frame());
    Some(Ok(builder.add_shared_entity(&entities::Ellipse {
        position,
        semi_axis_1: ellipse.major_radius(),
        semi_axis_2: ellipse.minor_radius(),
    })))
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
    if let Some(read) = read_circle(resolver, &curve) {
        return read;
    }
    if let Some(read) = read_ellipse(resolver, &curve) {
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

/// Reads a `CIRCLE`, or declines anything that is not one.
fn read_circle(
    resolver: &Resolver<'_>,
    curve: &Attributes<'_>,
) -> Option<Result<Curve, StepError>> {
    let circle = curve.decode::<entities::Circle>()?;
    Some(circle.map_err(StepError::from).and_then(|circle| {
        let frame = read_placement(resolver, curve.origin, circle.position)?;
        let radius = resolver.units().to_mm(circle.radius);
        Ok(Curve::Circle(Circle::new(Plane::from_frame(frame), radius)))
    }))
}

/// Reads an `ELLIPSE`, or declines anything that is not one.
fn read_ellipse(
    resolver: &Resolver<'_>,
    curve: &Attributes<'_>,
) -> Option<Result<Curve, StepError>> {
    let ellipse = curve.decode::<entities::Ellipse>()?;
    Some(ellipse.map_err(StepError::from).and_then(|ellipse| {
        let frame = read_placement(resolver, curve.origin, ellipse.position)?;
        let units = resolver.units();
        Ok(Curve::Ellipse(Ellipse::new(
            frame,
            units.to_mm(ellipse.semi_axis_1),
            units.to_mm(ellipse.semi_axis_2),
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
