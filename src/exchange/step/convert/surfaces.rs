//! Surfaces.
//!
//! Same declining dispatch as [`curves`](super::curves), and the same
//! extension point: stage 6's NURBS fallback is appended to the chain in
//! [`write_surface`] and nothing else moves.
//!
//! A plane is an identity mapping in both geometry and parameterization —
//! `PLANE`'s position is its frame, and `(u, v)` are distances along the
//! frame's x and y. The surfaces that *do not* share STEP's parameterization
//! (a cone's `v`, a revolution's transposition) arrive in stage 4 with the
//! `UvMap` that makes their sign reasoning mechanical.

use crate::geometry::{Plane, Surface};

use super::super::builder::InstanceBuilder;
use super::super::error::{GeometryError, StepError};
use super::super::part21::{EntityId, Record, Value};
use super::super::schema::resolver::{Entity, Resolver};
use super::placement::{read_placement, write_placement};

/// Writes a surface, preferring its closed form.
pub fn write_surface(
    builder: &mut InstanceBuilder,
    surface: &Surface,
) -> Result<EntityId, GeometryError> {
    if let Some(written) = write_plane(builder, surface) {
        return written;
    }
    Err(GeometryError::UnsupportedSurface {
        kind: kind(surface),
    })
}

/// Writes a `PLANE`, or declines anything that is not one.
fn write_plane(
    builder: &mut InstanceBuilder,
    surface: &Surface,
) -> Option<Result<EntityId, GeometryError>> {
    let Surface::Plane(plane) = surface else {
        return None;
    };

    let position = write_placement(builder, &plane.frame);
    Some(Ok(builder.add_shared(Record::new(
        "PLANE",
        vec![Value::Text(String::new()), Value::Ref(position)],
    ))))
}

/// Names a surface variant for an error message.
fn kind(surface: &Surface) -> &'static str {
    match surface {
        Surface::Plane(_) => "Plane",
        Surface::Cylinder(_) => "Cylinder",
        Surface::Sphere(_) => "Sphere",
        Surface::Cone(_) => "Cone",
        Surface::Torus(_) => "Torus",
        Surface::Ruled(_) => "Ruled",
        Surface::Revolution(_) => "Revolution",
        Surface::Nurbs(_) => "Nurbs",
    }
}

/// Reads a surface, preferring its closed form.
///
/// The same declining chain as [`write_surface`], read backwards: each reader
/// returns `None` for an entity it does not recognize and the next one tries.
/// Stage 6 appends the `B_SPLINE_SURFACE` reader and the certified NURBS
/// conversion that never declines.
pub fn read_surface(
    resolver: &Resolver<'_>,
    from: &Entity<'_>,
    id: EntityId,
) -> Result<Surface, StepError> {
    let surface = resolver.follow(from, id)?;
    if let Some(read) = read_plane(resolver, &surface) {
        return read;
    }
    Err(GeometryError::UnreadableSurface {
        keyword: surface.keyword().to_string(),
        id: surface.id,
        line: surface.line,
    }
    .into())
}

/// Reads a `PLANE`, or declines anything that is not one.
///
/// The mapping is an identity in geometry *and* in parameterization: STEP's
/// `(u, v)` are distances along the placement's x and y, which is exactly what
/// [`Plane::point_at`](crate::geometry::Plane::point_at) computes. That is why
/// stage 3 needs no `UvMap` — a plane is the one surface that does not.
fn read_plane(resolver: &Resolver<'_>, surface: &Entity<'_>) -> Option<Result<Surface, StepError>> {
    if !surface.is("PLANE") {
        return None;
    }
    Some(
        surface
            .reference(1)
            .map_err(StepError::from)
            .and_then(|position| {
                let frame = read_placement(resolver, surface, position)?;
                Ok(Surface::Plane(Plane::from_frame(frame)))
            }),
    )
}
