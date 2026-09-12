//! Surfaces.
//!
//! Same declining dispatch as [`curves`](super::curves): a reader or writer
//! that does not recognize a surface returns `None` and the next one tries,
//! and a kind no entry claims is refused by name.
//!
//! A plane is an identity mapping in both geometry and parameterization —
//! `PLANE`'s position is its frame, and `(u, v)` are distances along the
//! frame's x and y — so nothing here converts a parameter.

use crate::geometry::{Plane, Surface};

use super::super::builder::InstanceBuilder;
use super::super::error::{GeometryError, StepError};
use super::super::part21::EntityId;
use super::super::schema::entities;
use super::super::schema::resolver::{Attributes, Origin, Resolver};
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
    Some(Ok(builder.add_shared_entity(&entities::Plane { position })))
}

/// Reads a surface, preferring its closed form.
pub fn read_surface(
    resolver: &Resolver<'_>,
    from: Origin,
    id: EntityId,
) -> Result<Surface, StepError> {
    let surface = resolver.attributes(from, id)?;
    if let Some(read) = read_plane(resolver, &surface) {
        return read;
    }
    Err(GeometryError::UnreadableSurface {
        keyword: surface.keyword().to_string(),
        origin: surface.origin,
    }
    .into())
}

/// Reads a `PLANE`, or declines anything that is not one.
fn read_plane(
    resolver: &Resolver<'_>,
    surface: &Attributes<'_>,
) -> Option<Result<Surface, StepError>> {
    let plane = surface.decode::<entities::Plane>()?;
    Some(plane.map_err(StepError::from).and_then(|plane| {
        let frame = read_placement(resolver, surface.origin, plane.position)?;
        Ok(Surface::Plane(Plane::from_frame(frame)))
    }))
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
