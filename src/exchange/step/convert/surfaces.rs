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

use crate::geometry::Surface;

use super::super::builder::InstanceBuilder;
use super::super::error::GeometryError;
use super::super::part21::{EntityId, Record, Value};
use super::placement::write_placement;

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
