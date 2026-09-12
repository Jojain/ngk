//! Surfaces.
//!
//! Same declining dispatch as [`curves`](super::curves): a reader or writer
//! that does not recognize a surface returns `None` and the next one tries,
//! and a kind no entry claims is refused by name.
//!
//! **Every reader states its parameter map.** Most of these surfaces are
//! parameterized exactly as STEP parameterizes them, but a cone is not, and
//! the difference is scale-bearing — so a surface never arrives on its own.
//! It arrives as a [`MappedSurface`], which carries the surface together with
//! the [`UvMap`] that turns the file's parameters into the ones it answers to,
//! and the two cannot drift apart because they are one value.

use std::f64::consts::{FRAC_PI_2, PI};

use crate::geometry::{ANGULAR_TOLERANCE, Cone, Cylinder, Frame, Plane, Sphere, Surface, Torus};

use super::super::builder::InstanceBuilder;
use super::super::error::{GeometryError, StepError};
use super::super::part21::EntityId;
use super::super::schema::entities;
use super::super::schema::resolver::{Attributes, Origin, Resolver};
use super::placement::{read_placement, write_placement};
use super::uv_map::UvMap;

/// A surface and the change of parameters between its STEP spelling and it.
///
/// One value rather than two returns, because a caller that has the surface
/// and not the map has everything it needs to place a parameter curve in the
/// wrong chart, and nothing to warn it.
#[derive(Debug, Clone, PartialEq)]
pub struct MappedSurface {
    /// The support the entity describes.
    pub surface: Surface,
    /// How to read the entity's parameters in the support's own chart.
    pub map: UvMap,
}

impl MappedSurface {
    /// A surface NGK parameterizes exactly as STEP does.
    fn identical(surface: Surface) -> Self {
        Self {
            surface,
            map: UvMap::IDENTITY,
        }
    }
}

/// Writes a surface, preferring its closed form.
pub fn write_surface(
    builder: &mut InstanceBuilder,
    surface: &Surface,
) -> Result<EntityId, GeometryError> {
    let writers = [
        write_plane,
        write_cylindrical_surface,
        write_spherical_surface,
        write_conical_surface,
        write_toroidal_surface,
    ];
    for writer in writers {
        if let Some(written) = writer(builder, surface) {
            return written;
        }
    }
    Err(GeometryError::UnsupportedSurface {
        kind: kind(surface),
    })
}

/// Writes a `PLANE`, or declines anything that is not one.
///
/// A plane is an identity mapping in both geometry and parameterization —
/// `PLANE`'s position is its frame, and `(u, v)` are distances along the
/// frame's x and y — so nothing here converts a parameter.
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

/// Writes a `CYLINDRICAL_SURFACE`, or declines anything that is not one.
///
/// `u` is the angle about the axis and `v` the distance along it in both, so
/// the placement carries the whole conversion.
fn write_cylindrical_surface(
    builder: &mut InstanceBuilder,
    surface: &Surface,
) -> Option<Result<EntityId, GeometryError>> {
    let Surface::Cylinder(cylinder) = surface else {
        return None;
    };

    let position = write_placement(builder, &cylinder.frame);
    Some(Ok(builder.add_shared_entity(
        &entities::CylindricalSurface {
            position,
            radius: cylinder.radius,
        },
    )))
}

/// Writes a `SPHERICAL_SURFACE`, or declines anything that is not one.
///
/// `u` is longitude from the placement's reference direction and `v` latitude
/// from its equator in both.
fn write_spherical_surface(
    builder: &mut InstanceBuilder,
    surface: &Surface,
) -> Option<Result<EntityId, GeometryError>> {
    let Surface::Sphere(sphere) = surface else {
        return None;
    };

    let position = write_placement(builder, sphere.frame());
    Some(Ok(builder.add_shared_entity(&entities::SphericalSurface {
        position,
        radius: sphere.radius(),
    })))
}

/// Writes a `TOROIDAL_SURFACE`, or declines anything that is not one.
///
/// Verified term for term against ISO 10303-42's `toroidal_surface`:
/// `σ(u,v) = C + (R + r·cos v)·(cos u·x + sin u·y) + r·sin v·z`, which is
/// [`Torus::point_at`] exactly, with `u` the major angle and `v` the tube
/// angle. The identity is why a torus needs no parameter map at all.
fn write_toroidal_surface(
    builder: &mut InstanceBuilder,
    surface: &Surface,
) -> Option<Result<EntityId, GeometryError>> {
    let Surface::Torus(torus) = surface else {
        return None;
    };

    let position = write_placement(builder, torus.frame());
    Some(Ok(builder.add_shared_entity(&entities::ToroidalSurface {
        position,
        major_radius: torus.major_radius(),
        minor_radius: torus.minor_radius(),
    })))
}

/// Writes a `CONICAL_SURFACE`, or declines anything that is not one.
///
/// The schema requires a semi-angle strictly between zero and a right angle,
/// which NGK's half angle is not: it comes from the generatrix's own direction
/// and spans a full turn. Two normalizations bring any of them into range
/// without moving the surface, and both are applied to the *placement*, so the
/// entity describes the same point set whichever way the profile was drawn.
///
/// A half angle past a right angle names the generatrix running backwards
/// along the axis. It is the same line, so replacing the direction by its
/// opposite changes nothing but the sign of the parameter — which this entity
/// does not carry. A negative half angle names a cone narrowing along the
/// axis, which STEP spells by looking at it from the other end.
fn write_conical_surface(
    builder: &mut InstanceBuilder,
    surface: &Surface,
) -> Option<Result<EntityId, GeometryError>> {
    let Surface::Cone(cone) = surface else {
        return None;
    };

    let mut frame = cone.frame().clone();
    let mut semi_angle = cone.half_angle();
    if semi_angle.cos() < 0.0 {
        semi_angle -= PI * semi_angle.signum();
    }
    if semi_angle.abs() <= ANGULAR_TOLERANCE || semi_angle.abs() >= FRAC_PI_2 - ANGULAR_TOLERANCE {
        return Some(Err(GeometryError::DegenerateCone {
            half_angle: cone.half_angle(),
        }));
    }
    if semi_angle < 0.0 {
        frame = Frame::from_xz(frame.origin, frame.x_dir, -frame.z_dir);
        semi_angle = -semi_angle;
    }

    let position = write_placement(builder, &frame);
    Some(Ok(builder.add_shared_entity(&entities::ConicalSurface {
        position,
        radius: cone.reference_radius(),
        semi_angle,
    })))
}

/// Reads a surface, preferring its closed form.
pub fn read_surface(
    resolver: &Resolver<'_>,
    from: Origin,
    id: EntityId,
) -> Result<MappedSurface, StepError> {
    let surface = resolver.attributes(from, id)?;
    let readers = [
        read_plane,
        read_cylindrical_surface,
        read_spherical_surface,
        read_conical_surface,
        read_toroidal_surface,
    ];
    for reader in readers {
        if let Some(read) = reader(resolver, &surface) {
            return read;
        }
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
) -> Option<Result<MappedSurface, StepError>> {
    let plane = surface.decode::<entities::Plane>()?;
    Some(plane.map_err(StepError::from).and_then(|plane| {
        let frame = read_placement(resolver, surface.origin, plane.position)?;
        Ok(MappedSurface::identical(Surface::Plane(Plane::from_frame(
            frame,
        ))))
    }))
}

/// Reads a `CYLINDRICAL_SURFACE`, or declines anything that is not one.
fn read_cylindrical_surface(
    resolver: &Resolver<'_>,
    surface: &Attributes<'_>,
) -> Option<Result<MappedSurface, StepError>> {
    let cylinder = surface.decode::<entities::CylindricalSurface>()?;
    Some(cylinder.map_err(StepError::from).and_then(|cylinder| {
        let frame = read_placement(resolver, surface.origin, cylinder.position)?;
        Ok(MappedSurface::identical(Surface::Cylinder(Cylinder {
            frame,
            radius: resolver.units().to_mm(cylinder.radius),
        })))
    }))
}

/// Reads a `SPHERICAL_SURFACE`, or declines anything that is not one.
fn read_spherical_surface(
    resolver: &Resolver<'_>,
    surface: &Attributes<'_>,
) -> Option<Result<MappedSurface, StepError>> {
    let sphere = surface.decode::<entities::SphericalSurface>()?;
    Some(sphere.map_err(StepError::from).and_then(|sphere| {
        let frame = read_placement(resolver, surface.origin, sphere.position)?;
        Ok(MappedSurface::identical(Surface::Sphere(Sphere::new(
            frame,
            resolver.units().to_mm(sphere.radius),
        ))))
    }))
}

/// Reads a `TOROIDAL_SURFACE`, or declines anything that is not one.
fn read_toroidal_surface(
    resolver: &Resolver<'_>,
    surface: &Attributes<'_>,
) -> Option<Result<MappedSurface, StepError>> {
    let torus = surface.decode::<entities::ToroidalSurface>()?;
    Some(torus.map_err(StepError::from).and_then(|torus| {
        let frame = read_placement(resolver, surface.origin, torus.position)?;
        let units = resolver.units();
        Ok(MappedSurface::identical(Surface::Torus(Torus::new(
            frame,
            units.to_mm(torus.major_radius),
            units.to_mm(torus.minor_radius),
        ))))
    }))
}

/// Reads a `CONICAL_SURFACE`, or declines anything that is not one.
///
/// This is the one surface here whose parameterization differs: STEP measures
/// `v` along the axis, NGK along the generatrix, so `v_ngk = v_step / cos α`.
/// That substitution reproduces STEP's `radius = R + v·tan α` and `height = v`
/// exactly, and it is the whole of the difference — `u` is the same angle in
/// both.
fn read_conical_surface(
    resolver: &Resolver<'_>,
    surface: &Attributes<'_>,
) -> Option<Result<MappedSurface, StepError>> {
    let cone = surface.decode::<entities::ConicalSurface>()?;
    Some(cone.map_err(StepError::from).and_then(|cone| {
        let frame = read_placement(resolver, surface.origin, cone.position)?;
        let units = resolver.units();
        let half_angle = units.to_radians(cone.semi_angle);
        // The schema requires a semi-angle in the open interval `(0, π/2)`; at
        // either end the substitution below has no finite answer, and the
        // surface is a cylinder or a half-line rather than a cone.
        if half_angle.cos().abs() <= ANGULAR_TOLERANCE {
            return Err(GeometryError::DegenerateCone { half_angle }.into());
        }
        Ok(MappedSurface {
            surface: Surface::Cone(Cone::new(frame, units.to_mm(cone.radius), half_angle)),
            map: UvMap::scaled(1.0, 1.0 / half_angle.cos()),
        })
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
