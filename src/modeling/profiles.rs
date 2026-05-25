use crate::builders::profiles::{PolylineError, add_polyline, add_rectangle, add_square};
use crate::geometry::{Curve, Plane, Point3};
use crate::topology::gmap::GMap;
use crate::topology::payload::StandardPayload;
use crate::topology::shape::{ProfileTag, Shape};

pub fn rectangle(
    plane: Plane,
    x_size: f64,
    y_size: f64,
) -> Result<Shape<ProfileTag, StandardPayload>, PolylineError> {
    let mut g = GMap::new();
    let profile_dart = add_rectangle(&mut g, plane, x_size, y_size)?;
    Ok(Shape::new(g, profile_dart))
}

pub fn square(
    plane: Plane,
    size: f64,
) -> Result<Shape<ProfileTag, StandardPayload>, PolylineError> {
    let mut g = GMap::new();
    let handle = add_square(&mut g, plane, size)?;
    Ok(Shape::new(g, handle))
}

pub fn polyline(
    segments: &[(Point3, Point3, Curve)],
) -> Result<Shape<ProfileTag, StandardPayload>, PolylineError> {
    let mut g = GMap::new();
    let profile_dart = add_polyline(&mut g, segments)?;
    Ok(Shape::new(g, profile_dart))
}
