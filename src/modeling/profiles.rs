use crate::builders::profiles::{PolylineError, add_polyline, add_rectangle};
use crate::geometry::{Curve, Plane, Point3};
use crate::topology::gmap::GMap;
use crate::topology::payload::StandardPayload;
use crate::topology::shape::{ProfileTag, Shape};

pub fn polyline(
    segments: &[(Point3, Point3, Curve)],
) -> Result<Shape<ProfileTag, StandardPayload>, PolylineError> {
    let mut g = GMap::new();
    let profile_dart = add_polyline(&mut g, segments)?;
    Ok(Shape::new(g, profile_dart))
}

pub fn rectangle(
    plane: Plane,
    x_size: f64,
    y_size: f64,
) -> Result<Shape<ProfileTag, StandardPayload>, PolylineError> {
    let mut g = GMap::new();
    let profile_dart = add_rectangle(&mut g, plane, x_size, y_size)?;
    Ok(Shape::new(g, profile_dart))
}
