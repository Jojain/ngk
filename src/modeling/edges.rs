use crate::builders::edges::{add_circle, add_line};
use crate::builders::errors::EdgeCreationError;
use crate::geometry::{Plane, Point3};
use crate::topology::gmap::GMap;
use crate::topology::payload::StandardPayload;
use crate::topology::shape::{EdgeTag, Shape};

pub fn line(
    start: Point3,
    end: Point3,
) -> Result<Shape<EdgeTag, StandardPayload>, EdgeCreationError> {
    let mut g = GMap::new();
    let (_, edge_key) = add_line(&mut g, start, end)?;
    Ok(Shape::new(g, edge_key))
}

pub fn circle(
    plane: Plane,
    radius: f64,
) -> Result<Shape<EdgeTag, StandardPayload>, EdgeCreationError> {
    let mut g = GMap::new();
    let (_, edge_key) = add_circle(&mut g, plane, radius)?;
    Ok(Shape::new(g, edge_key))
}
