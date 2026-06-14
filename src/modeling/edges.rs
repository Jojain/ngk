use crate::builders::edges::{add_arc, add_circle, add_line};
use crate::builders::errors::EdgeCreationError;
use crate::geometry::{Plane, Point3};
use crate::topology::gmap::GMap;
use crate::topology::payload::{Payload, StandardPayload};
use crate::topology::shape::{EdgeTag, ProfileTag, Shape};

pub fn line(
    start: Point3,
    end: Point3,
) -> Result<Shape<EdgeTag, StandardPayload>, EdgeCreationError> {
    let mut g = GMap::new();
    let (_, edge_key) = add_line(&mut g, start, end)?;
    Ok(Shape::new(g, edge_key))
}

pub fn arc(
    plane: Plane,
    radius: f64,
    start_angle: f64,
    end_angle: f64,
) -> Result<Shape<EdgeTag, StandardPayload>, EdgeCreationError> {
    let mut g = GMap::new();
    let (_, edge_key) = add_arc(&mut g, plane, radius, start_angle, end_angle)?;
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

impl<P: Payload> Shape<EdgeTag, P> {
    pub fn into_profile(self) -> Shape<ProfileTag, P> {
        let (g, edge_key) = self.into_map();
        let dart = g
            .edge_attr(edge_key)
            .expect("edge shape key must be in the map")
            .dart;
        Shape::new(g, dart)
    }
}
