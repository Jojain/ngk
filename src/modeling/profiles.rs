use crate::builders::errors::EdgeCreationError;
use crate::builders::profiles::{
    PolylineError, add_edge_to_profile, add_polyline, add_rectangle, add_square,
};
use crate::geometry::{Plane, Point3};
use crate::modeling::edges;
use crate::topology::closed::Closeable;
use crate::topology::gmap::GMap;
use crate::topology::payload::{Payload, StandardPayload};
use crate::topology::shape::{EdgeTag, ProfileTag, Shape};

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

pub fn polyline(points: &[Point3]) -> Result<Shape<ProfileTag, StandardPayload>, PolylineError> {
    let mut g = GMap::new();
    let profile_dart = add_polyline(&mut g, points)?;
    Ok(Shape::new(g, profile_dart))
}

pub fn polygon(points: &[Point3]) -> Result<Shape<ProfileTag, StandardPayload>, PolylineError> {
    let mut closed_points = points.to_vec();
    let first = points.first().ok_or(PolylineError::InvalidPolygon {
        point_count: points.len(),
    })?;
    if points.len() < 3 {
        return Err(PolylineError::InvalidPolygon {
            point_count: points.len(),
        });
    }
    closed_points.push(*first);
    polyline(&closed_points)
}

pub fn arc(
    plane: Plane,
    radius: f64,
    start_angle: f64,
    end_angle: f64,
) -> Result<Shape<ProfileTag, StandardPayload>, EdgeCreationError> {
    Ok(edges::arc(plane, radius, start_angle, end_angle)?.into_profile())
}

impl<P: Payload> Shape<ProfileTag, P> {
    pub fn add(&mut self, edge: &Shape<EdgeTag, P>) -> Result<(), PolylineError> {
        let profile_dart = self.profile().dart;
        if self.profile().is_closed() {
            return Err(PolylineError::ClosedProfile { dart: profile_dart });
        }

        let edge_dart = self.map_mut().merge(edge.edge());
        add_edge_to_profile(self.map_mut(), profile_dart, edge_dart)
    }
}
