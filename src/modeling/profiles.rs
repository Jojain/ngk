use radians::Rad64;

use crate::builders::errors::EdgeCreationError;
use crate::builders::profiles::{
    PolylineError, add_polyline, add_profile_from_edges_edit, add_rectangle, add_square,
    append_edge_edit,
};
use crate::geometry::{Plane, Point3};
use crate::model::{Cell1, Model};
use crate::modeling::edges;
use crate::topology::closed::Closeable;
use crate::topology::payload::{Payload, StandardPayload};
use crate::topology::shape::{EdgeTag, ProfileTag, Shape};

/// Creates a closed rectangular profile whose first corner is `plane.origin()`.
pub fn rectangle(
    plane: Plane,
    x_size: f64,
    y_size: f64,
) -> Result<Shape<ProfileTag>, PolylineError> {
    rectangle_with::<StandardPayload>(plane, x_size, y_size)
}

/// As [`rectangle`], with the payload chosen by the caller.
pub fn rectangle_with<P: Payload>(
    plane: Plane,
    x_size: f64,
    y_size: f64,
) -> Result<Shape<ProfileTag, P>, PolylineError> {
    Shape::build(|model| add_rectangle(model, plane, x_size, y_size))
}

/// Creates a closed square profile whose first corner is `plane.origin()`.
pub fn square(plane: Plane, size: f64) -> Result<Shape<ProfileTag>, PolylineError> {
    square_with::<StandardPayload>(plane, size)
}

/// As [`square`], with the payload chosen by the caller.
pub fn square_with<P: Payload>(
    plane: Plane,
    size: f64,
) -> Result<Shape<ProfileTag, P>, PolylineError> {
    Shape::build(|model| add_square(model, plane, size))
}

/// Creates an open or closed profile by joining the supplied points in order.
pub fn polyline(points: &[Point3]) -> Result<Shape<ProfileTag>, PolylineError> {
    polyline_with::<StandardPayload>(points)
}

/// As [`polyline`], with the payload chosen by the caller.
pub fn polyline_with<P: Payload>(points: &[Point3]) -> Result<Shape<ProfileTag, P>, PolylineError> {
    Shape::build(|model| add_polyline(model, points))
}

/// Builds an owned profile from connected edge shapes in any input order.
///
/// The returned shape owns copies of the input edges. The source shapes remain
/// unchanged. The edges must make one non-branching chain or cycle.
pub fn from_edges<P: Payload>(
    edges: &[&Shape<EdgeTag, P>],
) -> Result<Shape<ProfileTag, P>, PolylineError> {
    let mut g = Model::new();
    let profile = g.transaction(|edit| {
        let keys = edges
            .iter()
            .map(|edge| {
                let dart = edit.merge(edge.edge());
                edit.cell_key_unchecked::<Cell1>(dart)
            })
            .collect::<Vec<_>>();
        add_profile_from_edges_edit(edit, &keys)
    })?;
    Ok(Shape::new(g, profile))
}

/// Creates a closed polygon profile from the supplied corners.
pub fn polygon(points: &[Point3]) -> Result<Shape<ProfileTag>, PolylineError> {
    polygon_with::<StandardPayload>(points)
}

/// As [`polygon`], with the payload chosen by the caller.
pub fn polygon_with<P: Payload>(points: &[Point3]) -> Result<Shape<ProfileTag, P>, PolylineError> {
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
    polyline_with::<P>(&closed_points)
}

/// Creates a circular arc profile in 3D space.
pub fn arc(
    plane: Plane,
    radius: f64,
    start_angle: Rad64,
    end_angle: Rad64,
) -> Result<Shape<ProfileTag>, EdgeCreationError> {
    arc_with::<StandardPayload>(plane, radius, start_angle, end_angle)
}

/// As [`arc`], with the payload chosen by the caller.
pub fn arc_with<P: Payload>(
    plane: Plane,
    radius: f64,
    start_angle: Rad64,
    end_angle: Rad64,
) -> Result<Shape<ProfileTag, P>, EdgeCreationError> {
    Ok(edges::arc_with::<P>(plane, radius, start_angle, end_angle)?.into_profile())
}

impl<P: Payload> Shape<ProfileTag, P> {
    pub fn add(&mut self, edge: &Shape<EdgeTag, P>) -> Result<(), PolylineError> {
        let profile_key = self.handle();
        let profile_dart = self.profile().dart;
        if self.profile().is_closed() {
            return Err(PolylineError::ClosedProfile { dart: profile_dart });
        }

        self.model_mut().transaction(|edit| {
            let edge_dart = edit.merge(edge.edge());
            let edge_key = edit.cell_key_unchecked::<Cell1>(edge_dart);
            append_edge_edit(edit, profile_key, edge_key).map(|_| ())
        })
    }
}
