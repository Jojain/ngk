use radians::Rad64;

use crate::builders::edges::{add_arc, add_circle, add_helix, add_line};
use crate::builders::errors::EdgeCreationError;
use crate::builders::profiles::add_profile_from_edges;
use crate::geometry::{Axis3, Plane, Point3};
use crate::topology::ModelEditError;
use crate::topology::payload::{Payload, StandardPayload};
use crate::topology::shape::{EdgeTag, ProfileTag, Shape};

/// Creates a line segment between two points in 3D space.
pub fn line(
    start: Point3,
    end: Point3,
) -> Result<Shape<EdgeTag, StandardPayload>, EdgeCreationError> {
    line_with::<StandardPayload>(start, end)
}

/// As [`line`], with the payload chosen by the caller.
pub fn line_with<P: Payload>(
    start: Point3,
    end: Point3,
) -> Result<Shape<EdgeTag, P>, EdgeCreationError> {
    Shape::build(|model| add_line(model, start, end))
}

/// Creates an arc of a circle in 3D space defined by a plane, radius, and start/end angles.
pub fn arc(
    plane: Plane,
    radius: f64,
    start_angle: Rad64,
    end_angle: Rad64,
) -> Result<Shape<EdgeTag, StandardPayload>, EdgeCreationError> {
    arc_with::<StandardPayload>(plane, radius, start_angle, end_angle)
}

/// As [`arc`], with the payload chosen by the caller.
pub fn arc_with<P: Payload>(
    plane: Plane,
    radius: f64,
    start_angle: Rad64,
    end_angle: Rad64,
) -> Result<Shape<EdgeTag, P>, EdgeCreationError> {
    Shape::build(|model| add_arc(model, plane, radius, start_angle, end_angle))
}

/// Creates a finite helical edge around an axis.
pub fn helix(
    axis: Axis3,
    radius: f64,
    pitch: f64,
    start_angle: Rad64,
    end_angle: Rad64,
) -> Result<Shape<EdgeTag, StandardPayload>, EdgeCreationError> {
    helix_with::<StandardPayload>(axis, radius, pitch, start_angle, end_angle)
}

/// As [`helix`], with the payload chosen by the caller.
pub fn helix_with<P: Payload>(
    axis: Axis3,
    radius: f64,
    pitch: f64,
    start_angle: Rad64,
    end_angle: Rad64,
) -> Result<Shape<EdgeTag, P>, EdgeCreationError> {
    Shape::build(|model| add_helix(model, axis, radius, pitch, start_angle, end_angle))
}

/// Creates a closed circular edge in 3D space.
pub fn circle(
    plane: Plane,
    radius: f64,
) -> Result<Shape<EdgeTag, StandardPayload>, EdgeCreationError> {
    circle_with::<StandardPayload>(plane, radius)
}

/// As [`circle`], with the payload chosen by the caller.
pub fn circle_with<P: Payload>(
    plane: Plane,
    radius: f64,
) -> Result<Shape<EdgeTag, P>, EdgeCreationError> {
    Shape::build(|model| add_circle(model, plane, radius))
}

impl<P: Payload> Shape<EdgeTag, P> {
    pub fn into_profile(self) -> Shape<ProfileTag, P> {
        let (mut g, edge_key) = self.into_model();
        let profile_key = add_profile_from_edges(&mut g, &[edge_key])
            .expect("valid edge can always be converted in a profile");
        Shape::new(g, profile_key)
    }
}
