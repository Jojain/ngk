use radians::Rad64;

use crate::builders::edges::{
    EdgeSplit, EdgeSplitError, add_arc, add_circle, add_helix, add_line, split_edge,
};
use crate::builders::errors::EdgeCreationError;
use crate::builders::profiles::add_profile_from_edges;
use crate::geometry::{Axis3, Fraction, Plane, Point3};
use crate::model::Model;
use crate::topology::payload::{Payload, StandardPayload};
use crate::topology::shape::{EdgeTag, ProfileTag, Shape};
use crate::topology::shape_keys::{EdgeKey, FaceKey, VertexKey};

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

/// A cut edge's one model and its resulting edge and corner handles.
pub struct EdgeSplitResult<P: Payload = StandardPayload> {
    pub(crate) model: Model<P>,
    pub(crate) split: EdgeSplit,
    pub(crate) face: Option<FaceKey>,
}

impl<P: Payload> EdgeSplitResult<P> {
    pub fn model(&self) -> &Model<P> {
        &self.model
    }
    pub fn edge_keys(&self) -> Vec<EdgeKey> {
        self.split.edges().collect()
    }
    pub fn vertex_key(&self) -> VertexKey {
        self.split.vertex()
    }
    pub fn face_key(&self) -> Option<FaceKey> {
        self.face
    }
    pub fn split(&self) -> EdgeSplit {
        self.split
    }
    pub fn into_model(self) -> (Model<P>, EdgeSplit, Option<FaceKey>) {
        (self.model, self.split, self.face)
    }
}

/// Cuts an owned profile-only edge at a fraction of its span.
pub fn split<P: Payload>(
    edge: Shape<EdgeTag, P>,
    fraction: Fraction,
) -> Result<EdgeSplitResult<P>, EdgeSplitError> {
    let (mut model, key) = edge.into_model();
    let split = split_edge(&mut model, key, fraction)?;
    Ok(EdgeSplitResult {
        model,
        split,
        face: None,
    })
}
