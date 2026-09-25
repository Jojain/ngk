use crate::topology::ModelEditError;
use std::error::Error;
use std::fmt;
use std::sync::Arc;

use crate::{
    geometry::{NurbsError, Point3},
    topology::{
        Dart,
        gmap::Dim,
        planar::PlanarityError,
        shape_keys::{EdgeKey, FaceKey, ProfileKey},
    },
};
use thiserror::Error;

/// Cloneable builder error wrapper that preserves a topology error as its source.
#[derive(Debug, Clone)]
pub struct ModelEditFailure(Arc<ModelEditError>);

impl ModelEditFailure {
    /// Wraps a topology error so builder errors can remain cloneable.
    pub fn new(error: ModelEditError) -> Self {
        Self(Arc::new(error))
    }
}

impl fmt::Display for ModelEditFailure {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.fmt(formatter)
    }
}

impl Error for ModelEditFailure {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        Some(self.0.as_ref())
    }
}

impl PartialEq for ModelEditFailure {
    fn eq(&self, other: &Self) -> bool {
        self.to_string() == other.to_string()
    }
}

#[derive(Debug, Clone, Error, PartialEq)]
pub enum EdgeCreationError {
    #[error("start and end points are coincident: {start:?} and {end:?}")]
    CoincidentPoints { start: Point3, end: Point3 },

    #[error("Invalid radius: {radius}")]
    InvalidRadius { radius: f64 },

    #[error("Invalid pitch: {pitch}")]
    InvalidPitch { pitch: f64 },

    #[error("Invalid {name} angle: {angle}")]
    InvalidAngle { name: &'static str, angle: f64 },

    #[error("edge model edit failed")]
    ModelEditFailed(#[source] ModelEditFailure),
}

impl From<ModelEditError> for EdgeCreationError {
    fn from(error: ModelEditError) -> Self {
        Self::ModelEditFailed(ModelEditFailure::new(error))
    }
}

#[derive(Debug, Clone, Error, PartialEq)]
pub enum ExtrudeError {
    #[error("profile is empty")]
    EmptyProfile,
    #[error("missing vertex point for dart {dart:?}")]
    MissingVertexPoint { dart: Dart },
    #[error("missing edge curve for dart {dart:?}")]
    MissingEdgeCurve { dart: Dart },
    #[error("extrusion direction must be non-zero")]
    ZeroDirection,
    #[error("edge at dart {dart:?} has zero length")]
    ZeroLengthEdge { dart: Dart },
    #[error("sweep at dart {dart:?} is degenerate")]
    DegenerateSweep { dart: Dart },
    #[error("darts {first:?} and {second:?} are not sewable in dimension {dim:?}")]
    SewFailed { dim: Dim, first: Dart, second: Dart },
    #[error("missing face for key {dart:?}")]
    MissingFace { dart: FaceKey },
    #[error("extrusion model edit failed")]
    ModelEditFailed(#[source] ModelEditFailure),
}

impl From<ModelEditError> for ExtrudeError {
    fn from(error: ModelEditError) -> Self {
        Self::ModelEditFailed(ModelEditFailure::new(error))
    }
}

#[derive(Debug, Clone, Error, PartialEq)]
pub enum PolylineError {
    #[error("polyline is empty")]
    EmptyPolyline,
    #[error("the supplied edges do not form one connected profile")]
    DisconnectedEdges,
    #[error("more than two supplied edges meet at {point:?}")]
    NonManifoldEdgeConnection { point: Point3 },
    #[error("polygon needs at least 3 points, got {point_count}")]
    InvalidPolygon { point_count: usize },
    #[error("profile starting at dart {dart:?} is already closed")]
    ClosedProfile { dart: Dart },
    #[error("profile {profile:?} does not exist")]
    MissingProfile { profile: ProfileKey },
    #[error("edge {edge:?} does not exist")]
    MissingEdge { edge: EdgeKey },
    #[error(
        "edge starting at {edge_start:?} cannot be added after profile ending at {profile_end:?}"
    )]
    NonContiguousEdge {
        profile_end: Point3,
        edge_start: Point3,
    },
    #[error("profile starting at dart {dart:?} is open")]
    OpenProfile { dart: Dart },
    #[error("profile is not planar: {0}")]
    NonPlanarProfile(#[from] PlanarityError),
    #[error("missing vertex point for dart {dart:?}")]
    MissingVertexPoint { dart: Dart },
    #[error("missing edge curve for dart {dart:?}")]
    MissingEdgeCurve { dart: Dart },
    #[error("rectangle {axis} size must be greater than 0, got {value}")]
    InvalidRectangleSize { axis: &'static str, value: f64 },
    #[error("darts {first:?} and {second:?} are not sewable in dimension {dim:?}")]
    SewFailed { dim: Dim, first: Dart, second: Dart },
    #[error("polyline model edit failed")]
    ModelEditFailed(#[source] ModelEditFailure),
    #[error("failed to create polyline edge")]
    EdgeCreationFailed(#[from] EdgeCreationError),
    #[error("failed to create profile pcurve")]
    Nurbs(#[from] NurbsError),
}

impl From<ModelEditError> for PolylineError {
    fn from(error: ModelEditError) -> Self {
        match error {
            ModelEditError::NotSewable { dim, first, second } => {
                Self::SewFailed { dim, first, second }
            }
            error => Self::ModelEditFailed(ModelEditFailure::new(error)),
        }
    }
}

#[derive(Debug, Clone, Error, PartialEq)]
pub enum FaceCreationError {
    #[error("polygon needs at least 3 points, got {point_count}")]
    InvalidPolygon { point_count: usize },
    #[error("profile bounding face is open at dart {dart:?}")]
    OpenProfile { dart: Dart },
    #[error("profile bounding face is not planar: {0}")]
    NonPlanarProfile(#[from] PlanarityError),
    #[error("failed to create face profile")]
    ProfileCreationFailed(#[from] PolylineError),
    #[error("failed to create face boundary edge")]
    EdgeCreationFailed(#[from] EdgeCreationError),
    #[error(
        "annulus outer radius must be greater than inner radius, got outer {outer_radius} and inner {inner_radius}"
    )]
    InvalidAnnulusRadii {
        outer_radius: f64,
        inner_radius: f64,
    },
    #[error("face model edit failed")]
    ModelEditFailed(#[source] ModelEditFailure),
}

impl From<ModelEditError> for FaceCreationError {
    fn from(error: ModelEditError) -> Self {
        Self::ModelEditFailed(ModelEditFailure::new(error))
    }
}

/// A closed support this kernel has no polygon schema for.
#[derive(Debug, Error)]
pub enum ClosedFaceCellError {
    /// The support leaves a boundary somewhere, so no face over it bounds
    /// nothing.
    #[error("a face that bounds nothing needs a support closing in both directions")]
    SupportNotClosed,

    /// Both parameter directions close by collapsing to a point. No polygon
    /// here describes that surface, and guessing one would put edges where the
    /// support has none.
    #[error("a support closing by collapse in both directions has no polygon schema")]
    ClosureNotSchematized,

    /// Building the polygon's darts or involutions failed.
    #[error(transparent)]
    ModelEdit(#[from] ModelEditError),
}
