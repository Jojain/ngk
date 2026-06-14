use crate::{
    geometry::{NurbsError, Point3},
    topology::{Dart, gmap::Dim, planar::PlanarityError, shape_keys::FaceKey},
};
use thiserror::Error;

#[derive(Debug, Clone, Error, PartialEq)]
pub enum EdgeCreationError {
    #[error("start and end points are coincident: {start:?} and {end:?}")]
    CoincidentPoints { start: Point3, end: Point3 },

    #[error("Invalid radius: {radius}")]
    InvalidRadius { radius: f64 },

    #[error("Invalid {name} angle: {angle}")]
    InvalidAngle { name: &'static str, angle: f64 },
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
    #[error("failed to translate curve at dart {dart:?}: {source}")]
    CurveTranslationFailed { dart: Dart, source: NurbsError },
    #[error("failed to translate surface at dart {dart:?}: {source}")]
    SurfaceTranslationFailed { dart: Dart, source: NurbsError },
    #[error("missing face for key {dart:?}")]
    MissingFace { dart: FaceKey },
}

#[derive(Debug, Clone, Error, PartialEq)]
pub enum PolylineError {
    #[error("polyline is empty")]
    EmptyPolyline,
    #[error("polygon needs at least 3 points, got {point_count}")]
    InvalidPolygon { point_count: usize },
    #[error("profile starting at dart {dart:?} is already closed")]
    ClosedProfile { dart: Dart },
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
    #[error("failed to create polyline edge")]
    EdgeCreationFailed(#[from] EdgeCreationError),
    #[error("failed to create profile pcurve")]
    Nurbs(#[from] NurbsError),
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
}
