use crate::{
    geometry::{NurbsError, Point3},
    topology::{Dart, gmap::Dim, shape_keys::FaceKey},
};
use thiserror::Error;

#[derive(Debug, Clone, Error, PartialEq)]
pub enum EdgeCreationError {
    #[error("start and end points are coincident: {start:?} and {end:?}")]
    CoincidentPoints { start: Point3, end: Point3 },

    #[error("Invalid radius: {radius}")]
    InvalidRadius { radius: f64 },
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
