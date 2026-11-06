use super::SketchCurve;
use crate::builders::errors::{EdgeCreationError, ModelEditFailure, PolylineError};
use crate::geometry::dim2::intersections::CurveIntersectionError;
use crate::geometry::dim2::utils::Point2;
use crate::topology::edit::ModelEditError;
use thiserror::Error;

/// Every refusal names the curves at fault and leaves the model unchanged.
#[derive(Debug, Error)]
pub enum SketchBuildError {
    #[error("the system has no solution for its current revision")]
    NotSolved,
    #[error("{0:?} is not in the system")]
    UnknownCurve(SketchCurve),
    #[error("{0:?} is listed twice")]
    DuplicateCurve(SketchCurve),
    #[error("more than two curve ends are joined: {curves:?}")]
    Branching { curves: Vec<SketchCurve> },
    #[error("{first:?} and {second:?} are joined but {gap} apart")]
    JointGap {
        first: SketchCurve,
        second: SketchCurve,
        gap: f64,
    },
    /// The curves meet somewhere no coincidence joins them: they cross, one ends on the
    /// other, or two ends touch without a coincidence saying they are one point.
    #[error("{first:?} and {second:?} meet at {point:?} without being joined there")]
    Crossing {
        first: SketchCurve,
        second: SketchCurve,
        point: Point2,
    },
    #[error("{first:?} and {second:?} overlap")]
    Overlapping {
        first: SketchCurve,
        second: SketchCurve,
    },
    /// The search could not establish that nothing else is there.
    #[error("whether {first:?} and {second:?} cross could not be established")]
    UncertainCrossing {
        first: SketchCurve,
        second: SketchCurve,
    },
    #[error("intersecting {first:?} and {second:?} failed")]
    Intersection {
        first: SketchCurve,
        second: SketchCurve,
        #[source]
        source: CurveIntersectionError,
    },
    #[error(transparent)]
    Edge(#[from] EdgeCreationError),
    #[error(transparent)]
    Profile(#[from] PolylineError),
    #[error("building the sketch's profiles failed")]
    ModelEditFailed(#[source] ModelEditFailure),
}

impl From<ModelEditError> for SketchBuildError {
    fn from(error: ModelEditError) -> Self {
        Self::ModelEditFailed(ModelEditFailure::new(error))
    }
}
