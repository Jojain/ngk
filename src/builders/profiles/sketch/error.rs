use crate::builders::errors::{EdgeCreationError, ModelEditFailure, PolylineError};
use crate::geometry::dim2::intersections::CurveIntersectionError;
use crate::geometry::dim2::utils::Point2;
use crate::solver::CurveId;
use crate::topology::edit::ModelEditError;
use thiserror::Error;

/// Every refusal names the curves at fault and leaves the model unchanged.
#[derive(Debug, Error)]
pub enum ProfilesFromSystemError {
    #[error("the system has no solution for its current revision")]
    NotSolved,
    #[error("{0:?} is not in the system")]
    UnknownCurve(CurveId),
    #[error("{0:?} is listed twice")]
    DuplicateCurve(CurveId),
    #[error("more than two curve ends are joined: {curves:?}")]
    Branching { curves: Vec<CurveId> },
    #[error("{first:?} and {second:?} are joined but {gap} apart")]
    JointGap {
        first: CurveId,
        second: CurveId,
        gap: f64,
    },
    /// The curves meet somewhere no coincidence joins them: they cross, one ends on the
    /// other, or two ends touch without a coincidence saying they are one point.
    #[error("{first:?} and {second:?} meet at {point:?} without being joined there")]
    Crossing {
        first: CurveId,
        second: CurveId,
        point: Point2,
    },
    #[error("{first:?} and {second:?} overlap")]
    Overlapping { first: CurveId, second: CurveId },
    /// The search could not establish that nothing else is there.
    #[error("whether {first:?} and {second:?} cross could not be established")]
    UncertainCrossing { first: CurveId, second: CurveId },
    #[error("intersecting {first:?} and {second:?} failed")]
    Intersection {
        first: CurveId,
        second: CurveId,
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

impl From<ModelEditError> for ProfilesFromSystemError {
    fn from(error: ModelEditError) -> Self {
        Self::ModelEditFailed(ModelEditFailure::new(error))
    }
}
