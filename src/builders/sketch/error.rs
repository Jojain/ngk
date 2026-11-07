use crate::builders::errors::{
    EdgeCreationError, FaceCreationError, ModelEditFailure, PolylineError,
};
use crate::geometry::dim2::intersections::CurveIntersectionError;
use crate::geometry::dim2::utils::Point2;
use crate::solver::CurveId;
use crate::topology::edit::ModelEditError;
use thiserror::Error;

/// Every refusal names what is at fault and leaves the model unchanged.
#[derive(Debug, Error)]
pub enum SketchError {
    #[error("the system has no solution for its current revision")]
    NotSolved,
    #[error("{0:?} is not in the system")]
    UnknownCurve(CurveId),
    #[error("{0:?} is listed twice")]
    DuplicateCurve(CurveId),
    /// A profile is a chain or a cycle, so it cannot hold this. Regions can.
    #[error("more than two curve ends are joined: {curves:?}")]
    Branching { curves: Vec<CurveId> },
    #[error("{first:?} and {second:?} are joined but {gap} apart")]
    JointGap {
        first: CurveId,
        second: CurveId,
        gap: f64,
    },
    /// The curves meet somewhere no coincidence joins them: they cross, one ends on the
    /// other, or two ends touch without a coincidence saying they are one point. Only a
    /// profile refuses this: regions cut the curves there.
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
    /// A round curve cannot end at a point this far from its circle: the edge would not
    /// lie on its support.
    #[error("{curve:?} is cut at {point:?}, {gap} away from it")]
    CutOffCurve {
        curve: CurveId,
        point: Point2,
        gap: f64,
    },
    /// A region's boundary runs through this point twice, as where a hole touches the
    /// outer boundary, and a boundary loop visits each corner once.
    #[error("a region's boundary runs through {point:?} twice")]
    PinchedRegion { point: Point2 },
    #[error(transparent)]
    Edge(#[from] EdgeCreationError),
    #[error(transparent)]
    Profile(#[from] PolylineError),
    #[error(transparent)]
    Face(#[from] FaceCreationError),
    #[error("building what the sketch draws failed")]
    ModelEditFailed(#[source] ModelEditFailure),
}

impl From<ModelEditError> for SketchError {
    fn from(error: ModelEditError) -> Self {
        Self::ModelEditFailed(ModelEditFailure::new(error))
    }
}
