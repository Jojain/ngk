//! Errors produced while computing and applying a Boolean preparation.

use thiserror::Error;

use super::{BooleanDiagnostics, IntersectionSpanId};
use crate::builders::edges::EdgeSplitError;
use crate::builders::faces::{FaceEdgeSplitError, FaceImprintSplitError};
use crate::geometry::Point3;
use crate::geometry::{CurveIntersectionError, IntersectionError, NurbsError};
use crate::topology::TopologyEditError;
use crate::topology::shape_keys::{EdgeKey, FaceKey, SolidKey};
use crate::topology::validation::GMapValidationError;

use super::{BooleanCell, BooleanOperand, IntersectionNetworkValidationError};

/// Failure returned by Boolean preparation.
#[derive(Debug, Error)]
pub enum BooleanError {
    #[error("imprint section cannot be attributed to canonical span {span:?}")]
    UnrealizedSpan { span: IntersectionSpanId },
    #[error("solid {solid:?} is not a closed, consistently oriented operand")]
    InvalidOperand {
        solid: SolidKey,
        source: GMapValidationError,
    },
    #[error("distinct operands share registered boundary faces")]
    SharedOperandBoundary,
    #[error("regularized Boolean result is empty")]
    EmptyResult,
    #[error("regularized Boolean result has {components} disconnected solids")]
    DisconnectedResult { components: usize },
    #[error("surface intersection coverage is incomplete")]
    IncompleteIntersections {
        diagnostics: Box<BooleanDiagnostics>,
    },
    #[error("no certified ray/trim classifier is available for face {face:?}")]
    UncertifiedClassificationSurface { face: FaceKey },
    #[error("face {face:?} has no interior probe with sufficient clearance")]
    MissingFragmentProbe { face: FaceKey },
    #[error(
        "classification of face {face:?} at {point:?} is ambiguous after {directions} directions"
    )]
    AmbiguousClassification {
        face: FaceKey,
        point: Point3,
        directions: usize,
    },
    #[error("span {span:?} has {first} first-side and {second} second-side boundary edges")]
    NonIsomorphicSpanSubdivision {
        span: IntersectionSpanId,
        first: usize,
        second: usize,
    },
    #[error("sewing endpoints disagree for span {span:?}")]
    SpanEndpointMismatch { span: IntersectionSpanId },
    #[error("result shell containing face {face:?} is open")]
    OpenResultShell { face: FaceKey },
    #[error("result shell containing face {face:?} has zero signed volume")]
    DegenerateResultShell { face: FaceKey },
    #[error("Boolean result failed topology validation")]
    InvalidResult(#[from] GMapValidationError),
    #[error("Boolean tolerance policy contains an invalid or non-finite budget")]
    InvalidTolerances,
    #[error("face {face:?} has no trim curve for edge {edge:?}")]
    MissingTrimCurve { face: FaceKey, edge: EdgeKey },
    #[error("Boolean trim intersection failed")]
    TrimIntersection(#[from] CurveIntersectionError),
    #[error("operand {operand:?} is not registered in the map")]
    MissingOperand { operand: BooleanOperand },
    #[error("Boolean cell {cell:?} has no geometric payload")]
    MissingGeometry { cell: BooleanCell },
    #[error("Boolean split plan is stale because operand {operand:?} no longer exists")]
    StalePlan { operand: BooleanOperand },
    #[error("Boolean intersection computation failed")]
    Intersection(#[from] IntersectionError),
    #[error("Boolean contact-curve construction failed")]
    Nurbs(#[from] NurbsError),
    #[error("Boolean edge splitting failed")]
    EdgeSplit(#[from] EdgeSplitError),
    #[error("Boolean face-boundary splitting failed")]
    FaceEdgeSplit(#[from] FaceEdgeSplitError),
    #[error("Boolean face imprint failed")]
    FaceSplit(#[from] FaceImprintSplitError),
    #[error("Boolean topology transaction failed")]
    Topology(#[from] TopologyEditError),
    #[error("Boolean intersection network is inconsistent")]
    InvalidNetwork(#[from] IntersectionNetworkValidationError),
}
