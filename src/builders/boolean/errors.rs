//! Errors produced while computing and applying a Boolean preparation.

use thiserror::Error;

use crate::builders::edges::EdgeSplitError;
use crate::builders::faces::{FaceEdgeSplitError, FaceImprintSplitError};
use crate::geometry::{IntersectionError, NurbsError};
use crate::topology::TopologyEditError;

use super::{BooleanCell, BooleanOperand, IntersectionNetworkValidationError};

/// Failure returned by Boolean preparation.
#[derive(Debug, Error)]
pub enum BooleanError {
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
