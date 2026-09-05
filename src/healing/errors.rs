use thiserror::Error;

use crate::builders::removal::CellRemovalError;
use crate::topology::TopologyEditError;
use crate::topology::shape_keys::{FaceKey, SolidKey};

/// Failure raised while healing a map.
#[derive(Debug, Error)]
pub enum HealingError {
    /// Passes kept changing the map after the iteration budget ran out.
    ///
    /// Every accepted removal strictly reduces the cell count, so this signals
    /// a predicate that keeps re-proposing work rather than a large model.
    #[error("healing did not reach a fixed point within {iterations} iterations")]
    NoConvergence { iterations: usize },
    /// The requested scope names a solid the map does not hold.
    #[error("healing scope references solid {solid:?}, which is not registered")]
    MissingSolid { solid: SolidKey },
    /// A fused face's parameter curves could not be projected back onto its
    /// support surface after the removal succeeded.
    #[error("healing could not rebuild the parameter curves of face {face:?}")]
    PcurveRebuildFailed { face: FaceKey },
    /// A removal accepted by the policy was rejected by the mechanism.
    #[error(transparent)]
    Removal(#[from] CellRemovalError),
    /// A staged edit was rejected.
    #[error(transparent)]
    Topology(#[from] TopologyEditError),
}
