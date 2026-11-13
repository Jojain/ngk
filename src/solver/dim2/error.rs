use super::{ConstraintId, ConstraintResidual, GeometryId};
use thiserror::Error;

/// A rejected edit leaves the constraint system unchanged.
#[derive(Clone, Debug, Error)]
pub enum EditError {
    #[error("{0:?} is not in this system")]
    InvalidGeometryReference(GeometryId),
    #[error("{0:?} is not in this system")]
    InvalidConstraintReference(ConstraintId),
    #[error("{0:?} is already in this system")]
    DuplicateGeometry(GeometryId),
    #[error("{0:?} is already in this system")]
    DuplicateConstraint(ConstraintId),
    #[error("invalid geometry: {0}")]
    InvalidGeometry(&'static str),
    #[error("invalid constraint: {0}")]
    InvalidConstraint(&'static str),
    #[error("geometry is referenced by constraints {constraints:?}")]
    ReferencedGeometry {
        geometry: GeometryId,
        constraints: Vec<ConstraintId>,
    },
}

/// A failed solve never publishes its candidate configuration.
#[derive(Clone, Debug, Error)]
pub enum SolveError {
    #[error("invalid solver tolerances or iteration budgets")]
    InvalidOptions,
    #[error(transparent)]
    InvalidInput(#[from] EditError),
    #[error("contradictory dimensions or fixed geometry: {constraints:?}")]
    Conflict { constraints: Vec<ConstraintId> },
    #[error("local resolution did not converge")]
    NonConvergence {
        constraints: Vec<ConstraintId>,
        residuals: Vec<ConstraintResidual>,
    },
}
