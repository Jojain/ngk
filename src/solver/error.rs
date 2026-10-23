use super::{ConstraintKey, ConstraintResidual, GeometryKey};
use thiserror::Error;

/// A rejected edit leaves the constraint system unchanged.
#[derive(Clone, Debug, Error)]
pub enum EditError {
    #[error("geometry reference is stale or belongs to another system")]
    InvalidGeometryReference(GeometryKey),
    #[error("constraint reference is stale or belongs to another system")]
    InvalidConstraintReference(ConstraintKey),
    #[error("invalid geometry: {0}")]
    InvalidGeometry(&'static str),
    #[error("invalid constraint: {0}")]
    InvalidConstraint(&'static str),
    #[error("geometry is referenced by constraints {constraints:?}")]
    ReferencedGeometry {
        geometry: GeometryKey,
        constraints: Vec<ConstraintKey>,
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
    Conflict { constraints: Vec<ConstraintKey> },
    #[error("local resolution did not converge")]
    NonConvergence {
        constraints: Vec<ConstraintKey>,
        residuals: Vec<ConstraintResidual>,
    },
}
