use super::{BodyId, JointId, JointResidual};
use thiserror::Error;

/// A rejected edit leaves the assembly unchanged.
#[derive(Clone, Debug, Error)]
pub enum EditError {
    #[error("{0:?} is not in this assembly")]
    InvalidBodyReference(BodyId),
    #[error("{0:?} is not in this assembly")]
    InvalidJointReference(JointId),
    #[error("{0:?} is already in this assembly")]
    DuplicateBody(BodyId),
    #[error("{0:?} is already in this assembly")]
    DuplicateJoint(JointId),
    #[error("invalid pose: {0}")]
    InvalidPose(&'static str),
    #[error("invalid joint: {0}")]
    InvalidJoint(&'static str),
    #[error("invalid target: {0}")]
    InvalidTarget(&'static str),
    #[error("body is held by joints {joints:?}")]
    ReferencedBody { body: BodyId, joints: Vec<JointId> },
}

/// A failed solve never publishes its candidate poses.
#[derive(Clone, Debug, Error)]
pub enum SolveError {
    #[error("invalid solver tolerances or iteration budgets")]
    InvalidOptions,
    #[error(transparent)]
    InvalidInput(#[from] EditError),
    #[error("local resolution did not converge")]
    NonConvergence {
        joints: Vec<JointId>,
        residuals: Vec<JointResidual>,
    },
}
