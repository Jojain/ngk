use super::{BodyId, EditError, JointId};
use crate::geometry::dim3::utils::Point3;
use crate::geometry::transform::Rigid;
use std::collections::BTreeMap;
use std::sync::Arc;

/// A joint's residuals in the caller's units and radians, and their largest
/// tolerance-relative violation.
#[derive(Clone, Debug)]
pub struct JointResidual {
    pub joint: JointId,
    pub values: Vec<f64>,
    pub max_tolerance_ratio: f64,
}

/// A joint some of whose equations depend, at this configuration, on the joints
/// before it: the assembly is hyperstatic there.
#[derive(Clone, Debug)]
pub struct JointRedundancy {
    pub joint: JointId,
    /// How many of the joint's equations are locally dependent.
    pub equations: usize,
}

/// How freely one body can still move.
#[derive(Clone, Debug)]
pub struct BodyFreedom {
    pub body: BodyId,
    pub degrees: usize,
}

#[derive(Clone, Debug)]
pub struct TargetReport {
    pub index: usize,
    pub achieved: Point3,
    pub distance: f64,
    pub reached: bool,
}

/// Diagnostics of the permanent problem; temporary targets do not consume freedoms.
///
/// Counts are linearized at the solution: at a singular configuration they can
/// differ from the finite mobility.
#[derive(Clone, Debug, Default)]
pub struct SolveReport {
    /// The freedoms the assembly keeps, over all its bodies.
    pub mobility: usize,
    /// The degree of redundancy (hyperstatism): joint equations depending on others.
    pub redundancy: usize,
    /// Per body. A motion shared by several bodies counts in each, so these do not
    /// add up to `mobility`.
    pub freedoms: Vec<BodyFreedom>,
    pub redundancies: Vec<JointRedundancy>,
    pub residuals: Vec<JointResidual>,
    pub targets: Vec<TargetReport>,
    pub iterations: usize,
    pub target_iterations: usize,
    pub target_budget_exhausted: bool,
}

/// An owned immutable result; its poses survive later edits and removals.
#[derive(Clone, Debug)]
pub struct Solution3d(pub(super) Arc<SolutionData>);

#[derive(Debug)]
pub(super) struct SolutionData {
    pub revision: u64,
    pub sequence: u64,
    pub poses: BTreeMap<BodyId, Rigid>,
    pub report: SolveReport,
}

impl Solution3d {
    pub fn revision(&self) -> u64 {
        self.0.revision
    }
    pub fn sequence(&self) -> u64 {
        self.0.sequence
    }
    pub fn report(&self) -> &SolveReport {
        &self.0.report
    }
    /// The solved pose of a body: where its own coordinates land in the world.
    pub fn pose(&self, body: BodyId) -> Result<Rigid, EditError> {
        self.0
            .poses
            .get(&body)
            .copied()
            .ok_or(EditError::InvalidBodyReference(body))
    }
}
