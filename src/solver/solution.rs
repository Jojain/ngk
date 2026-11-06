use super::primitives::Primitive;
use super::{
    ArcId, CircleId, ConstraintId, EditError, GeometryId, PointRef, SegmentId, TangencyMode,
};
use crate::geometry::dim2::{curves::Circle2, trimmed::TrimmedCurve2, utils::Point2};
use std::collections::BTreeMap;
use std::sync::Arc;

/// Residual in geometric units and its largest tolerance-relative violation.
#[derive(Clone, Debug)]
pub struct ConstraintResidual {
    pub constraint: ConstraintId,
    pub values: Vec<f64>,
    pub max_tolerance_ratio: f64,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RedundancyKind {
    /// An identical permanent relation, including captured fixed values.
    Duplicate,
    /// Jacobian dependency at this configuration; not proof of a removable relation.
    LocalDependency,
    PartialLocalDependency,
}
#[derive(Clone, Debug)]
pub struct Redundancy {
    pub constraint: ConstraintId,
    pub kind: RedundancyKind,
}
#[derive(Clone, Debug)]
pub struct TargetReport {
    pub index: usize,
    pub achieved: Point2,
    pub distance: f64,
    pub reached: bool,
}
/// How freely one primitive's own parameters can still move.
#[derive(Clone, Debug)]
pub struct PrimitiveFreedom {
    pub geometry: GeometryId,
    pub degrees: usize,
}
/// Diagnostics of the permanent problem; temporary objectives do not consume DOFs.
#[derive(Clone, Debug, Default)]
pub struct SolveReport {
    /// Linearized freedom, which can differ from finite mobility at singularities.
    pub degrees_of_freedom: usize,
    /// Linearized freedom per primitive. A motion shared by several primitives counts in
    /// each, so these do not add up to `degrees_of_freedom`.
    pub freedoms: Vec<PrimitiveFreedom>,
    pub redundancies: Vec<Redundancy>,
    pub residuals: Vec<ConstraintResidual>,
    pub targets: Vec<TargetReport>,
    pub tangencies: Vec<(ConstraintId, TangencyMode)>,
    pub iterations: usize,
    pub target_iterations: usize,
    pub target_budget_exhausted: bool,
}

/// An owned immutable result; historical geometry survives edits and deletions.
#[derive(Clone, Debug)]
pub struct Solution2d(pub(crate) Arc<SolutionData>);
#[derive(Debug)]
pub(crate) struct SolutionData {
    pub revision: u64,
    pub sequence: u64,
    pub geometry: BTreeMap<GeometryId, Primitive>,
    pub report: SolveReport,
}
impl Solution2d {
    pub fn revision(&self) -> u64 {
        self.0.revision
    }
    pub fn sequence(&self) -> u64 {
        self.0.sequence
    }
    pub fn report(&self) -> &SolveReport {
        &self.0.report
    }
    pub fn point(&self, point: impl Into<PointRef>) -> Result<Point2, EditError> {
        let p = point.into();
        Ok(self.get(p.geometry)?.point_at(p.role))
    }
    pub fn segment(&self, id: SegmentId) -> Result<TrimmedCurve2, EditError> {
        Ok(self.get(id.0)?.as_segment())
    }
    pub fn circle(&self, id: CircleId) -> Result<Circle2, EditError> {
        Ok(self.get(id.0)?.as_circle())
    }
    pub fn arc(&self, id: ArcId) -> Result<TrimmedCurve2, EditError> {
        Ok(self.get(id.0)?.as_arc())
    }
    fn get(&self, id: GeometryId) -> Result<&Primitive, EditError> {
        self.0
            .geometry
            .get(&id)
            .ok_or(EditError::InvalidGeometryReference(id))
    }
}
