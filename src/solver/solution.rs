use super::{
    ArcKey, CircleKey, ConstraintKey, EditError, GeometryKey, PointRef, SegmentKey, TangencyMode,
};
use super::{
    keys::GeometrySlot,
    primitives::{Kind, Primitive},
};
use crate::geometry::dim2::{curves::Circle2, trimmed::TrimmedCurve2, utils::Point2};
use slotmap::SlotMap;
use std::sync::Arc;

/// Residual in geometric units and its largest tolerance-relative violation.
#[derive(Clone, Debug)]
pub struct ConstraintResidual {
    pub constraint: ConstraintKey,
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
    pub constraint: ConstraintKey,
    pub kind: RedundancyKind,
}
#[derive(Clone, Debug)]
pub struct TargetReport {
    pub index: usize,
    pub achieved: Point2,
    pub distance: f64,
    pub reached: bool,
}
/// Diagnostics of the permanent problem; temporary objectives do not consume DOFs.
#[derive(Clone, Debug, Default)]
pub struct SolveReport {
    /// Linearized freedom, which can differ from finite mobility at singularities.
    pub degrees_of_freedom: usize,
    pub redundancies: Vec<Redundancy>,
    pub residuals: Vec<ConstraintResidual>,
    pub targets: Vec<TargetReport>,
    pub tangencies: Vec<(ConstraintKey, TangencyMode)>,
    pub iterations: usize,
    pub target_iterations: usize,
    pub target_budget_exhausted: bool,
}

/// An owned immutable result; historical geometry survives edits and deletions.
#[derive(Clone, Debug)]
pub struct Solution2d(pub(crate) Arc<SolutionData>);
#[derive(Debug)]
pub(crate) struct SolutionData {
    pub owner: u64,
    pub revision: u64,
    pub sequence: u64,
    pub geometry: SlotMap<GeometrySlot, Primitive>,
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
    pub fn segment(&self, key: SegmentKey) -> Result<TrimmedCurve2, EditError> {
        Ok(self.typed(key.0, Kind::Segment)?.as_segment())
    }
    pub fn circle(&self, key: CircleKey) -> Result<Circle2, EditError> {
        Ok(self.typed(key.0, Kind::Circle)?.as_circle())
    }
    pub fn arc(&self, key: ArcKey) -> Result<TrimmedCurve2, EditError> {
        Ok(self.typed(key.0, Kind::Arc)?.as_arc())
    }
    fn get(&self, key: GeometryKey) -> Result<&Primitive, EditError> {
        if key.owner != self.0.owner {
            return Err(EditError::InvalidGeometryReference(key));
        }
        self.0
            .geometry
            .get(key.slot)
            .filter(|p| p.serial == key.serial)
            .ok_or(EditError::InvalidGeometryReference(key))
    }
    fn typed(&self, key: GeometryKey, kind: Kind) -> Result<&Primitive, EditError> {
        let p = self.get(key)?;
        if p.kind != kind {
            return Err(EditError::InvalidGeometryReference(key));
        }
        Ok(p)
    }
}
