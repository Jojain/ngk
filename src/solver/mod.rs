//! Constraint solvers: editable 2D geometric constraints and rigid-body assemblies,
//! with immutable solutions and local movement objectives.

pub mod assembly;
mod dim2;
mod numerical;
mod options;

pub(crate) use dim2::AnyId;
pub use dim2::{
    ArcId, CircleId, CircularId, Constraint2d, ConstraintId, ConstraintResidual, CurveId,
    EditError, GeometryId, PointId, PointRef, PrimitiveFreedom, Redundancy, RedundancyKind,
    Removal, SegmentId, Solution2d, SolveError, SolveReport, System2d, TangencyMode, Target2d,
    TargetReport,
};
pub use options::SolveOptions;
