//! Editable 2D geometric constraints, immutable solutions, and local movement objectives.

mod constraints;
mod error;
mod ids;
mod numerical;
mod options;
mod primitives;
mod solution;
mod storage;
mod system;

pub use constraints::{Constraint2d, TangencyMode, Target2d};
pub use error::{EditError, SolveError};
pub(crate) use ids::AnyId;
pub use ids::{
    ArcId, CircleId, CircularId, ConstraintId, CurveId, GeometryId, PointId, PointRef, SegmentId,
};
pub use options::SolveOptions;
pub use solution::{
    ConstraintResidual, PrimitiveFreedom, Redundancy, RedundancyKind, Solution2d, SolveReport,
    TargetReport,
};
pub use system::{Removal, System2d};
