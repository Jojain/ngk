//! The 2D sketch solver: primitives, relations, and the system that solves them.

mod constraints;
mod error;
mod ids;
mod primitives;
mod solution;
mod solving;
mod storage;
mod system;

use super::SolveOptions;

pub use constraints::{Constraint2d, TangencyMode, Target2d};
pub use error::{EditError, SolveError};
pub(crate) use ids::AnyId;
pub use ids::{
    ArcId, CircleId, CircularId, ConstraintId, CurveId, GeometryId, PointId, PointRef, SegmentId,
};
pub use solution::{
    ConstraintResidual, PrimitiveFreedom, Redundancy, RedundancyKind, Solution2d, SolveReport,
    TargetReport,
};
pub use system::{Removal, System2d};
