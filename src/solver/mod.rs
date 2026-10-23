//! Editable 2D geometric constraints, immutable solutions, and local movement objectives.

mod constraints;
mod error;
mod keys;
mod numerical;
mod options;
mod primitives;
mod solution;
mod system;

pub use constraints::{Constraint2d, TangencyMode, Target2d};
pub use error::{EditError, SolveError};
pub use keys::{
    ArcKey, CircleKey, CircularKey, ConstraintKey, GeometryKey, PointKey, PointRef, SegmentKey,
};
pub use options::SolveOptions;
pub use solution::{
    ConstraintResidual, Redundancy, RedundancyKind, Solution2d, SolveReport, TargetReport,
};
pub use system::{Removal, System2d};
