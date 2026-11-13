//! The assembly solver: rigid bodies positioned by joints between frames they carry.
//!
//! A body is a pose with no shape. A joint holds a frame on each of two bodies and
//! lets the second move relative to the first only as its kind allows. Solving finds
//! every body's pose, moving each as little as possible from where it starts.

mod error;
mod ids;
mod joint;
mod solution;
mod solving;
mod storage;
mod system;
mod target;

use super::SolveOptions;

pub use error::{EditError, SolveError};
pub use ids::{BodyId, JointId};
pub use joint::{Attachment, Bound, Coordinate, Joint3d, JointKind, Limit};
pub use solution::{
    ActiveLimit, BodyFreedom, JointRedundancy, JointResidual, Solution3d, SolveReport, TargetReport,
};
pub use system::{Assembly3d, Removal};
pub use target::Target3d;
