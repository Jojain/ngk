//! Sweeping sections along edge or profile spines.
//!
//! A path segment contributes one tape of lateral faces. Tangent-continuous
//! junctions sew adjacent tapes directly; sharp junctions are handled by an
//! explicit transition policy, independently of frame transport along each
//! smooth segment.

mod errors;
mod operations;

pub use errors::*;
pub use operations::*;
