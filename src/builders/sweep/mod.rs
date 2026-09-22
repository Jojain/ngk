//! Sweeping sections along edge or profile spines.
//!
//! A path segment contributes one group of lateral walls. Tangent-continuous
//! junctions sew adjacent wall groups directly; sharp junctions are handled by an
//! explicit transition policy, independently of frame transport along each
//! smooth segment.

mod errors;
mod operations;

pub use errors::*;
pub use operations::*;
