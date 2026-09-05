//! Geometric tests that decide whether removing a cell would change the shape.
//!
//! Everything here is pure geometry: it reads curves and surfaces and answers
//! questions about them, and never touches the map. The passes in
//! [`super::passes`] combine these answers with the topological removability
//! condition and the structural guards.

pub mod curve;
pub mod pcurve;
pub mod surface;

pub use curve::{join_curves, sample_between};
pub use pcurve::boundary_pcurve;
pub use surface::{SurfaceMatch, surfaces_match};
