//! Projecting points onto geometry.
//!
//! Projection is a pure query: it reads geometry and writes nothing. A single
//! entry point, [`project`], is dispatched by the [`Project`] trait on the
//! (source, target) pair. The trait names the mode the pair understands
//! ([`Project::Mode`]), what the target reports as touched ([`Project::Hit`])
//! and the result ([`Project::Output`]).
//!
//! - A two-dimensional target (plane, surface) takes a [`Projection`]: the
//!   foot of the perpendicular, or the first hit of a half-line along a fixed
//!   direction.
//! - A curve or edge target has only the perpendicular, so its mode is `()` and
//!   a directional projection onto it cannot be written.
//! - A face, or a [`FaceList`], is a surface that exists only where it is
//!   trimmed: a projection landing outside the trim misses.
//!
//! A result carries what the algorithm computed on the way and nothing more;
//! what a caller can recompute from it is left out. Where the answer is not
//! unique or not defined, projection refuses with a named
//! [`ProjectionError`] rather than picking.

mod ambiguity;
mod error;
mod mode;
mod options;
mod point_curve;
mod point_edge;
mod point_face;
mod point_face_list;
mod point_plane;
mod point_surface;
mod project;
mod result;

pub use error::ProjectionError;
pub use mode::Projection;
pub use options::ProjectOptions;
pub use point_face_list::FaceList;
pub use project::{Project, project, project_point};
pub use result::{PointProjection, TargetParameters};
