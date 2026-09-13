//! Which logical entity owns each raw cell, and what that lets you walk.
//!
//! The GMap below this module knows nothing about geometry or about modeling
//! entities: it is darts and involutions. This module is the interpretation
//! laid over it. Every raw cell is labelled with the logical entity whose
//! *interior* contains it, which is what makes a cylinder's seam a cell inside
//! its wall rather than an edge a user can select, and a circle's closure point
//! a cell inside the circle rather than a vertex.
//!
//! Labels are stored one per orbit, anchored at a representative dart
//! ([`Subdivision`]). Nothing stores the list of darts an entity covers:
//! [`recover_region`] walks it out of the map each time, so a refinement the
//! entity never heard about leaves its region, its identity and its boundary
//! unchanged.
//!
//! Two walks read those regions. [`boundary_cycles`] gives a logical face its
//! oriented loops, turning across interior cuts instead of emitting them, so a
//! bridged annulus comes back as the two cycles it really has.
//! [`boundary_shells`] gives a logical solid its boundary components, turning
//! across the internal faces that subdivide its volume, so a block with a
//! cavity comes back as two shells reached from one anchor.

pub mod boundary;
pub mod ownership;
pub mod region;
pub mod walk;

pub use boundary::{
    BoundaryCycle, BoundaryError, BoundaryShell, LogicalEdgeUse, boundary_cycles, boundary_shells,
    boundary_vertices,
};
pub(crate) use ownership::OwnerRemap;
pub use ownership::{EntityOwner, OrbitOwnership, OwnershipIndex, Subdivision, SubdivisionError};
pub use region::{LogicalRegion, RegionError, recover_all_regions, recover_region};
pub use walk::turn;
