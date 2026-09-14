//! Which raw cells lie inside a logical entity, and what that lets you walk.
//!
//! The GMap below this module knows nothing about geometry or about modeling
//! entities: it is darts and involutions. This module is the interpretation
//! laid over it. A logical entity occupies exactly one raw cell of its own
//! dimension; the converse does not hold, and a raw cell carrying no entity of
//! its own is **embedded** in the entity of higher dimension whose *interior*
//! contains it. That is what makes a cylinder's seam a cell inside its wall
//! rather than an edge a user can select, and a circle's closure point a cell
//! inside the circle rather than a vertex.
//!
//! Every stored record names an embedded cell, so its owner is always of
//! strictly greater dimension than the cell. Where an entity's *own* cell is
//! comes from the entity's anchor instead, and is never written down.
//!
//! Records are stored one per orbit, anchored at a representative dart
//! ([`Embedding`]). Nothing stores the list of darts an entity covers:
//! [`recover_region`] walks it out of the map each time. An entity occupies
//! exactly one raw cell of its own dimension, so that walk is one orbit and
//! nothing floods.
//!
//! Two walks read those regions. [`boundary_cycles`] gives a logical face its
//! oriented loops, turning across interior cuts instead of emitting them, so a
//! bridged annulus comes back as the two cycles it really has.
//! [`boundary_shells`] gives a logical solid its boundary components, turning
//! across the internal faces that subdivide its volume, so a block with a
//! cavity comes back as two shells reached from one anchor.

pub mod boundary;
pub mod cells;
pub mod region;
pub mod walk;

pub use boundary::{
    BoundaryCycle, BoundaryError, BoundaryShell, boundary_cycles, boundary_shells,
    boundary_vertices,
};
pub(crate) use cells::OwnerRemap;
pub use cells::{EmbeddedCell, Embedding, EmbeddingError, EmbeddingIndex, EntityOwner};
pub use region::{LogicalRegion, RegionError, recover_region};
pub use walk::{is_embedded_cell, turn, turn_where};
