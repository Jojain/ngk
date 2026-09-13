//! **L3 — the geometry mapping.** `PLANE` ↔ `Surface::Plane`, and the
//! parameter maps.
//!
//! Knows `geometry::` and Part 21 entity names; knows nothing of `topology::`,
//! the map or darts. That makes it exhaustively testable with no map at all,
//! which matters because this is exactly where the parameterization bugs live.
//!
//! One table drives both directions, so that read and write cannot drift into
//! a pair of mappings that disagree.

pub mod curves;
pub mod iso_curve;
pub mod nurbs;
pub mod pcurve;
pub mod placement;
pub mod surfaces;
pub mod uv_map;
