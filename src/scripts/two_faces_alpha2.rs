//! Two coplanar squares sharing one edge, α2-sewn along that edge.
//!
//! Layout (XY plane):
//!
//! ```text
//!   (0,1) ───────── (1,1) ───────── (2,1)
//!     │                │                │
//!     │      A         │       B        │
//!     │                │                │
//!   (0,0) ───────── (1,0) ───────── (2,0)
//! ```
//!
//! - Square A: corners (0,0), (1,0), (1,1), (0,1). Edge 1 is x=1.
//! - Square B: corners (1,0), (2,0), (2,1), (1,1). Edge 3 is x=1.
//! - α2-sew Square A's edge 1 with Square B's edge 3.

use crate::builders::add_polygon;
use crate::geometry::utils::Point3;
use crate::topology::StandardPayload;
use crate::topology::gmap::GMap;
use crate::viz::ScriptResult;

pub fn run() -> Result<ScriptResult, String> {
    let mut g = GMap::<StandardPayload>::new(3);

    let da = add_polygon(
        &mut g,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ],
    );
    let db = add_polygon(
        &mut g,
        &[
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(2.0, 1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ],
    );

    g.sew(2, da, db)
        .map_err(|e| format!("a2 sew failed: {e}"))?;

    Ok(ScriptResult::from_gmap(&g))
}
