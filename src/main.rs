mod geometry;
mod topology;

// `use geometry::*` only pulls items declared in `geometry/mod.rs` (submodule names), not
// nested `pub` items. Use `surfaces::*`, `gmap::*`, etc., or add `pub use` re-exports in mod.rs.
use geometry::{surfaces::*, utils::*};
use nalgebra::Vector3;
use topology::gmap::*;

fn main() {
    let plane = Plane::from_xy(
        Point3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
    );
    let gmap = GMap::new(3);
    gmap.
    println!("Hello, world! {:?}", plane.origin);
}
