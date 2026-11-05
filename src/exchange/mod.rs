//! File interchange: reading and writing foreign formats.
//!
//! STEP carries the B-rep; STL carries a tessellated surface.
//!
//! This module sits **above `healing`** in the crate's layering — import calls
//! healing to canonicalize a file whose periodic faces arrive cut open along a
//! seam. Nothing in the kernel may depend on `exchange`.

pub mod step;
pub mod stl;
