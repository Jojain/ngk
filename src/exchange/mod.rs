//! File interchange: reading and writing foreign formats.
//!
//! Named `exchange` rather than `io` or `step` because IGES, STL, glTF and a
//! native format belong beside STEP later.
//!
//! This module sits **above `healing`** in the crate's layering — import calls
//! healing to canonicalize a file whose periodic faces arrive cut open along a
//! seam. Nothing in the kernel may depend on `exchange`.

pub mod step;
