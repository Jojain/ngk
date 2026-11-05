use crate::tessellate::{TessellateOpts, VIEWER};

/// Sampling and validation applied before writing binary STL.
#[derive(Debug, Clone, Copy)]
pub struct StlWriteOptions {
    /// Angular sampling of edges and surface refinement, shared with tessellation.
    pub tessellation: TessellateOpts,
    /// Requires every serialized edge to occur twice in opposite directions.
    /// Disable explicitly when exporting an open face or sheet.
    pub require_closed: bool,
}

impl Default for StlWriteOptions {
    fn default() -> Self {
        Self {
            tessellation: VIEWER,
            require_closed: true,
        }
    }
}
