use crate::geometry::{ANGULAR_TOLERANCE, LINEAR_TOLERANCE};
use crate::topology::shape_keys::{EdgeKey, SolidKey, VertexKey};

/// The part of a map a healing run is allowed to touch.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub enum HealingScope {
    /// Every registered cell.
    #[default]
    WholeMap,
    /// Every cell of one solid.
    Solid(SolidKey),
    /// An explicit candidate list.
    ///
    /// An operation that already knows which cells it created — a Boolean
    /// knows exactly that — passes them here so healing stays proportional to
    /// the edit rather than to the model.
    Cells {
        /// Vertices offered to the 0-removal pass.
        vertices: Vec<VertexKey>,
        /// Edges offered to the 1-removal pass.
        edges: Vec<EdgeKey>,
    },
}

/// Tunables for one healing run.
#[derive(Debug, Clone, PartialEq)]
pub struct HealingOptions {
    /// Which cells the run may consider.
    pub scope: HealingScope,
    /// Fuse the two edges meeting at a shape-free vertex.
    pub remove_redundant_vertices: bool,
    /// Fuse the two faces sharing a shape-free edge.
    pub remove_redundant_edges: bool,
    /// Remove a closed redundant interface where an inner loop is completely
    /// filled by another face on the same support surface.
    pub remove_filled_inner_loops: bool,
    /// Distance below which two positions are treated as one.
    pub linear_tolerance: f64,
    /// Angle below which two directions are treated as one.
    pub angular_tolerance: f64,
    /// Upper bound on passes before the run is declared non-convergent.
    pub max_iterations: usize,
}

impl Default for HealingOptions {
    fn default() -> Self {
        Self {
            scope: HealingScope::WholeMap,
            remove_redundant_vertices: true,
            remove_redundant_edges: true,
            remove_filled_inner_loops: true,
            linear_tolerance: LINEAR_TOLERANCE,
            angular_tolerance: ANGULAR_TOLERANCE,
            max_iterations: 16,
        }
    }
}

impl HealingOptions {
    /// Returns the default options restricted to `scope`.
    pub fn for_scope(scope: HealingScope) -> Self {
        Self {
            scope,
            ..Self::default()
        }
    }

    /// Returns the same options with both tolerances scaled to `linear`.
    ///
    /// Geometry produced by numeric intersection rarely meets the kernel's
    /// default tolerance, so callers that know their own accuracy budget
    /// should widen it here rather than loosen the predicates.
    pub fn with_linear_tolerance(mut self, linear: f64) -> Self {
        self.linear_tolerance = linear;
        self
    }
}
