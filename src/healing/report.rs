use crate::topology::shape_keys::{EdgeKey, FaceKey, VertexKey};

/// A cell a healing pass considered.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HealedCell {
    /// A candidate for 0-removal.
    Vertex(VertexKey),
    /// A candidate for 1-removal.
    Edge(EdgeKey),
}

/// Why a pass left a candidate in place.
///
/// The reasons are diagnostic, not an error taxonomy: a healed map is expected
/// to leave most cells untouched. They matter when a model still looks
/// redundant after a run and the question is which guard held it back.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SkipReason {
    /// The cell fails the Def. 58 removability condition.
    NotRemovable,
    /// The cell does not bound exactly two distinct cells one dimension up.
    NotBetweenTwoCells,
    /// Removing the edge would break one boundary loop into several, which
    /// leaves open which of them bounds the face from outside.
    LoopWouldSplit,
    /// The edge is a seam: the face bounds it on both sides, and its surface is
    /// periodic, so the seam is where the parameterization closes rather than a
    /// slit the boundary can close over.
    PeriodicSurface,
    /// The removal would fuse loops that are not both outer boundaries.
    NotOuterLoop,
    /// A cell involved in the removal has no stored geometry.
    MissingGeometry,
    /// The two curves do not lie on one support that can carry the fused edge.
    CurvesNotJoinable,
    /// The two surfaces are neither coplanar nor the same parameterization.
    SurfacesNotJoinable,
    /// The fused edge would close on itself, leaving a vertexless loop.
    WouldCloseEdge,
    /// The fused boundary's parameter curve could not be rebuilt.
    PcurveNotJoinable,
    /// A cell involved in the removal has no registered identity.
    Unregistered,
}

/// One candidate a pass declined, with the guard that held it back.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HealingSkip {
    /// The candidate that stayed in place.
    pub cell: HealedCell,
    /// Why it stayed.
    pub reason: SkipReason,
}

/// What one healing run changed.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct HealingReport {
    /// Vertices removed by the 0-removal pass.
    pub removed_vertices: Vec<VertexKey>,
    /// Edges fused by the 0-removal pass, as `(survivor, consumed)`.
    pub fused_edges: Vec<(EdgeKey, EdgeKey)>,
    /// Edges removed by the 1-removal pass.
    pub removed_edges: Vec<EdgeKey>,
    /// Faces fused by the 1-removal pass, as `(survivor, consumed)`.
    pub fused_faces: Vec<(FaceKey, FaceKey)>,
    /// Faces whose own boundary the 1-removal pass rejoined, because one face
    /// bounded the removed edge on both sides.
    pub rejoined_faces: Vec<FaceKey>,
    /// Passes run before the map stopped changing.
    pub iterations: usize,
    /// Candidates the final pass declined, with their reasons.
    pub skipped: Vec<HealingSkip>,
}

impl HealingReport {
    /// Returns the number of cells the run removed.
    pub fn changes(&self) -> usize {
        self.removed_vertices.len() + self.removed_edges.len()
    }

    /// Returns whether the run left the map unchanged.
    pub fn is_empty(&self) -> bool {
        self.changes() == 0
    }

    /// Records a declined candidate.
    pub(crate) fn skip(&mut self, cell: HealedCell, reason: SkipReason) {
        self.skipped.push(HealingSkip { cell, reason });
    }
}
