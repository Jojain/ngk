//! Public operands, provenance, lineage, and preparation results.

use std::collections::HashMap;

use crate::topology::shape_keys::{EdgeKey, FaceKey, ProfileKey, SheetKey, SolidKey, VertexKey};

use super::{BooleanDiagnostics, IntersectionNetwork, IntersectionSpanId};

/// A dimension-erased operand accepted by Boolean preparation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum BooleanOperand {
    Vertex(VertexKey),
    Edge(EdgeKey),
    Profile(ProfileKey),
    Face(FaceKey),
    Sheet(SheetKey),
    Solid(SolidKey),
}

/// Selects one side of a prepared Boolean pair.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BooleanSide {
    First,
    Second,
}

/// Atomic topology provenance attached to an intersection-network element.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum BooleanCell {
    Vertex(VertexKey),
    Edge(EdgeKey),
    Face(FaceKey),
}

/// Relationship between the two geometries at an isolated event.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PointContactKind {
    Transverse,
    Tangent,
}

/// Source-to-fragment lineage for one prepared operand.
#[derive(Debug, Clone, Default)]
pub struct BooleanLineage {
    pub vertices: HashMap<VertexKey, Vec<VertexKey>>,
    pub edges: HashMap<EdgeKey, Vec<EdgeKey>>,
    pub faces: HashMap<FaceKey, Vec<FaceKey>>,
}

/// Result of importing, intersecting, and splitting both operands.
#[derive(Clone)]
pub struct BooleanOperandPreparation {
    pub first: BooleanOperand,
    pub second: BooleanOperand,
    /// The working-map handle of an imported external tool.
    pub imported_tool: Option<BooleanOperand>,
    /// Retained for callers that only need to distinguish import mode.
    pub imported_second: bool,
    pub network: IntersectionNetwork,
    pub diagnostics: BooleanDiagnostics,
    /// Section edges produced by face imprints, ordered along each span on each side.
    pub span_edges: HashMap<IntersectionSpanId, [Vec<EdgeKey>; 2]>,
    pub first_lineage: BooleanLineage,
    pub second_lineage: BooleanLineage,
}

impl BooleanOperandPreparation {
    /// Returns the final fragments derived from `source` on the selected side.
    pub fn edge_fragments(&self, side: BooleanSide, source: EdgeKey) -> &[EdgeKey] {
        self.lineage(side)
            .edges
            .get(&source)
            .map(Vec::as_slice)
            .unwrap_or(&[])
    }

    /// Returns the final face fragments derived from `source` on the selected side.
    pub fn face_fragments(&self, side: BooleanSide, source: FaceKey) -> &[FaceKey] {
        self.lineage(side)
            .faces
            .get(&source)
            .map(Vec::as_slice)
            .unwrap_or(&[])
    }

    fn lineage(&self, side: BooleanSide) -> &BooleanLineage {
        match side {
            BooleanSide::First => &self.first_lineage,
            BooleanSide::Second => &self.second_lineage,
        }
    }
}

/// Regularized set operations on two admitted solids.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BooleanOperation {
    Union,
    Intersection,
    Difference,
}

/// One operand of a Boolean, named by its place in the call.
///
/// A Boolean takes one target and any number of tools, and everything it
/// reports per operand -- lineage, sections, errors -- is indexed this way, so
/// `Tool(i)` is always `tools[i]` as the caller passed it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum BooleanInput {
    Target,
    Tool(usize),
}

impl std::fmt::Display for BooleanInput {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Target => f.write_str("the target"),
            Self::Tool(index) => write!(f, "tools[{index}]"),
        }
    }
}

/// The edges one intersection span left on each of the two operands it joins.
///
/// Each pair of operands is imprinted on the other in its own pass, so a span
/// id is only unique together with the two inputs that produced it. The edges
/// are the ones left after every pass, so a later operand that crossed the
/// section has split them further. Keys are historical: sewing consumes one
/// side of each pair, and healing may fuse what is left.
#[derive(Debug, Clone)]
pub struct BooleanSection {
    pub inputs: [BooleanInput; 2],
    pub span: IntersectionSpanId,
    pub edges: [Vec<EdgeKey>; 2],
}

/// One successfully validated, committed solid boundary.
#[derive(Debug)]
pub struct SolidBoolean {
    pub operation: BooleanOperation,
    pub solid: SolidKey,
    pub lineage: SolidBooleanLineage,
    pub diagnostics: BooleanDiagnostics,
}

/// Surviving source identities and pre-sewing intersection edge provenance.
#[derive(Debug)]
pub struct SolidBooleanLineage {
    pub target: BooleanLineage,
    /// One lineage per tool, in the order the tools were passed.
    pub tools: Vec<BooleanLineage>,
    pub sections: Vec<BooleanSection>,
    pub discarded_faces: Vec<FaceKey>,
}
