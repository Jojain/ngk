//! Public operands, provenance, lineage, and preparation results.

use std::collections::HashMap;

use crate::topology::shape_keys::{EdgeKey, FaceKey, ProfileKey, SheetKey, SolidKey, VertexKey};

use super::IntersectionNetwork;

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
pub struct BooleanPreparation {
    pub first: BooleanOperand,
    pub second: BooleanOperand,
    /// The working-map handle of an imported external tool.
    pub imported_tool: Option<BooleanOperand>,
    /// Retained for callers that only need to distinguish import mode.
    pub imported_second: bool,
    pub network: IntersectionNetwork,
    pub first_lineage: BooleanLineage,
    pub second_lineage: BooleanLineage,
}

impl BooleanPreparation {
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
