//! BRep-typed [`VizScene`] IR.
//!
//! The renderer doesn't see flat point / segment / mesh primitives. It sees
//! vertices, edges, faces, darts and α-involution links — the same vocabulary
//! the kernel uses. Each entity carries an id so the front-end can correlate
//! geometry with the underlying topology (vertex/edge/face slot ids in the
//! GMap; dart numbers for darts).

use serde::Serialize;

/// All of the geometry to be rendered for one scripted scene.
#[derive(Debug, Clone, Default, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct VizScene {
    pub vertices: Vec<VizVertex>,
    pub edges: Vec<VizEdge>,
    pub faces: Vec<VizFace>,
    pub darts: Vec<VizDart>,
    pub alpha_links: Vec<VizAlphaLink>,
    pub labels: Vec<VizLabel>,
}

impl VizScene {
    pub fn new() -> Self {
        Self::default()
    }
}

/// A 0-cell. `vertex_id` is a per-scene index assigned by the orchestrator.
#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct VizVertex {
    pub vertex_id: u32,
    pub position: [f64; 3],
    #[serde(skip_serializing_if = "Option::is_none")]
    pub color: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub size: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub label: Option<String>,
}

/// A 1-cell tessellated as a polyline. Two points = straight edge.
#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct VizEdge {
    pub edge_id: u32,
    pub polyline: Vec<[f64; 3]>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub color: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub width: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub label: Option<String>,
}

/// A 2-cell tessellated as an indexed triangle mesh.
#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct VizFace {
    pub face_id: u32,
    pub positions: Vec<[f64; 3]>,
    pub normals: Vec<[f64; 3]>,
    pub indices: Vec<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub color: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub opacity: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub double_sided: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub label: Option<String>,
}

/// A single dart drawn as a half-edge arrow. The shaft is a polyline that
/// follows the underlying edge's curve from the dart's vertex to the edge
/// midpoint; `tip_dir` is the unit tangent at the last shaft sample so the
/// renderer can orient the arrow's cone.
#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct VizDart {
    pub dart_id: u32,
    pub edge_id: u32,
    pub shaft: Vec<[f64; 3]>,
    pub tip_dir: [f64; 3],
    #[serde(skip_serializing_if = "Option::is_none")]
    pub color: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub label: Option<String>,
}

/// A link between two darts representing one αᵢ pairing. Endpoint geometry
/// (the shaft midpoint of each dart) is precomputed so the renderer can draw
/// without joining tables.
#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct VizAlphaLink {
    pub involution: u32,
    pub dart_a: u32,
    pub dart_b: u32,
    pub a: [f64; 3],
    pub b: [f64; 3],
}

/// Free-form annotation. Used for ad-hoc graphic debugging.
#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct VizLabel {
    pub position: [f64; 3],
    pub text: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub color: Option<String>,
}
