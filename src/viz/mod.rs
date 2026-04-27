//! Neutral visualization IR and scripting result types.
//!
//! Kernel code produces [`VizScene`] values; the viz front-end only knows how
//! to render the primitives below. A script may additionally return a
//! [`GMapSnapshot`] so the JS side can inspect the underlying combinatorial
//! map interactively (see `visualization/src/kernel/viz.ts`).

use serde::Serialize;

pub mod gmap;

pub use gmap::scene_from_gmap;

#[derive(Debug, Clone, Serialize, Default)]
#[serde(rename_all = "camelCase")]
pub struct VizScene {
    pub points: Vec<VizPoint>,
    pub segments: Vec<VizSegment>,
    pub arrows: Vec<VizArrow>,
    pub alpha_links: Vec<VizLink>,
    pub labels: Vec<VizLabel>,
}

impl VizScene {
    pub fn new() -> Self {
        Self::default()
    }
}

#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct VizPoint {
    pub position: [f64; 3],
    #[serde(skip_serializing_if = "Option::is_none")]
    pub color: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub size: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub label: Option<String>,
}

#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct VizSegment {
    pub start: [f64; 3],
    pub end: [f64; 3],
    #[serde(skip_serializing_if = "Option::is_none")]
    pub color: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub width: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub label: Option<String>,
}

#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct VizArrow {
    pub origin: [f64; 3],
    pub tip: [f64; 3],
    /// Dart id this arrow represents, if any. Lets the viewer correlate
    /// arrows with GMapSnapshot entries.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dart: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub color: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub label: Option<String>,
}

/// A visual link between two points representing an α-involution pairing.
/// `involution` is the index `i` in αᵢ.
#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct VizLink {
    pub involution: u32,
    pub a: [f64; 3],
    pub b: [f64; 3],
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dart_a: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dart_b: Option<u32>,
}

#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct VizLabel {
    pub position: [f64; 3],
    pub text: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub color: Option<String>,
}

/// A serializable dump of a `GMap`'s state, consumable from JS.
///
/// - `alphas[i][d]` = id of αᵢ(d). Free darts map to themselves.
/// - `vertexPoints` is a list of `{ dart, position }` entries — not every
///   dart has a point (it's an attribute), and two darts of the same 0-cell
///   may each carry a copy.
#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct GMapSnapshot {
    pub dimension: u32,
    pub dart_count: u32,
    pub alphas: Vec<Vec<u32>>,
    pub vertex_points: Vec<VertexPointEntry>,
}

#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct VertexPointEntry {
    pub dart: u32,
    pub position: [f64; 3],
}

/// What a script returns. `scene` is always present; `gmap` is present when
/// the script actually built a combinatorial map.
#[derive(Debug, Clone, Serialize, Default)]
#[serde(rename_all = "camelCase")]
pub struct ScriptResult {
    pub scene: VizScene,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub gmap: Option<GMapSnapshot>,
}

impl ScriptResult {
    pub fn from_gmap<P: crate::topology::payload::Payload>(
        g: &crate::topology::gmap::GMap<P>,
    ) -> Self {
        Self {
            scene: scene_from_gmap(g),
            gmap: None,
        }
    }
}
