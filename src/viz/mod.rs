//! Visualization pipeline.
//!
//! ```text
//!   Model + VizHints
//!         │
//!         ▼
//!   viz::brep   (vertices, edges, faces — via crate::tessellate::*)
//!         │
//!         ▼
//!   viz::gmap   (dart arrows that follow edge curves, α-involution links)
//!         │
//!         ▼
//!   VizScene  ──►  wasm bridge  ──►  visualization/src/components/VizSceneView.tsx
//! ```
//!
//! Scripts only build a [`Model`] and an optional [`VizHints`] bag; the
//! orchestrator [`scene_from_model`] does the rest. A [`ScriptResult`] also
//! ships an opaque [`GMapSnapshot`] so the front-end can drive an interactive
//! console (`window.$gmap`).

pub mod brep;
pub mod debug_viewer;
pub mod geometry;
pub mod gmap;
pub mod hints;
pub mod ocp_vscode;
pub mod scene;

use serde::Serialize;

pub use geometry::{
    scene_from_curve, scene_from_plane, scene_from_point, scene_from_surface, scene_from_vector,
};
pub use hints::{Style, VizHints};
pub use scene::{VizAlphaLink, VizDart, VizEdge, VizFace, VizLabel, VizScene, VizVertex};

use crate::model::{Cell0, Model};
use crate::tessellate::{CurveOpts, SurfaceOpts, TessellateOpts};
use crate::topology::gmap::{Dart, Dim};
use crate::topology::payload::Payload;

const VIEWER_TESSELLATION_OPTS: TessellateOpts = TessellateOpts {
    curve: CurveOpts { segments: 64 },
    surface: SurfaceOpts { nu: 64, nv: 32 },
};

/// Builds a viewer-quality tessellated scene from a `Model`.
///
/// Curves use 64 segments and curved surfaces use a 64 × 32 grid. `hints`
/// carries presentation overrides (colors, labels, opacity); pass
/// [`VizHints::new`] when no overrides are needed. Lower-level tessellation
/// consumers keep the lighter [`TessellateOpts::default`] preset.
pub fn scene_from_model<P: Payload>(g: &Model<P>, hints: &VizHints) -> VizScene {
    scene_from_model_with_opts(g, hints, VIEWER_TESSELLATION_OPTS)
}

/// Same as [`scene_from_model`], with explicit tessellation knobs.
pub fn scene_from_model_with_opts<P: Payload>(
    g: &Model<P>,
    hints: &VizHints,
    opts: TessellateOpts,
) -> VizScene {
    let mut scene = VizScene::new();
    let index = brep::emit_brep(g, hints, opts, &mut scene);
    gmap::emit_gmap_overlay(g, hints, &index, opts, &mut scene);
    scene
}

/// A serializable dump of a `Model`'s state, consumable from JS.
///
/// - `alphas[i][d]` = id of αᵢ(d). Free darts map to themselves.
/// - `vertexPoints` is one entry per dart that carries a stamped position
///   (an attribute lookup, not a full orbit) — same convention the previous
///   IR used, kept here for the `window.$gmap` console API.
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

/// Snapshot every involution table and per-dart vertex position.
pub fn gmap_snapshot<P: Payload>(g: &Model<P>) -> GMapSnapshot {
    let dim = g.dimension();
    let n = g.dart_count();
    let mut alphas: Vec<Vec<u32>> = (0..dim).map(|_| Vec::with_capacity(n)).collect();
    for i in 0..dim {
        for id in 0..n {
            let d = Dart::new(id);
            alphas[i].push(g.alpha(Dim::from_index(i), d).id() as u32);
        }
    }
    let mut vertex_points = Vec::new();
    for id in 0..n {
        let d = Dart::new(id);
        if let Some(v) = g.attribute::<Cell0>(d) {
            vertex_points.push(VertexPointEntry {
                dart: id as u32,
                position: [v.point.x, v.point.y, v.point.z],
            });
        }
    }
    GMapSnapshot {
        dimension: dim as u32,
        dart_count: n as u32,
        alphas,
        vertex_points,
    }
}

/// What a script returns. `scene` is always present; `gmap` is the optional
/// inspectable snapshot (used by the JS-side `$gmap` console helper).
#[derive(Debug, Clone, Default, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct ScriptResult {
    pub scene: VizScene,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub gmap: Option<GMapSnapshot>,
}

impl ScriptResult {
    /// Build a result from a `Model` with default hints and no gmap snapshot.
    pub fn from_model<P: Payload>(g: &Model<P>) -> Self {
        Self {
            scene: scene_from_model(g, &VizHints::new()),
            gmap: None,
        }
    }

    /// Build a result from a `Model` + presentation hints, including the gmap
    /// snapshot so the front-end can introspect the topology.
    pub fn from_model_with_hints<P: Payload>(g: &Model<P>, hints: &VizHints) -> Self {
        Self {
            scene: scene_from_model(g, hints),
            gmap: Some(gmap_snapshot(g)),
        }
    }
}
