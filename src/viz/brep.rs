//! BRep layer of the visualization pipeline.
//!
//! Walks a [`GMap`] and emits one [`VizVertex`] per stored 0-cell, one
//! [`VizEdge`] per stored 1-cell, one [`VizFace`] per stored 2-cell. The
//! actual geometry comes from [`crate::tessellate`]; this module only does
//! id-bookkeeping and applies styling from [`VizHints`].
//!
//! Per-scene ids:
//! - `vertex_id`, `edge_id`, `face_id` are sequential `u32`s assigned in
//!   iteration order. They have no semantic meaning beyond "stable within
//!   one scene", so the renderer can correlate hover targets with hints.
//! - The mapping back to topology keys is exposed via [`BrepIndex`] for
//!   downstream layers (notably [`super::gmap`]) that need to look up an
//!   `EdgeKey` from a dart.

use std::collections::HashMap;

use super::hints::{Style, VizHints};
use super::scene::{VizEdge, VizFace, VizScene, VizVertex};
use crate::tessellate::{TessellateOpts, tessellate_edge, tessellate_face};
use crate::topology::gmap::{Cell0, Dim, GMap};
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, FaceKey, VertexKey};

/// Lookup tables built while emitting the BRep layer. Lets the GMap overlay
/// resolve "which edge does this dart belong to" without re-scanning.
#[derive(Debug, Clone, Default)]
pub struct BrepIndex {
    pub edge_id_by_key: HashMap<EdgeKey, u32>,
    pub edge_key_by_dart: HashMap<u32, EdgeKey>,
    pub vertex_id_by_key: HashMap<VertexKey, u32>,
    pub face_id_by_key: HashMap<FaceKey, u32>,
}

/// Tessellate every BRep entity in `g` and append it to `scene`. Returns an
/// [`BrepIndex`] mapping topology keys to per-scene ids.
pub fn emit_brep<P: Payload>(
    g: &GMap<P>,
    hints: &VizHints,
    opts: TessellateOpts,
    scene: &mut VizScene,
) -> BrepIndex {
    let mut index = BrepIndex::default();
    emit_vertices(g, hints, scene, &mut index);
    emit_edges(g, hints, opts, scene, &mut index);
    emit_faces(g, hints, opts, scene, &mut index);
    index
}

fn emit_vertices<P: Payload>(
    g: &GMap<P>,
    hints: &VizHints,
    scene: &mut VizScene,
    index: &mut BrepIndex,
) {
    for (key, attr) in g.iter_vertices() {
        let id = scene.vertices.len() as u32;
        index.vertex_id_by_key.insert(key, id);
        let style = hints.vertex_styles.get(&key);
        scene.vertices.push(VizVertex {
            vertex_id: id,
            position: [attr.point.x, attr.point.y, attr.point.z],
            color: style.and_then(|s| s.color.clone()),
            size: style.and_then(|s| s.size),
            label: style.and_then(|s| s.label.clone()),
        });
    }
}

fn emit_edges<P: Payload>(
    g: &GMap<P>,
    hints: &VizHints,
    opts: TessellateOpts,
    scene: &mut VizScene,
    index: &mut BrepIndex,
) {
    for (key, attr) in g.iter_edges() {
        let id = scene.edges.len() as u32;
        index.edge_id_by_key.insert(key, id);

        // Map every dart in the 1-cell to this edge so the GMap layer can
        // find the curve from any dart.
        for d in g.orbit(attr.dart, g.orbit_indices(Dim::One)) {
            index.edge_key_by_dart.insert(d.id() as u32, key);
        }

        let polyline = match tessellate_edge(g, key, opts) {
            Some(p) if !p.is_empty() => {
                p.points.iter().map(|p| [p.x, p.y, p.z]).collect::<Vec<_>>()
            }
            _ => fallback_chord(g, attr),
        };
        let style = hints.edge_styles.get(&key);
        scene.edges.push(VizEdge {
            edge_id: id,
            polyline,
            color: style.and_then(|s| s.color.clone()),
            width: style.and_then(|s| s.width),
            label: style.and_then(|s| s.label.clone()),
        });
    }
}

fn emit_faces<P: Payload>(
    g: &GMap<P>,
    hints: &VizHints,
    opts: TessellateOpts,
    scene: &mut VizScene,
    index: &mut BrepIndex,
) {
    for (key, _) in g.iter_faces() {
        let id = scene.faces.len() as u32;
        index.face_id_by_key.insert(key, id);
        let Some(mesh) = tessellate_face(g, key, opts) else {
            continue;
        };
        if mesh.is_empty() {
            continue;
        }
        let positions: Vec<[f64; 3]> = mesh.positions.iter().map(|p| [p.x, p.y, p.z]).collect();
        let normals: Vec<[f64; 3]> = mesh.normals.iter().map(|n| [n.x, n.y, n.z]).collect();
        let style = hints.face_styles.get(&key);
        scene.faces.push(VizFace {
            face_id: id,
            positions,
            normals,
            indices: mesh.indices,
            color: style.and_then(|s| s.color.clone()),
            opacity: style.and_then(|s| s.opacity),
            double_sided: style.and_then(|s| s.double_sided),
            label: style.and_then(|s| s.label.clone()),
        });
    }
}

/// Last-ditch polyline when `tessellate_edge` couldn't produce one (e.g. a
/// vertex's stored point lies off the curve and `param_at` is broken). We
/// still want to draw *something*, so fall back to the chord between the
/// dart's two endpoints.
fn fallback_chord<P: Payload>(
    g: &GMap<P>,
    attr: &crate::topology::attributes::EdgeAttr<P::E>,
) -> Vec<[f64; 3]> {
    let dart = attr.dart;
    let other = g.alpha(Dim::Zero, dart);
    let p0 = g
        .attribute::<Cell0>(dart)
        .map(|v| [v.point.x, v.point.y, v.point.z]);
    let p1 = g
        .attribute::<Cell0>(other)
        .map(|v| [v.point.x, v.point.y, v.point.z]);
    match (p0, p1) {
        (Some(a), Some(b)) => vec![a, b],
        _ => Vec::new(),
    }
}

/// Apply a `Style` override on top of an existing entity's optional fields,
/// preferring the style. Used by the GMap overlay for darts.
#[allow(dead_code)]
pub(crate) fn merge_color(base: Option<String>, style: Option<&Style>) -> Option<String> {
    style.and_then(|s| s.color.clone()).or(base)
}
