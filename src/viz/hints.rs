//! Per-entity styling that scripts attach to a GMap before handing it to the
//! visualization orchestrator.
//!
//! Hints are pure presentation data — colors, labels, opacity. The
//! tessellation pipeline never reads them; the BRep / GMap layers do, when
//! emitting [`super::scene::VizScene`] entities. Scripts can leave them
//! empty and get sensible defaults.

use std::collections::HashMap;

use crate::topology::shape_keys::{EdgeKey, FaceKey, VertexKey};

/// Style overrides for a single drawable entity. Fields left as `None` fall
/// back to the renderer's defaults.
#[derive(Debug, Clone, Default)]
pub struct Style {
    pub color: Option<String>,
    pub opacity: Option<f32>,
    pub label: Option<String>,
    pub width: Option<f32>,
    pub size: Option<f32>,
    pub double_sided: Option<bool>,
}

impl Style {
    pub fn color(mut self, c: impl Into<String>) -> Self {
        self.color = Some(c.into());
        self
    }
    pub fn label(mut self, l: impl Into<String>) -> Self {
        self.label = Some(l.into());
        self
    }
    pub fn opacity(mut self, o: f32) -> Self {
        self.opacity = Some(o);
        self
    }
    pub fn width(mut self, w: f32) -> Self {
        self.width = Some(w);
        self
    }
    pub fn size(mut self, s: f32) -> Self {
        self.size = Some(s);
        self
    }
    pub fn double_sided(mut self, d: bool) -> Self {
        self.double_sided = Some(d);
        self
    }
}

/// Bag of styles indexed by topology key / dart id. Scripts populate this and
/// hand it to [`scene_from_gmap`](super::scene_from_gmap).
#[derive(Debug, Clone, Default)]
pub struct VizHints {
    pub vertex_styles: HashMap<VertexKey, Style>,
    pub edge_styles: HashMap<EdgeKey, Style>,
    pub face_styles: HashMap<FaceKey, Style>,
    /// Per-dart override, keyed by raw dart id (matches `Dart::id`).
    pub dart_styles: HashMap<u32, Style>,
}

impl VizHints {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn face(&mut self, key: FaceKey, style: Style) -> &mut Self {
        self.face_styles.insert(key, style);
        self
    }
    pub fn edge(&mut self, key: EdgeKey, style: Style) -> &mut Self {
        self.edge_styles.insert(key, style);
        self
    }
    pub fn vertex(&mut self, key: VertexKey, style: Style) -> &mut Self {
        self.vertex_styles.insert(key, style);
        self
    }
    pub fn dart(&mut self, dart: u32, style: Style) -> &mut Self {
        self.dart_styles.insert(dart, style);
        self
    }
}
