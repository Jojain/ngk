use serde::Serialize;
use thiserror::Error;

use crate::tessellate::{Tessellate, TessellateOpts, Tessellation, TessellationError};
use crate::topology::payload::Payload;
use crate::topology::shape::{EdgeTag, FaceTag, ProfileTag, Shape, SolidTag};

#[derive(Debug, Clone, Error)]
pub enum TcvError {
    #[error(transparent)]
    Tessellation(#[from] TessellationError),
}

#[derive(Debug, Clone)]
pub struct TcvOptions {
    pub name: String,
    pub color: String,
    pub alpha: f64,
    pub tessellate: TessellateOpts,
}

impl TcvOptions {
    pub fn named(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            ..Self::default()
        }
    }

    /// Uses the sampling density appropriate for an interactive CAD viewer.
    ///
    /// TCV is consumed by viewers that expose curved faces directly, so its
    /// default preview must not use the lighter geometry-inspection sampling.
    pub fn viewer(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            tessellate: crate::tessellate::VIEWER,
            ..Self::default()
        }
    }
}

impl Default for TcvOptions {
    fn default() -> Self {
        Self {
            name: "shape".to_string(),
            color: "#e8b024".to_string(),
            alpha: 1.0,
            tessellate: TessellateOpts::default(),
        }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct TcvBoundingBox {
    pub xmin: f64,
    pub xmax: f64,
    pub ymin: f64,
    pub ymax: f64,
    pub zmin: f64,
    pub zmax: f64,
}

#[derive(Debug, Clone, Serialize)]
pub struct TcvNode {
    pub version: u8,
    pub name: String,
    pub id: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub parts: Option<Vec<TcvNode>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub shape: Option<TcvShape>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub loc: Option<([f64; 3], [f64; 4])>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bb: Option<TcvBoundingBox>,
    #[serde(rename = "type", skip_serializing_if = "Option::is_none")]
    pub kind: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub subtype: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub color: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub alpha: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub renderback: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub state: Option<[u8; 2]>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub accuracy: Option<Option<f64>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub normal_len: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub width: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub size: Option<f64>,
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct TcvShape {
    pub vertices: Vec<f64>,
    pub normals: Vec<f64>,
    pub triangles: Vec<u32>,
    pub edges: Vec<f64>,
    pub obj_vertices: Vec<f64>,
    pub face_types: Vec<u32>,
    pub edge_types: Vec<u32>,
    pub triangles_per_face: Vec<u32>,
    pub segments_per_edge: Vec<u32>,
}

pub trait ToTcv {
    fn to_tcv(&self, opts: TcvOptions) -> Result<TcvNode, TcvError>;
}

pub fn to_tcv<T: ToTcv>(shape: &T, opts: TcvOptions) -> Result<TcvNode, TcvError> {
    shape.to_tcv(opts)
}

impl<P: Payload> ToTcv for Shape<EdgeTag, P> {
    fn to_tcv(&self, opts: TcvOptions) -> Result<TcvNode, TcvError> {
        let shape = tcv_shape(self.edge().tessellate_with(opts.tessellate)?);
        Ok(root_with_leaf(edge_leaf(&opts, shape), opts.name))
    }
}

impl<P: Payload> ToTcv for Shape<ProfileTag, P> {
    fn to_tcv(&self, opts: TcvOptions) -> Result<TcvNode, TcvError> {
        let shape = tcv_shape(self.profile().tessellate_with(opts.tessellate)?);
        Ok(root_with_leaf(edge_leaf(&opts, shape), opts.name))
    }
}

impl<P: Payload> ToTcv for Shape<FaceTag, P> {
    fn to_tcv(&self, opts: TcvOptions) -> Result<TcvNode, TcvError> {
        let shape = tcv_shape(self.face().tessellate_with(opts.tessellate)?);
        Ok(root_with_leaf(shape_leaf(&opts, "face", shape), opts.name))
    }
}

impl<P: Payload> ToTcv for Shape<SolidTag, P> {
    fn to_tcv(&self, opts: TcvOptions) -> Result<TcvNode, TcvError> {
        let shape = tcv_shape(self.solid().tessellate_with(opts.tessellate)?);
        Ok(root_with_leaf(shape_leaf(&opts, "solid", shape), opts.name))
    }
}

/// Lays a keyed tessellation out the way TCV reads it: triangle counts per
/// face, each edge's polyline as segment pairs, and the vertices as points.
fn tcv_shape(tessellation: Tessellation) -> TcvShape {
    let Tessellation {
        mesh,
        faces,
        edge_points,
        edges,
        vertices,
    } = tessellation;
    let mut shape = TcvShape {
        vertices: mesh
            .positions
            .iter()
            .flat_map(|p| [p.x, p.y, p.z])
            .collect(),
        normals: mesh.normals.iter().flat_map(|n| [n.x, n.y, n.z]).collect(),
        triangles: mesh.indices,
        triangles_per_face: faces.iter().map(|range| (range.count / 3) as u32).collect(),
        face_types: vec![0; faces.len()],
        edge_types: vec![0; edges.len()],
        obj_vertices: vertices
            .iter()
            .flat_map(|vertex| [vertex.point.x, vertex.point.y, vertex.point.z])
            .collect(),
        ..TcvShape::default()
    };
    for range in &edges {
        let polyline = &edge_points[range.start..range.start + range.count];
        for pair in polyline.windows(2) {
            shape.edges.extend([
                pair[0].x, pair[0].y, pair[0].z, pair[1].x, pair[1].y, pair[1].z,
            ]);
        }
        shape
            .segments_per_edge
            .push(range.count.saturating_sub(1) as u32);
    }
    shape
}

fn root_with_leaf(leaf: TcvNode, name: String) -> TcvNode {
    let bb = leaf.shape.as_ref().and_then(bounding_box);
    TcvNode {
        version: 3,
        name: name.clone(),
        id: format!("/{name}"),
        parts: Some(vec![leaf]),
        shape: None,
        loc: None,
        bb,
        kind: None,
        subtype: None,
        color: None,
        alpha: None,
        renderback: None,
        state: None,
        accuracy: None,
        normal_len: None,
        width: None,
        size: None,
    }
}

fn edge_leaf(opts: &TcvOptions, shape: TcvShape) -> TcvNode {
    leaf(opts, "edges", None, [3, 1], shape)
}

fn shape_leaf(opts: &TcvOptions, subtype: &str, shape: TcvShape) -> TcvNode {
    leaf(opts, "shapes", Some(subtype), [1, 1], shape)
}

fn leaf(
    opts: &TcvOptions,
    kind: &str,
    subtype: Option<&str>,
    state: [u8; 2],
    shape: TcvShape,
) -> TcvNode {
    TcvNode {
        version: 3,
        name: opts.name.clone(),
        id: format!("/{}/{}", opts.name, opts.name),
        parts: None,
        shape: Some(shape),
        loc: None,
        bb: None,
        kind: Some(kind.to_string()),
        subtype: subtype.map(String::from),
        color: Some(opts.color.clone()),
        alpha: Some(opts.alpha),
        renderback: Some(false),
        state: Some(state),
        accuracy: Some(None),
        normal_len: Some(0.0),
        width: if kind == "edges" { Some(2.0) } else { None },
        size: None,
    }
}

fn bounding_box(shape: &TcvShape) -> Option<TcvBoundingBox> {
    let mut chunks = shape
        .vertices
        .chunks_exact(3)
        .chain(shape.obj_vertices.chunks_exact(3));
    let first = chunks.next()?;
    let mut bb = TcvBoundingBox {
        xmin: first[0],
        xmax: first[0],
        ymin: first[1],
        ymax: first[1],
        zmin: first[2],
        zmax: first[2],
    };
    for chunk in chunks {
        bb.xmin = bb.xmin.min(chunk[0]);
        bb.xmax = bb.xmax.max(chunk[0]);
        bb.ymin = bb.ymin.min(chunk[1]);
        bb.ymax = bb.ymax.max(chunk[1]);
        bb.zmin = bb.zmin.min(chunk[2]);
        bb.zmax = bb.zmax.max(chunk[2]);
    }
    Some(bb)
}
