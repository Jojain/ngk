use std::collections::HashSet;

use serde::Serialize;
use thiserror::Error;

use crate::geometry::Point3;
use crate::tessellate::{
    IndexedMesh, Polyline3, TessellateOpts, tessellate_edge, tessellate_face_key,
};
use crate::topology::edge::Edge;
use crate::topology::face::Face;
use crate::topology::gmap::{Dim, GMap};
use crate::topology::payload::Payload;
use crate::topology::profile::Profile;
use crate::topology::shape::{EdgeTag, FaceTag, ProfileTag, Shape, SolidTag};
use crate::topology::shape_keys::{EdgeKey, FaceKey};
use crate::topology::solid::Solid;

#[derive(Debug, Clone, Error)]
pub enum TcvError {
    #[error("missing topology for TCV export")]
    MissingTopology,
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
        let mut shape = TcvShape::default();
        append_edge(self.map(), self.handle(), opts.tessellate, &mut shape)?;
        let attr = self
            .map()
            .edge_attr(self.handle())
            .ok_or(TcvError::MissingTopology)?;
        append_edge_vertices(&attr.edge(self.map()), &mut shape);
        Ok(root_with_leaf(edge_leaf(&opts, shape), opts.name))
    }
}

impl<P: Payload> ToTcv for Shape<ProfileTag, P> {
    fn to_tcv(&self, opts: TcvOptions) -> Result<TcvNode, TcvError> {
        let mut shape = TcvShape::default();
        let profile = self.profile();
        append_profile(self.map(), &profile, opts.tessellate, &mut shape)?;
        append_profile_vertices(&profile, &mut shape);
        Ok(root_with_leaf(edge_leaf(&opts, shape), opts.name))
    }
}

impl<P: Payload> ToTcv for Shape<FaceTag, P> {
    fn to_tcv(&self, opts: TcvOptions) -> Result<TcvNode, TcvError> {
        let mut shape = TcvShape::default();
        append_face_mesh(self.map(), self.handle(), opts.tessellate, &mut shape)?;
        let attr = self
            .map()
            .face_attr(self.handle())
            .ok_or(TcvError::MissingTopology)?;
        let face = Face::new(self.map(), attr);
        append_profile(self.map(), &face.outer_loop(), opts.tessellate, &mut shape)?;
        for loop_ in face.inner_loops() {
            append_profile(self.map(), &loop_, opts.tessellate, &mut shape)?;
        }
        append_face_vertices(&face, &mut shape);
        Ok(root_with_leaf(shape_leaf(&opts, "face", shape), opts.name))
    }
}

impl<P: Payload> ToTcv for Shape<SolidTag, P> {
    fn to_tcv(&self, opts: TcvOptions) -> Result<TcvNode, TcvError> {
        let mut shape = TcvShape::default();
        let solid = self.solid();
        append_solid(self.map(), &solid, opts.tessellate, &mut shape)?;
        append_all_vertices(self.map(), &mut shape);
        Ok(root_with_leaf(shape_leaf(&opts, "solid", shape), opts.name))
    }
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

fn append_solid<P: Payload>(
    g: &GMap<P>,
    _solid: &Solid<'_, P>,
    opts: TessellateOpts,
    shape: &mut TcvShape,
) -> Result<(), TcvError> {
    for (key, _) in g.iter_faces() {
        append_face_mesh(g, key, opts, shape)?;
    }
    for key in unique_edge_keys(g) {
        append_edge(g, key, opts, shape)?;
    }
    Ok(())
}

fn append_face_mesh<P: Payload>(
    g: &GMap<P>,
    key: FaceKey,
    opts: TessellateOpts,
    shape: &mut TcvShape,
) -> Result<(), TcvError> {
    let mesh = tessellate_face_key(g, key, opts).ok_or(TcvError::MissingTopology)?;
    append_mesh(&mesh, shape);
    shape.face_types.push(0);
    Ok(())
}

fn append_mesh(mesh: &IndexedMesh, shape: &mut TcvShape) {
    let offset = (shape.vertices.len() / 3) as u32;
    for position in &mesh.positions {
        push_point(&mut shape.vertices, position);
    }
    for normal in &mesh.normals {
        shape.normals.extend([normal.x, normal.y, normal.z]);
    }
    shape
        .triangles
        .extend(mesh.indices.iter().map(|index| index + offset));
    shape
        .triangles_per_face
        .push((mesh.indices.len() / 3) as u32);
}

fn append_profile<P: Payload>(
    g: &GMap<P>,
    profile: &Profile<'_, P>,
    opts: TessellateOpts,
    shape: &mut TcvShape,
) -> Result<(), TcvError> {
    for edge in profile.edges() {
        let key = edge_key_from_edge(g, &edge).ok_or(TcvError::MissingTopology)?;
        append_edge(g, key, opts, shape)?;
    }
    Ok(())
}

fn append_edge<P: Payload>(
    g: &GMap<P>,
    key: EdgeKey,
    opts: TessellateOpts,
    shape: &mut TcvShape,
) -> Result<(), TcvError> {
    let attr = g.edge_attr(key).ok_or(TcvError::MissingTopology)?;
    let edge = attr.edge(g);
    let polyline = tessellate_edge(g, key, opts).filter(|line| !line.is_empty());
    let polyline = polyline.unwrap_or_else(|| fallback_chord(&edge));
    append_polyline(&polyline, shape);
    shape.edge_types.push(0);
    Ok(())
}

fn append_polyline(polyline: &Polyline3, shape: &mut TcvShape) {
    let mut segments = 0;
    for pair in polyline.points.windows(2) {
        push_point(&mut shape.edges, &pair[0]);
        push_point(&mut shape.edges, &pair[1]);
        segments += 1;
    }
    shape.segments_per_edge.push(segments);
}

fn fallback_chord<P: Payload>(edge: &Edge<'_, P>) -> Polyline3 {
    let points = edge
        .start()
        .point()
        .zip(edge.end().point())
        .map(|(start, end)| vec![*start, *end])
        .unwrap_or_default();
    Polyline3::new(points)
}

fn append_edge_vertices<P: Payload>(edge: &Edge<'_, P>, shape: &mut TcvShape) {
    if let Some(point) = edge.start().point() {
        push_point(&mut shape.obj_vertices, point);
    }
    if let Some(point) = edge.end().point() {
        push_point(&mut shape.obj_vertices, point);
    }
}

fn append_profile_vertices<P: Payload>(profile: &Profile<'_, P>, shape: &mut TcvShape) {
    for vertex in profile.vertices() {
        if let Some(point) = vertex.point() {
            push_point(&mut shape.obj_vertices, point);
        }
    }
}

fn append_face_vertices<P: Payload>(face: &Face<'_, P>, shape: &mut TcvShape) {
    for vertex in face.outer_loop().vertices() {
        if let Some(point) = vertex.point() {
            push_point(&mut shape.obj_vertices, point);
        }
    }
    for loop_ in face.inner_loops() {
        for vertex in loop_.vertices() {
            if let Some(point) = vertex.point() {
                push_point(&mut shape.obj_vertices, point);
            }
        }
    }
}

fn append_all_vertices<P: Payload>(g: &GMap<P>, shape: &mut TcvShape) {
    let mut seen = HashSet::new();
    for (_, attr) in g.iter_vertices() {
        let repr = g.cell_representative(attr.dart, Dim::Zero);
        if seen.insert(repr) {
            push_point(&mut shape.obj_vertices, &attr.point);
        }
    }
}

fn unique_edge_keys<P: Payload>(g: &GMap<P>) -> Vec<EdgeKey> {
    let mut seen = HashSet::new();
    let mut keys = Vec::new();
    for (key, attr) in g.iter_edges() {
        let repr = g.cell_representative(attr.dart, Dim::One);
        if seen.insert(repr) {
            keys.push(key);
        }
    }
    keys
}

fn push_point(values: &mut Vec<f64>, point: &Point3) {
    values.extend([point.x, point.y, point.z]);
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

fn edge_key_from_edge<P: Payload>(g: &GMap<P>, edge: &Edge<'_, P>) -> Option<EdgeKey> {
    let repr = g.cell_representative(edge.dart, Dim::One);
    g.dart_to_edge
        .get(&repr)
        .or_else(|| g.dart_to_edge.get(&edge.dart))
        .copied()
}
