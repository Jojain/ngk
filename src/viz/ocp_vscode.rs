//! Send NGK shapes to the OCP CAD Viewer used by `ocp_vscode`.
//!
//! The Python package sends a JSON payload over a localhost WebSocket with a
//! `D:` prefix. This module mirrors that transport and adapts NGK's existing
//! [`VizScene`](super::VizScene) into the `three-cad-viewer` version 3 shape
//! tree consumed by the viewer.

use std::collections::HashSet;
use std::env;
use std::io::{Read, Write};
use std::mem::size_of;
use std::net::{TcpStream, ToSocketAddrs};
use std::time::Duration;

use serde::Serialize;
use thiserror::Error;

use super::{VizScene, VizVertex, scene_from_gmap};
use crate::topology::edge::Edge;
use crate::topology::face::Face;
use crate::topology::facet::Facet;
use crate::topology::gmap::{GMap, MergeTopology};
use crate::topology::payload::Payload;
use crate::topology::profile::Profile;
use crate::topology::shape::{Shape, ShapeKind};
use crate::topology::sheet::Sheet;
use crate::topology::solid::Solid;
use crate::topology::vertex::Vertex;

const DEFAULT_HOST: &str = "127.0.0.1";
const DEFAULT_PORT: u16 = 3939;
const ROOT_NAME: &str = "NGK";
const DEFAULT_COLOR: &str = "#4a7bc8";
const DEFAULT_EDGE_COLOR: &str = "#707070";
const DEFAULT_VERTEX_COLOR: &str = "MediumOrchid";
const DEFAULT_EDGE_WIDTH: f32 = 2.0;
const DEFAULT_VERTEX_SIZE: f32 = 6.0;

#[derive(Debug, Error)]
pub enum OcpVscodeError {
    #[error("failed to serialize OCP viewer payload: {0}")]
    Serialize(#[from] serde_json::Error),
    #[error("failed to connect to OCP viewer on {host}:{port}: {source}")]
    Connect {
        host: String,
        port: u16,
        #[source]
        source: std::io::Error,
    },
    #[error("OCP viewer rejected the WebSocket handshake: {0}")]
    Handshake(String),
    #[error("failed to send OCP viewer payload: {0}")]
    Send(#[from] std::io::Error),
}

#[derive(Debug, Clone)]
pub struct OcpViewerOptions {
    pub host: String,
    pub port: u16,
    pub name: String,
    pub reset_camera: String,
    pub axes: bool,
    pub axes0: bool,
    pub grid: [bool; 3],
    pub ortho: bool,
    pub transparent: bool,
    pub default_opacity: f32,
    pub color: String,
    pub edge_color: String,
    pub vertex_color: String,
}

impl Default for OcpViewerOptions {
    fn default() -> Self {
        Self {
            host: DEFAULT_HOST.to_owned(),
            port: env::var("OCP_PORT")
                .ok()
                .and_then(|value| value.parse().ok())
                .unwrap_or(DEFAULT_PORT),
            name: "shape".to_owned(),
            reset_camera: "reset".to_owned(),
            axes: false,
            axes0: false,
            grid: [false; 3],
            ortho: true,
            transparent: false,
            default_opacity: 0.5,
            color: DEFAULT_COLOR.to_owned(),
            edge_color: DEFAULT_EDGE_COLOR.to_owned(),
            vertex_color: DEFAULT_VERTEX_COLOR.to_owned(),
        }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct OcpViewerPayload {
    pub data: OcpViewerData,
    #[serde(rename = "type")]
    pub message_type: String,
    pub config: OcpViewerConfig,
    pub count: u32,
}

#[derive(Debug, Clone, Serialize)]
pub struct OcpViewerData {
    pub instances: Vec<OcpShape>,
    pub shapes: OcpShapes,
}

#[derive(Debug, Clone, Serialize)]
pub struct OcpViewerConfig {
    #[serde(rename = "reset_camera")]
    pub reset_camera: String,
    #[serde(rename = "render_edges")]
    pub render_edges: bool,
    pub axes: bool,
    pub axes0: bool,
    pub grid: [bool; 3],
    pub ortho: bool,
    pub transparent: bool,
    #[serde(rename = "default_opacity")]
    pub default_opacity: f32,
    #[serde(rename = "default_color")]
    pub default_color: String,
    #[serde(rename = "default_edgecolor")]
    pub default_edgecolor: String,
    #[serde(rename = "default_vertexcolor")]
    pub default_vertexcolor: String,
    #[serde(rename = "ambient_intensity")]
    pub ambient_intensity: f32,
    #[serde(rename = "direct_intensity")]
    pub direct_intensity: f32,
    pub metalness: f32,
    pub roughness: f32,
    pub collapse: u8,
    pub up: String,
}

#[derive(Debug, Clone, Serialize)]
pub struct OcpShapes {
    pub version: u32,
    pub parts: Vec<OcpPart>,
    pub loc: OcpLocation,
    pub name: String,
    pub id: String,
    #[serde(rename = "normal_len")]
    pub normal_len: f64,
    pub bb: OcpBoundingBox,
}

#[derive(Debug, Clone, Serialize)]
pub struct OcpPart {
    pub id: String,
    #[serde(rename = "type")]
    pub kind: String,
    pub subtype: String,
    pub name: String,
    pub shape: OcpShapeRef,
    pub state: [u8; 2],
    pub color: String,
    pub alpha: f32,
    pub material: Option<String>,
    pub normalize_uvs: bool,
    pub texture: Option<String>,
    pub loc: OcpLocation,
    pub renderback: bool,
    pub accuracy: Option<f64>,
    pub bb: Option<OcpBoundingBox>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub width: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub size: Option<f32>,
}

#[derive(Debug, Clone, Serialize)]
pub struct OcpShapeRef {
    #[serde(rename = "ref")]
    pub reference: u32,
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct OcpShape {
    pub vertices: OcpBuffer,
    pub triangles: OcpBuffer,
    pub normals: OcpBuffer,
    pub edges: OcpBuffer,
    pub obj_vertices: OcpBuffer,
    pub face_types: OcpBuffer,
    pub edge_types: OcpBuffer,
    pub triangles_per_face: OcpBuffer,
    pub segments_per_edge: OcpBuffer,
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct OcpBuffer {
    pub shape: [usize; 1],
    pub dtype: String,
    pub buffer: String,
    pub codec: String,
}

#[derive(Debug, Clone, Default)]
struct RawOcpShape {
    vertices: Vec<f32>,
    triangles: Vec<i32>,
    normals: Vec<f32>,
    edges: Vec<f32>,
    obj_vertices: Vec<f32>,
    face_types: Vec<i32>,
    edge_types: Vec<i32>,
    triangles_per_face: Vec<i32>,
    segments_per_edge: Vec<i32>,
}

#[derive(Debug, Clone, Copy, Serialize)]
pub struct OcpBoundingBox {
    pub xmin: f64,
    pub xmax: f64,
    pub ymin: f64,
    pub ymax: f64,
    pub zmin: f64,
    pub zmax: f64,
}

impl OcpBoundingBox {
    fn empty() -> Self {
        Self {
            xmin: -1e-6,
            xmax: 1e-6,
            ymin: -1e-6,
            ymax: 1e-6,
            zmin: -1e-6,
            zmax: 1e-6,
        }
    }

    fn include(&mut self, point: [f64; 3]) {
        self.xmin = self.xmin.min(point[0]);
        self.xmax = self.xmax.max(point[0]);
        self.ymin = self.ymin.min(point[1]);
        self.ymax = self.ymax.max(point[1]);
        self.zmin = self.zmin.min(point[2]);
        self.zmax = self.zmax.max(point[2]);
    }
}

pub type OcpLocation = ([f64; 3], [f64; 4]);

#[derive(Debug, Clone, Copy)]
enum OcpDisplayRole {
    Shape,
    Edge,
    Vertex,
}

pub struct OcpDisplayItem {
    scene: VizScene,
    role: OcpDisplayRole,
}

impl OcpDisplayItem {
    fn shape(scene: VizScene) -> Self {
        Self {
            scene,
            role: OcpDisplayRole::Shape,
        }
    }

    fn edge(scene: VizScene) -> Self {
        Self {
            scene,
            role: OcpDisplayRole::Edge,
        }
    }

    fn vertex(scene: VizScene) -> Self {
        Self {
            scene,
            role: OcpDisplayRole::Vertex,
        }
    }
}

/// Values that can be displayed in the OCP CAD Viewer.
///
/// Implemented for owned [`Shape`] values, typed topology views such as
/// [`Face`] and [`Edge`], and collections (`Vec<T>`, slices, arrays). This
/// lets callers use the same entry point for `show(&solid)` and
/// `show(&solid.solid().faces()[0..5])`.
pub trait OcpDisplay {
    fn append_ocp_items(&self, items: &mut Vec<OcpDisplayItem>);
}

impl<T: OcpDisplay + ?Sized> OcpDisplay for &T {
    fn append_ocp_items(&self, items: &mut Vec<OcpDisplayItem>) {
        (*self).append_ocp_items(items);
    }
}

impl<T: OcpDisplay> OcpDisplay for Vec<T> {
    fn append_ocp_items(&self, items: &mut Vec<OcpDisplayItem>) {
        self.as_slice().append_ocp_items(items);
    }
}

impl<T: OcpDisplay> OcpDisplay for [T] {
    fn append_ocp_items(&self, items: &mut Vec<OcpDisplayItem>) {
        for item in self {
            item.append_ocp_items(items);
        }
    }
}

impl<T: OcpDisplay, const N: usize> OcpDisplay for [T; N] {
    fn append_ocp_items(&self, items: &mut Vec<OcpDisplayItem>) {
        self.as_slice().append_ocp_items(items);
    }
}

impl<K: ShapeKind, P: Payload> OcpDisplay for Shape<K, P> {
    fn append_ocp_items(&self, items: &mut Vec<OcpDisplayItem>) {
        items.push(OcpDisplayItem::shape(scene_from_gmap(
            self.map(),
            &super::VizHints::new(),
        )));
    }
}

impl<P: Payload> OcpDisplay for Vertex<'_, P> {
    fn append_ocp_items(&self, items: &mut Vec<OcpDisplayItem>) {
        items.push(OcpDisplayItem::vertex(scene_from_topology::<P, _>(self)));
    }
}

impl<P: Payload> OcpDisplay for Edge<'_, P> {
    fn append_ocp_items(&self, items: &mut Vec<OcpDisplayItem>) {
        items.push(OcpDisplayItem::edge(scene_from_topology::<P, _>(self)));
    }
}

impl<P: Payload> OcpDisplay for Profile<'_, P> {
    fn append_ocp_items(&self, items: &mut Vec<OcpDisplayItem>) {
        items.push(OcpDisplayItem::edge(scene_from_topology::<P, _>(self)));
    }
}

impl<P: Payload> OcpDisplay for Face<'_, P> {
    fn append_ocp_items(&self, items: &mut Vec<OcpDisplayItem>) {
        items.push(OcpDisplayItem::shape(scene_from_topology::<P, _>(self)));
    }
}

impl<P: Payload> OcpDisplay for Facet<'_, P> {
    fn append_ocp_items(&self, items: &mut Vec<OcpDisplayItem>) {
        items.push(OcpDisplayItem::shape(scene_from_topology::<P, _>(self)));
    }
}

impl<P: Payload> OcpDisplay for Sheet<'_, P> {
    fn append_ocp_items(&self, items: &mut Vec<OcpDisplayItem>) {
        items.push(OcpDisplayItem::shape(scene_from_topology::<P, _>(self)));
    }
}

impl<P: Payload> OcpDisplay for Solid<'_, P> {
    fn append_ocp_items(&self, items: &mut Vec<OcpDisplayItem>) {
        items.push(OcpDisplayItem::shape(scene_from_topology::<P, _>(self)));
    }
}

fn scene_from_topology<P, T>(topology: T) -> VizScene
where
    P: Payload,
    T: MergeTopology<P>,
{
    let (g, _) = GMap::isolate(topology);
    scene_from_gmap(&g, &super::VizHints::new())
}

/// Serialize and send NGK display items to a running OCP CAD Viewer.
pub fn show<T: OcpDisplay + ?Sized>(display: &T) -> Result<(), OcpVscodeError> {
    show_with_options(display, &OcpViewerOptions::default())
}

/// Serialize and send NGK display items using explicit viewer options.
pub fn show_with_options<T: OcpDisplay + ?Sized>(
    display: &T,
    options: &OcpViewerOptions,
) -> Result<(), OcpVscodeError> {
    let payload = payload_for_display(display, options)?;
    send_payload(&payload, options)
}

/// Serialize and send a raw GMap to a running OCP CAD Viewer.
pub fn show_gmap<P: Payload>(g: &GMap<P>) -> Result<(), OcpVscodeError> {
    show_gmap_with_options(g, &OcpViewerOptions::default())
}

/// Serialize and send a raw GMap using explicit viewer options.
pub fn show_gmap_with_options<P: Payload>(
    g: &GMap<P>,
    options: &OcpViewerOptions,
) -> Result<(), OcpVscodeError> {
    let payload = payload_for_gmap(g, options)?;
    send_payload(&payload, options)
}

/// Build the OCP viewer payload for an NGK shape without sending it.
pub fn payload_for_shape<K: ShapeKind, P: Payload>(
    shape: &Shape<K, P>,
    options: &OcpViewerOptions,
) -> Result<OcpViewerPayload, OcpVscodeError> {
    payload_for_display(shape, options)
}

/// Build the OCP viewer payload for one or more NGK display items without sending it.
pub fn payload_for_display<T: OcpDisplay + ?Sized>(
    display: &T,
    options: &OcpViewerOptions,
) -> Result<OcpViewerPayload, OcpVscodeError> {
    let mut items = Vec::new();
    display.append_ocp_items(&mut items);
    Ok(payload_for_items(&items, options))
}

/// Build the OCP viewer payload for a raw GMap without sending it.
pub fn payload_for_gmap<P: Payload>(
    g: &GMap<P>,
    options: &OcpViewerOptions,
) -> Result<OcpViewerPayload, OcpVscodeError> {
    Ok(payload_for_scene(
        &scene_from_gmap(g, &super::VizHints::new()),
        options,
    ))
}

/// Build the OCP viewer payload for an already-tessellated scene.
pub fn payload_for_scene(scene: &VizScene, options: &OcpViewerOptions) -> OcpViewerPayload {
    payload_for_scenes(std::slice::from_ref(scene), options)
}

/// Build the OCP viewer payload for already-tessellated scenes.
pub fn payload_for_scenes(scenes: &[VizScene], options: &OcpViewerOptions) -> OcpViewerPayload {
    let items = scenes
        .iter()
        .cloned()
        .map(OcpDisplayItem::shape)
        .collect::<Vec<_>>();
    payload_for_items(&items, options)
}

fn payload_for_items(items: &[OcpDisplayItem], options: &OcpViewerOptions) -> OcpViewerPayload {
    let part_name = clean_part_name(&options.name);
    let mut instances = Vec::with_capacity(items.len());
    let mut parts = Vec::with_capacity(items.len());
    let mut bb: Option<OcpBoundingBox> = None;

    for (index, item) in items.iter().enumerate() {
        let scene = &item.scene;
        instances.push(ocp_shape_from_item(item));
        let scene_bb = bounding_box(scene);
        bb = Some(match bb {
            Some(mut bb) => {
                bb.include([scene_bb.xmin, scene_bb.ymin, scene_bb.zmin]);
                bb.include([scene_bb.xmax, scene_bb.ymax, scene_bb.zmax]);
                bb
            }
            None => scene_bb,
        });

        let name = if items.len() == 1 {
            part_name.clone()
        } else {
            format!("{part_name}_{index}")
        };
        let part_kind = part_kind_for_role(item.role);
        parts.push(OcpPart {
            id: format!("/{ROOT_NAME}/{name}"),
            kind: part_kind.kind.to_owned(),
            subtype: part_kind.subtype.to_owned(),
            name,
            shape: OcpShapeRef {
                reference: index as u32,
            },
            state: part_kind.state,
            color: part_kind.color(options),
            alpha: 1.0,
            material: None,
            normalize_uvs: true,
            texture: None,
            loc: identity_location(),
            renderback: true,
            accuracy: None,
            bb: None,
            width: part_kind.width,
            size: part_kind.size,
        });
    }

    OcpViewerPayload {
        data: OcpViewerData {
            instances,
            shapes: OcpShapes {
                version: 3,
                parts,
                loc: identity_location(),
                name: ROOT_NAME.to_owned(),
                id: format!("/{ROOT_NAME}"),
                normal_len: 0.0,
                bb: bb.unwrap_or_else(OcpBoundingBox::empty),
            },
        },
        message_type: "data".to_owned(),
        config: config_from_options(options),
        count: items.len() as u32,
    }
}

struct OcpPartKind {
    kind: &'static str,
    subtype: &'static str,
    state: [u8; 2],
    width: Option<f32>,
    size: Option<f32>,
}

impl OcpPartKind {
    fn color(&self, options: &OcpViewerOptions) -> String {
        match self.kind {
            "edges" => options.edge_color.clone(),
            "vertices" => options.vertex_color.clone(),
            _ => options.color.clone(),
        }
    }
}

fn part_kind_for_role(role: OcpDisplayRole) -> OcpPartKind {
    match role {
        OcpDisplayRole::Shape => OcpPartKind {
            kind: "shapes",
            subtype: "solid",
            state: [1, 1],
            width: None,
            size: None,
        },
        OcpDisplayRole::Edge => OcpPartKind {
            kind: "edges",
            subtype: "edge",
            state: [3, 1],
            width: Some(DEFAULT_EDGE_WIDTH),
            size: None,
        },
        OcpDisplayRole::Vertex => OcpPartKind {
            kind: "vertices",
            subtype: "vertex",
            state: [3, 1],
            width: None,
            size: Some(DEFAULT_VERTEX_SIZE),
        },
    }
}

fn ocp_shape_from_item(item: &OcpDisplayItem) -> OcpShape {
    match item.role {
        OcpDisplayRole::Shape => ocp_shape_from_scene(&item.scene),
        OcpDisplayRole::Edge => ocp_shape_from_scene(&VizScene {
            edges: item.scene.edges.clone(),
            ..VizScene::new()
        }),
        OcpDisplayRole::Vertex => ocp_shape_from_scene(&VizScene {
            vertices: item.scene.vertices.clone(),
            ..VizScene::new()
        }),
    }
}

pub fn send_payload(
    payload: &OcpViewerPayload,
    options: &OcpViewerOptions,
) -> Result<(), OcpVscodeError> {
    let json = serde_json::to_string(payload)?;
    send_prefixed_json(&options.host, options.port, &json)
}

fn ocp_shape_from_scene(scene: &VizScene) -> OcpShape {
    let mut shape = RawOcpShape::default();

    for face in &scene.faces {
        let base = (shape.vertices.len() / 3) as u32;
        for position in &face.positions {
            push_point(&mut shape.vertices, *position);
        }
        for index in &face.indices {
            shape.triangles.push((base + index) as i32);
        }
        push_normals(&mut shape.normals, &face.positions, &face.normals);
        shape.face_types.push(0);
        shape
            .triangles_per_face
            .push((face.indices.len() / 3) as i32);
    }

    let mut seen_edges = HashSet::new();
    for edge in &scene.edges {
        if edge.polyline.len() < 2 {
            continue;
        }
        if !seen_edges.insert(edge_key(&edge.polyline)) {
            continue;
        }

        let mut segments = 0;
        for segment in edge.polyline.windows(2) {
            push_point(&mut shape.edges, segment[0]);
            push_point(&mut shape.edges, segment[1]);
            segments += 1;
        }
        shape.edge_types.push(0);
        shape.segments_per_edge.push(segments);
    }

    let mut seen_vertices = HashSet::new();
    for vertex in &scene.vertices {
        if !seen_vertices.insert(point_key(vertex.position)) {
            continue;
        }
        push_vertex(&mut shape.obj_vertices, vertex);
    }

    encode_shape(shape)
}

fn push_normals(normals: &mut Vec<f32>, positions: &[[f64; 3]], source_normals: &[[f64; 3]]) {
    for i in 0..positions.len() {
        let normal = source_normals.get(i).copied().unwrap_or([0.0, 0.0, 1.0]);
        push_point(normals, normal);
    }
}

fn push_vertex(target: &mut Vec<f32>, vertex: &VizVertex) {
    push_point(target, vertex.position);
}

fn push_point(target: &mut Vec<f32>, point: [f64; 3]) {
    target.extend(point.map(|coord| coord as f32));
}

fn encode_shape(shape: RawOcpShape) -> OcpShape {
    OcpShape {
        vertices: encode_f32_buffer(shape.vertices),
        triangles: encode_i32_buffer(shape.triangles),
        normals: encode_f32_buffer(shape.normals),
        edges: encode_f32_buffer(shape.edges),
        obj_vertices: encode_f32_buffer(shape.obj_vertices),
        face_types: encode_i32_buffer(shape.face_types),
        edge_types: encode_i32_buffer(shape.edge_types),
        triangles_per_face: encode_i32_buffer(shape.triangles_per_face),
        segments_per_edge: encode_i32_buffer(shape.segments_per_edge),
    }
}

fn encode_f32_buffer(values: Vec<f32>) -> OcpBuffer {
    let mut bytes = Vec::with_capacity(values.len() * size_of::<f32>());
    for value in &values {
        bytes.extend_from_slice(&value.to_le_bytes());
    }
    OcpBuffer {
        shape: [values.len()],
        dtype: "float32".to_owned(),
        buffer: base64_encode(&bytes),
        codec: "b64".to_owned(),
    }
}

fn encode_i32_buffer(values: Vec<i32>) -> OcpBuffer {
    let mut bytes = Vec::with_capacity(values.len() * size_of::<i32>());
    for value in &values {
        bytes.extend_from_slice(&value.to_le_bytes());
    }
    OcpBuffer {
        shape: [values.len()],
        dtype: "int32".to_owned(),
        buffer: base64_encode(&bytes),
        codec: "b64".to_owned(),
    }
}

fn base64_encode(bytes: &[u8]) -> String {
    const TABLE: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut encoded = String::with_capacity(bytes.len().div_ceil(3) * 4);

    for chunk in bytes.chunks(3) {
        let b0 = chunk[0];
        let b1 = chunk.get(1).copied().unwrap_or(0);
        let b2 = chunk.get(2).copied().unwrap_or(0);
        let triple = ((b0 as u32) << 16) | ((b1 as u32) << 8) | b2 as u32;

        encoded.push(TABLE[((triple >> 18) & 0x3f) as usize] as char);
        encoded.push(TABLE[((triple >> 12) & 0x3f) as usize] as char);
        if chunk.len() > 1 {
            encoded.push(TABLE[((triple >> 6) & 0x3f) as usize] as char);
        } else {
            encoded.push('=');
        }
        if chunk.len() > 2 {
            encoded.push(TABLE[(triple & 0x3f) as usize] as char);
        } else {
            encoded.push('=');
        }
    }

    encoded
}

fn edge_key(polyline: &[[f64; 3]]) -> String {
    let forward = polyline
        .iter()
        .map(|point| point_key(*point))
        .collect::<Vec<_>>()
        .join("|");
    let reverse = polyline
        .iter()
        .rev()
        .map(|point| point_key(*point))
        .collect::<Vec<_>>()
        .join("|");
    if forward <= reverse { forward } else { reverse }
}

fn point_key(point: [f64; 3]) -> String {
    format!(
        "{:.9},{:.9},{:.9}",
        rounded_key_coord(point[0]),
        rounded_key_coord(point[1]),
        rounded_key_coord(point[2])
    )
}

fn rounded_key_coord(value: f64) -> f64 {
    let rounded = (value * 1e9).round() / 1e9;
    if rounded == 0.0 { 0.0 } else { rounded }
}

fn bounding_box(scene: &VizScene) -> OcpBoundingBox {
    let mut bb = OcpBoundingBox::empty();
    let mut has_point = false;

    for vertex in &scene.vertices {
        bb.include(vertex.position);
        has_point = true;
    }
    for edge in &scene.edges {
        for point in &edge.polyline {
            bb.include(*point);
            has_point = true;
        }
    }
    for face in &scene.faces {
        for point in &face.positions {
            bb.include(*point);
            has_point = true;
        }
    }

    if has_point {
        bb
    } else {
        OcpBoundingBox::empty()
    }
}

fn config_from_options(options: &OcpViewerOptions) -> OcpViewerConfig {
    OcpViewerConfig {
        reset_camera: options.reset_camera.clone(),
        render_edges: true,
        axes: options.axes,
        axes0: options.axes0,
        grid: options.grid,
        ortho: options.ortho,
        transparent: options.transparent,
        default_opacity: options.default_opacity,
        default_color: options.color.clone(),
        default_edgecolor: options.edge_color.clone(),
        default_vertexcolor: options.vertex_color.clone(),
        ambient_intensity: 1.0,
        direct_intensity: 1.1,
        metalness: 0.3,
        roughness: 0.65,
        collapse: 1,
        up: "Z".to_owned(),
    }
}

fn clean_part_name(name: &str) -> String {
    let clean = name.replace(['/', '\\'], "_");
    if clean.trim().is_empty() {
        "shape".to_owned()
    } else {
        clean
    }
}

fn identity_location() -> OcpLocation {
    ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
}

fn send_prefixed_json(host: &str, port: u16, json: &str) -> Result<(), OcpVscodeError> {
    let mut stream = connect(host, port)?;
    websocket_handshake(&mut stream, host, port)?;
    let message = format!("D:{json}");
    send_websocket_text(&mut stream, message.as_bytes())?;
    Ok(())
}

fn connect(host: &str, port: u16) -> Result<TcpStream, OcpVscodeError> {
    let addr = (host, port)
        .to_socket_addrs()
        .map_err(|source| OcpVscodeError::Connect {
            host: host.to_owned(),
            port,
            source,
        })?
        .next()
        .ok_or_else(|| OcpVscodeError::Connect {
            host: host.to_owned(),
            port,
            source: std::io::Error::new(std::io::ErrorKind::NotFound, "no socket address"),
        })?;

    let stream = TcpStream::connect_timeout(&addr, Duration::from_secs(1)).map_err(|source| {
        OcpVscodeError::Connect {
            host: host.to_owned(),
            port,
            source,
        }
    })?;
    stream
        .set_read_timeout(Some(Duration::from_secs(1)))
        .map_err(OcpVscodeError::Send)?;
    stream
        .set_write_timeout(Some(Duration::from_secs(1)))
        .map_err(OcpVscodeError::Send)?;
    Ok(stream)
}

fn websocket_handshake(
    stream: &mut TcpStream,
    host: &str,
    port: u16,
) -> Result<(), OcpVscodeError> {
    let request = format!(
        "GET / HTTP/1.1\r\n\
         Host: {host}:{port}\r\n\
         Upgrade: websocket\r\n\
         Connection: Upgrade\r\n\
         Sec-WebSocket-Key: AAAAAAAAAAAAAAAAAAAAAA==\r\n\
         Sec-WebSocket-Version: 13\r\n\
         \r\n"
    );
    stream.write_all(request.as_bytes())?;

    let mut response = Vec::new();
    let mut buffer = [0; 512];
    loop {
        let read = stream.read(&mut buffer)?;
        if read == 0 {
            break;
        }
        response.extend_from_slice(&buffer[..read]);
        if response.windows(4).any(|window| window == b"\r\n\r\n") {
            break;
        }
    }

    let response_text = String::from_utf8_lossy(&response);
    if response_text.starts_with("HTTP/1.1 101") || response_text.starts_with("HTTP/1.0 101") {
        Ok(())
    } else {
        Err(OcpVscodeError::Handshake(response_text.into_owned()))
    }
}

fn send_websocket_text(stream: &mut TcpStream, payload: &[u8]) -> Result<(), std::io::Error> {
    let mut frame = Vec::with_capacity(payload.len() + 14);
    frame.push(0x81);

    let mask = [0x12, 0x34, 0x56, 0x78];
    match payload.len() {
        0..=125 => frame.push(0x80 | payload.len() as u8),
        126..=65_535 => {
            frame.push(0x80 | 126);
            frame.extend_from_slice(&(payload.len() as u16).to_be_bytes());
        }
        _ => {
            frame.push(0x80 | 127);
            frame.extend_from_slice(&(payload.len() as u64).to_be_bytes());
        }
    }

    frame.extend_from_slice(&mask);
    for (i, byte) in payload.iter().enumerate() {
        frame.push(byte ^ mask[i % mask.len()]);
    }

    stream.write_all(&frame)?;
    stream.flush()
}
