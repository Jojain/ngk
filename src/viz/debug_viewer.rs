//! Send rich NGK topology dumps to the dedicated debug viewer.

use std::any::type_name;
use std::collections::HashSet;
use std::env;
use std::fmt::Debug;
use std::io::{Read, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::time::Duration;

use serde::Serialize;
use thiserror::Error;

use super::{VizHints, VizScene, scene_from_gmap};
use crate::geometry::dim2::curves::Curve2;
use crate::geometry::{Curve, Surface};
use crate::topology::edge::Edge;
use crate::topology::face::Face;
use crate::topology::facet::Facet;
use crate::topology::gmap::{Cell0, Cell1, Dart, Dim, GMap, MergeTopology};
use crate::topology::payload::Payload;
use crate::topology::profile::Profile;
use crate::topology::shape::{
    EdgeTag, FaceTag, FacetTag, ProfileTag, Shape, SheetTag, SolidTag, VertexTag,
};
use crate::topology::sheet::Sheet;
use crate::topology::solid::Solid;
use crate::topology::vertex::Vertex;

const DEFAULT_HOST: &str = "127.0.0.1";
const DEFAULT_PORT: u16 = 3941;
const DEFAULT_ENDPOINT: &str = "/__ngk_debug/dumps";

#[derive(Debug, Error)]
pub enum DebugViewerError {
    #[error("failed to serialize debug viewer payload: {0}")]
    Serialize(#[from] serde_json::Error),
    #[error("failed to connect to debug viewer on {host}:{port}: {source}")]
    Connect {
        host: String,
        port: u16,
        #[source]
        source: std::io::Error,
    },
    #[error("debug viewer rejected the POST with response: {0}")]
    Http(String),
    #[error("failed to send debug viewer payload: {0}")]
    Send(#[from] std::io::Error),
}

#[derive(Debug, Clone)]
pub struct DebugViewerOptions {
    pub host: String,
    pub port: u16,
    pub endpoint: String,
    pub name: String,
}

impl Default for DebugViewerOptions {
    fn default() -> Self {
        Self {
            host: DEFAULT_HOST.to_owned(),
            port: env::var("NGK_DEBUG_VIEWER_PORT")
                .ok()
                .and_then(|value| value.parse().ok())
                .unwrap_or(DEFAULT_PORT),
            endpoint: DEFAULT_ENDPOINT.to_owned(),
            name: "shape".to_owned(),
        }
    }
}

#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct DebugViewerPayload {
    pub kind: String,
    pub name: String,
    pub scene: VizScene,
    pub gmap: GMapDebugSnapshot,
    pub selection: SelectionIndex,
    pub metadata: DebugMetadata,
}

#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct GMapDebugSnapshot {
    pub dimension: u32,
    pub dart_count: u32,
    pub alphas: Vec<Vec<u32>>,
    pub darts: Vec<DartMetadata>,
}

#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct DartMetadata {
    pub dart: u32,
    pub vertex: Option<String>,
    pub edge: Option<String>,
    pub face: Option<String>,
    pub solid: Option<String>,
}

#[derive(Debug, Clone, Default, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct SelectionIndex {
    pub vertices: Vec<EntitySelection>,
    pub edges: Vec<EntitySelection>,
    pub faces: Vec<EntitySelection>,
}

#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct EntitySelection {
    pub render_id: u32,
    pub key: String,
    pub representative_dart: u32,
}

#[derive(Debug, Clone, Default, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct DebugMetadata {
    pub vertices: Vec<VertexMetadata>,
    pub edges: Vec<EdgeMetadata>,
    pub faces: Vec<FaceMetadata>,
    pub solids: Vec<SolidMetadata>,
}

#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct VertexMetadata {
    pub key: String,
    pub representative_dart: u32,
    pub darts: Vec<u32>,
    pub point: [f64; 3],
    pub payload: PayloadSummary,
}

#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct EdgeMetadata {
    pub key: String,
    pub representative_dart: u32,
    pub darts: Vec<u32>,
    pub curve: GeometrySummary,
    pub payload: PayloadSummary,
}

#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct FaceMetadata {
    pub key: String,
    pub representative_dart: u32,
    pub darts: Vec<u32>,
    pub outer_loop: Vec<u32>,
    pub inner_loops: Vec<Vec<u32>>,
    pub surface: GeometrySummary,
    pub normals: Vec<NormalSample>,
    pub pcurves: Vec<PcurveMetadata>,
    pub payload: PayloadSummary,
}

#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct NormalSample {
    pub origin: [f64; 3],
    pub direction: [f64; 3],
}

#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct SolidMetadata {
    pub key: String,
    pub representative_dart: u32,
    pub darts: Vec<u32>,
    pub inner_shells: Option<Vec<u32>>,
    pub payload: PayloadSummary,
}

#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct PcurveMetadata {
    pub dart: u32,
    pub edge_key: String,
    pub start_vertex_key: String,
    pub end_vertex_key: String,
    pub curve: GeometrySummary,
    pub samples: Vec<[f64; 2]>,
}

#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct GeometrySummary {
    pub kind: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub details: Option<String>,
}

#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct PayloadSummary {
    pub type_name: String,
    pub debug: String,
}

pub trait DebugDisplay {
    fn append_debug_gmaps(&self, gmaps: &mut Vec<GMapDebugItem>);
}

pub struct GMapDebugItem {
    scene: VizScene,
    snapshot: GMapDebugSnapshot,
    selection: SelectionIndex,
    metadata: DebugMetadata,
}

impl<T: DebugDisplay + ?Sized> DebugDisplay for &T {
    fn append_debug_gmaps(&self, gmaps: &mut Vec<GMapDebugItem>) {
        (*self).append_debug_gmaps(gmaps);
    }
}

impl<T: DebugDisplay> DebugDisplay for Vec<T> {
    fn append_debug_gmaps(&self, gmaps: &mut Vec<GMapDebugItem>) {
        self.as_slice().append_debug_gmaps(gmaps);
    }
}

impl<T: DebugDisplay> DebugDisplay for [T] {
    fn append_debug_gmaps(&self, gmaps: &mut Vec<GMapDebugItem>) {
        for item in self {
            item.append_debug_gmaps(gmaps);
        }
    }
}

impl<T: DebugDisplay, const N: usize> DebugDisplay for [T; N] {
    fn append_debug_gmaps(&self, gmaps: &mut Vec<GMapDebugItem>) {
        self.as_slice().append_debug_gmaps(gmaps);
    }
}

impl<P> DebugDisplay for GMap<P>
where
    P: Payload,
    P::V: Debug,
    P::E: Debug,
    P::F: Debug,
    P::S: Debug,
{
    fn append_debug_gmaps(&self, gmaps: &mut Vec<GMapDebugItem>) {
        gmaps.push(item_for_gmap(self));
    }
}

impl<P> DebugDisplay for Shape<VertexTag, P>
where
    P: Payload,
    P::V: Debug,
    P::E: Debug,
    P::F: Debug,
    P::S: Debug,
{
    fn append_debug_gmaps(&self, gmaps: &mut Vec<GMapDebugItem>) {
        self.map().append_debug_gmaps(gmaps);
    }
}

impl<P> DebugDisplay for Shape<EdgeTag, P>
where
    P: Payload,
    P::V: Debug,
    P::E: Debug,
    P::F: Debug,
    P::S: Debug,
{
    fn append_debug_gmaps(&self, gmaps: &mut Vec<GMapDebugItem>) {
        self.map().append_debug_gmaps(gmaps);
    }
}

impl<P> DebugDisplay for Shape<ProfileTag, P>
where
    P: Payload,
    P::V: Debug,
    P::E: Debug,
    P::F: Debug,
    P::S: Debug,
{
    fn append_debug_gmaps(&self, gmaps: &mut Vec<GMapDebugItem>) {
        self.map().append_debug_gmaps(gmaps);
    }
}

impl<P> DebugDisplay for Shape<FaceTag, P>
where
    P: Payload,
    P::V: Debug,
    P::E: Debug,
    P::F: Debug,
    P::S: Debug,
{
    fn append_debug_gmaps(&self, gmaps: &mut Vec<GMapDebugItem>) {
        self.map().append_debug_gmaps(gmaps);
    }
}

impl<P> DebugDisplay for Shape<FacetTag, P>
where
    P: Payload,
    P::V: Debug,
    P::E: Debug,
    P::F: Debug,
    P::S: Debug,
{
    fn append_debug_gmaps(&self, gmaps: &mut Vec<GMapDebugItem>) {
        self.map().append_debug_gmaps(gmaps);
    }
}

impl<P> DebugDisplay for Shape<SheetTag, P>
where
    P: Payload,
    P::V: Debug,
    P::E: Debug,
    P::F: Debug,
    P::S: Debug,
{
    fn append_debug_gmaps(&self, gmaps: &mut Vec<GMapDebugItem>) {
        self.map().append_debug_gmaps(gmaps);
    }
}

impl<P> DebugDisplay for Shape<SolidTag, P>
where
    P: Payload,
    P::V: Debug,
    P::E: Debug,
    P::F: Debug,
    P::S: Debug,
{
    fn append_debug_gmaps(&self, gmaps: &mut Vec<GMapDebugItem>) {
        self.map().append_debug_gmaps(gmaps);
    }
}

impl<P> DebugDisplay for Vertex<'_, P>
where
    P: Payload,
    P::V: Debug,
    P::E: Debug,
    P::F: Debug,
    P::S: Debug,
{
    fn append_debug_gmaps(&self, gmaps: &mut Vec<GMapDebugItem>) {
        append_topology_item::<P, _>(self, gmaps);
    }
}

impl<P> DebugDisplay for Edge<'_, P>
where
    P: Payload,
    P::V: Debug,
    P::E: Debug,
    P::F: Debug,
    P::S: Debug,
{
    fn append_debug_gmaps(&self, gmaps: &mut Vec<GMapDebugItem>) {
        append_topology_item::<P, _>(self, gmaps);
    }
}

impl<P> DebugDisplay for Profile<'_, P>
where
    P: Payload,
    P::V: Debug,
    P::E: Debug,
    P::F: Debug,
    P::S: Debug,
{
    fn append_debug_gmaps(&self, gmaps: &mut Vec<GMapDebugItem>) {
        append_topology_item::<P, _>(self, gmaps);
    }
}

impl<P> DebugDisplay for Face<'_, P>
where
    P: Payload,
    P::V: Debug,
    P::E: Debug,
    P::F: Debug,
    P::S: Debug,
{
    fn append_debug_gmaps(&self, gmaps: &mut Vec<GMapDebugItem>) {
        append_topology_item::<P, _>(self, gmaps);
    }
}

impl<P> DebugDisplay for Facet<'_, P>
where
    P: Payload,
    P::V: Debug,
    P::E: Debug,
    P::F: Debug,
    P::S: Debug,
{
    fn append_debug_gmaps(&self, gmaps: &mut Vec<GMapDebugItem>) {
        append_topology_item::<P, _>(self, gmaps);
    }
}

impl<P> DebugDisplay for Sheet<'_, P>
where
    P: Payload,
    P::V: Debug,
    P::E: Debug,
    P::F: Debug,
    P::S: Debug,
{
    fn append_debug_gmaps(&self, gmaps: &mut Vec<GMapDebugItem>) {
        append_topology_item::<P, _>(self, gmaps);
    }
}

impl<P> DebugDisplay for Solid<'_, P>
where
    P: Payload,
    P::V: Debug,
    P::E: Debug,
    P::F: Debug,
    P::S: Debug,
{
    fn append_debug_gmaps(&self, gmaps: &mut Vec<GMapDebugItem>) {
        append_topology_item::<P, _>(self, gmaps);
    }
}

pub fn show<T: DebugDisplay + ?Sized>(display: &T) -> Result<(), DebugViewerError> {
    show_with_options(display, &DebugViewerOptions::default())
}

pub fn show_with_options<T: DebugDisplay + ?Sized>(
    display: &T,
    options: &DebugViewerOptions,
) -> Result<(), DebugViewerError> {
    let payload = payload_for_display(display, options);
    send_payload(&payload, options)
}

pub fn show_gmap<P>(g: &GMap<P>) -> Result<(), DebugViewerError>
where
    P: Payload,
    P::V: Debug,
    P::E: Debug,
    P::F: Debug,
    P::S: Debug,
{
    show_gmap_with_options(g, &DebugViewerOptions::default())
}

pub fn show_gmap_with_options<P>(
    g: &GMap<P>,
    options: &DebugViewerOptions,
) -> Result<(), DebugViewerError>
where
    P: Payload,
    P::V: Debug,
    P::E: Debug,
    P::F: Debug,
    P::S: Debug,
{
    let payload = payload_for_gmap(g, options);
    send_payload(&payload, options)
}

pub fn payload_for_display<T: DebugDisplay + ?Sized>(
    display: &T,
    options: &DebugViewerOptions,
) -> DebugViewerPayload {
    let mut items = Vec::new();
    display.append_debug_gmaps(&mut items);
    payload_for_items(items, options)
}

pub fn payload_for_gmap<P>(g: &GMap<P>, options: &DebugViewerOptions) -> DebugViewerPayload
where
    P: Payload,
    P::V: Debug,
    P::E: Debug,
    P::F: Debug,
    P::S: Debug,
{
    payload_for_items(vec![item_for_gmap(g)], options)
}

pub fn send_payload(
    payload: &DebugViewerPayload,
    options: &DebugViewerOptions,
) -> Result<(), DebugViewerError> {
    let json = serde_json::to_string(payload)?;
    post_json(options, &json)
}

fn append_topology_item<P, T>(topology: T, gmaps: &mut Vec<GMapDebugItem>)
where
    P: Payload,
    P::V: Debug,
    P::E: Debug,
    P::F: Debug,
    P::S: Debug,
    T: MergeTopology<P>,
{
    let (g, _) = GMap::isolate(topology);
    gmaps.push(item_for_gmap(&g));
}

fn payload_for_items(
    items: Vec<GMapDebugItem>,
    options: &DebugViewerOptions,
) -> DebugViewerPayload {
    if items.len() == 1 {
        let item = items.into_iter().next().expect("one item exists");
        return DebugViewerPayload {
            kind: "ngk.debug.v1".to_owned(),
            name: clean_name(&options.name),
            scene: item.scene,
            gmap: item.snapshot,
            selection: item.selection,
            metadata: item.metadata,
        };
    }

    let mut scene = VizScene::new();
    let mut metadata = DebugMetadata::default();
    for item in items {
        scene.vertices.extend(item.scene.vertices);
        scene.edges.extend(item.scene.edges);
        scene.faces.extend(item.scene.faces);
        scene.darts.extend(item.scene.darts);
        scene.alpha_links.extend(item.scene.alpha_links);
        scene.labels.extend(item.scene.labels);
        metadata.vertices.extend(item.metadata.vertices);
        metadata.edges.extend(item.metadata.edges);
        metadata.faces.extend(item.metadata.faces);
        metadata.solids.extend(item.metadata.solids);
    }

    DebugViewerPayload {
        kind: "ngk.debug.v1".to_owned(),
        name: clean_name(&options.name),
        scene,
        gmap: GMapDebugSnapshot {
            dimension: 0,
            dart_count: 0,
            alphas: Vec::new(),
            darts: Vec::new(),
        },
        selection: SelectionIndex::default(),
        metadata,
    }
}

fn item_for_gmap<P>(g: &GMap<P>) -> GMapDebugItem
where
    P: Payload,
    P::V: Debug,
    P::E: Debug,
    P::F: Debug,
    P::S: Debug,
{
    let scene = scene_from_gmap(g, &VizHints::new());
    let metadata = metadata_for_gmap(g);
    let selection = selection_for_scene(g, &scene, &metadata);
    let snapshot = snapshot_for_gmap(g);
    GMapDebugItem {
        scene,
        snapshot,
        selection,
        metadata,
    }
}

fn snapshot_for_gmap<P: Payload>(g: &GMap<P>) -> GMapDebugSnapshot {
    let dim = g.dimension();
    let n = g.dart_count();
    let mut alphas: Vec<Vec<u32>> = (0..dim).map(|_| Vec::with_capacity(n)).collect();
    for (i, alpha) in alphas.iter_mut().enumerate().take(dim) {
        for id in 0..n {
            let d = Dart::new(id);
            alpha.push(g.alpha(Dim::from_index(i), d).id() as u32);
        }
    }

    let darts = (0..n)
        .map(|id| {
            let d = Dart::new(id);
            DartMetadata {
                dart: id as u32,
                vertex: g
                    .attribute::<Cell0>(d)
                    .map(|_| key_string(g.cell_representative(d, Dim::Zero), "vertex")),
                edge: g
                    .attribute::<Cell1>(d)
                    .map(|_| key_string(g.cell_representative(d, Dim::One), "edge")),
                face: g
                    .attribute::<crate::topology::gmap::Cell2>(d)
                    .map(debug_key),
                solid: g
                    .attribute::<crate::topology::gmap::Cell3>(d)
                    .map(debug_key),
            }
        })
        .collect();

    GMapDebugSnapshot {
        dimension: dim as u32,
        dart_count: n as u32,
        alphas,
        darts,
    }
}

fn metadata_for_gmap<P>(g: &GMap<P>) -> DebugMetadata
where
    P: Payload,
    P::V: Debug,
    P::E: Debug,
    P::F: Debug,
    P::S: Debug,
{
    let vertices = g
        .iter_vertices()
        .map(|(key, attr)| VertexMetadata {
            key: debug_key(&key),
            representative_dart: attr.dart.id() as u32,
            darts: cell_darts(g, attr.dart, Dim::Zero),
            point: [attr.point.x, attr.point.y, attr.point.z],
            payload: payload_summary(&attr.data),
        })
        .collect();

    let edges = g
        .iter_edges()
        .map(|(key, attr)| EdgeMetadata {
            key: debug_key(&key),
            representative_dart: attr.dart.id() as u32,
            darts: cell_darts(g, attr.dart, Dim::One),
            curve: curve_summary(&attr.curve),
            payload: payload_summary(&attr.data),
        })
        .collect();

    let faces = g
        .iter_faces()
        .map(|(key, attr)| {
            let face = attr.face(g);
            let pcurves = face
                .loops()
                .into_iter()
                .flat_map(|loop_| loop_.edges())
                .filter_map(|edge| {
                    let curve = face.pcurve(edge.dart())?;
                    Some(PcurveMetadata {
                        dart: edge.dart().id() as u32,
                        edge_key: debug_key(&edge.key()),
                        start_vertex_key: debug_key(&edge.start().key()),
                        end_vertex_key: debug_key(&edge.end().key()),
                        curve: curve2_summary(&curve),
                        samples: curve.sample(32).iter().map(|p| [p.x, p.y]).collect(),
                    })
                })
                .collect::<Vec<_>>();

            FaceMetadata {
                key: debug_key(&key),
                representative_dart: attr.outer_loop.id() as u32,
                darts: cell_darts(g, attr.outer_loop, Dim::Two),
                outer_loop: loop_darts(g, attr.outer_loop),
                inner_loops: attr
                    .inner_loops
                    .iter()
                    .map(|dart| loop_darts(g, *dart))
                    .collect(),
                surface: surface_summary(&attr.surface),
                normals: normal_samples_for_face(&face, &pcurves),
                pcurves,
                payload: payload_summary(&attr.data),
            }
        })
        .collect();

    let solids = g
        .iter_solids()
        .map(|(key, attr)| SolidMetadata {
            key: debug_key(&key),
            representative_dart: attr.outer_shell.id() as u32,
            darts: cell_darts(g, attr.outer_shell, Dim::Three),
            inner_shells: attr
                .inner_shells
                .as_ref()
                .map(|darts| darts.iter().map(|d| d.id() as u32).collect()),
            payload: payload_summary(&attr.data),
        })
        .collect();

    DebugMetadata {
        vertices,
        edges,
        faces,
        solids,
    }
}

fn selection_for_scene<P: Payload>(
    g: &GMap<P>,
    scene: &VizScene,
    metadata: &DebugMetadata,
) -> SelectionIndex {
    SelectionIndex {
        vertices: scene
            .vertices
            .iter()
            .filter_map(|vertex| {
                let meta = metadata.vertices.get(vertex.vertex_id as usize)?;
                Some(EntitySelection {
                    render_id: vertex.vertex_id,
                    key: meta.key.clone(),
                    representative_dart: g
                        .cell_representative(
                            Dart::new(meta.representative_dart as usize),
                            Dim::Zero,
                        )
                        .id() as u32,
                })
            })
            .collect(),
        edges: scene
            .edges
            .iter()
            .filter_map(|edge| {
                let meta = metadata.edges.get(edge.edge_id as usize)?;
                Some(EntitySelection {
                    render_id: edge.edge_id,
                    key: meta.key.clone(),
                    representative_dart: g
                        .cell_representative(Dart::new(meta.representative_dart as usize), Dim::One)
                        .id() as u32,
                })
            })
            .collect(),
        faces: scene
            .faces
            .iter()
            .filter_map(|face| {
                let meta = metadata.faces.get(face.face_id as usize)?;
                Some(EntitySelection {
                    render_id: face.face_id,
                    key: meta.key.clone(),
                    representative_dart: g
                        .cell_representative(Dart::new(meta.representative_dart as usize), Dim::Two)
                        .id() as u32,
                })
            })
            .collect(),
    }
}

fn cell_darts<P: Payload>(g: &GMap<P>, dart: Dart, dim: Dim) -> Vec<u32> {
    let mut darts = g
        .orbit(dart, g.orbit_indices(dim))
        .map(|d| d.id() as u32)
        .collect::<Vec<_>>();
    darts.sort_unstable();
    darts
}

fn loop_darts<P: Payload>(g: &GMap<P>, dart: Dart) -> Vec<u32> {
    let mut out = Vec::new();
    let mut seen = HashSet::new();
    let mut current = dart;
    while seen.insert(current) {
        out.push(current.id() as u32);
        current = g.alpha(Dim::One, g.alpha(Dim::Zero, current));
    }
    out
}

fn normal_samples_for_face<P: Payload>(
    face: &Face<'_, P>,
    pcurves: &[PcurveMetadata],
) -> Vec<NormalSample> {
    let Some((min_u, max_u, min_v, max_v)) = uv_bounds(pcurves) else {
        return Vec::new();
    };

    const SAMPLES_U: usize = 10;
    const SAMPLES_V: usize = 10;
    let step_u = if SAMPLES_U > 1 {
        (max_u - min_u) / (SAMPLES_U - 1) as f64
    } else {
        0.0
    };
    let step_v = if SAMPLES_V > 1 {
        (max_v - min_v) / (SAMPLES_V - 1) as f64
    } else {
        0.0
    };

    let mut samples = Vec::with_capacity(SAMPLES_U * SAMPLES_V);
    for u_index in 0..SAMPLES_U {
        for v_index in 0..SAMPLES_V {
            let u = min_u + step_u * u_index as f64;
            let v = min_v + step_v * v_index as f64;
            let origin = face.point_at(u, v);
            let direction = face.normal_at(u, v).into_inner();
            samples.push(NormalSample {
                origin: [origin.x, origin.y, origin.z],
                direction: [direction.x, direction.y, direction.z],
            });
        }
    }
    samples
}

fn uv_bounds(pcurves: &[PcurveMetadata]) -> Option<(f64, f64, f64, f64)> {
    let mut points = pcurves.iter().flat_map(|pcurve| pcurve.samples.iter());
    let first = points.next()?;
    let (mut min_u, mut max_u) = (first[0], first[0]);
    let (mut min_v, mut max_v) = (first[1], first[1]);
    for point in points {
        min_u = min_u.min(point[0]);
        max_u = max_u.max(point[0]);
        min_v = min_v.min(point[1]);
        max_v = max_v.max(point[1]);
    }
    Some((min_u, max_u, min_v, max_v))
}

fn curve_summary(curve: &Curve) -> GeometrySummary {
    GeometrySummary {
        kind: match curve {
            Curve::Line(_) => "line",
            Curve::Circle(_) => "circle",
            Curve::Nurbs(_) => "nurbs",
            Curve::Bounded(_) => "bounded",
        }
        .to_owned(),
        details: None,
    }
}

fn curve2_summary(curve: &Curve2) -> GeometrySummary {
    GeometrySummary {
        kind: match curve {
            Curve2::Line(_) => "line",
            Curve2::Nurbs(_) => "nurbs",
        }
        .to_owned(),
        details: Some(format!("{curve:?}")),
    }
}

fn surface_summary(surface: &Surface) -> GeometrySummary {
    GeometrySummary {
        kind: match surface {
            Surface::Plane(_) => "plane",
            Surface::Cylinder(_) => "cylinder",
            Surface::Ruled(_) => "ruled",
            Surface::Revolution(_) => "revolution",
            Surface::Nurbs(_) => "nurbs",
        }
        .to_owned(),
        details: None,
    }
}

fn payload_summary<T: Debug + 'static>(value: &T) -> PayloadSummary {
    PayloadSummary {
        type_name: type_name::<T>().to_owned(),
        debug: format!("{value:#?}"),
    }
}

fn debug_key<T: Debug>(key: &T) -> String {
    format!("{key:?}")
}

fn key_string(dart: Dart, kind: &str) -> String {
    format!("{kind}@d{}", dart.id())
}

fn clean_name(name: &str) -> String {
    let clean = name.replace(['/', '\\'], "_");
    if clean.trim().is_empty() {
        "shape".to_owned()
    } else {
        clean
    }
}

fn post_json(options: &DebugViewerOptions, json: &str) -> Result<(), DebugViewerError> {
    let mut stream = connect(&options.host, options.port)?;
    let request = format!(
        "POST {} HTTP/1.1\r\n\
         Host: {}:{}\r\n\
         Content-Type: application/json\r\n\
         Content-Length: {}\r\n\
         Connection: close\r\n\
         \r\n\
         {}",
        options.endpoint,
        options.host,
        options.port,
        json.len(),
        json
    );
    stream.write_all(request.as_bytes())?;
    stream.flush()?;

    let mut response = String::new();
    stream.read_to_string(&mut response)?;
    if response.starts_with("HTTP/1.1 2") || response.starts_with("HTTP/1.0 2") {
        Ok(())
    } else {
        Err(DebugViewerError::Http(response))
    }
}

fn connect(host: &str, port: u16) -> Result<TcpStream, DebugViewerError> {
    let addr = (host, port)
        .to_socket_addrs()
        .map_err(|source| DebugViewerError::Connect {
            host: host.to_owned(),
            port,
            source,
        })?
        .next()
        .ok_or_else(|| DebugViewerError::Connect {
            host: host.to_owned(),
            port,
            source: std::io::Error::new(std::io::ErrorKind::NotFound, "no socket address"),
        })?;

    let stream = TcpStream::connect_timeout(&addr, Duration::from_secs(1)).map_err(|source| {
        DebugViewerError::Connect {
            host: host.to_owned(),
            port,
            source,
        }
    })?;
    stream
        .set_read_timeout(Some(Duration::from_secs(1)))
        .map_err(DebugViewerError::Send)?;
    stream
        .set_write_timeout(Some(Duration::from_secs(1)))
        .map_err(DebugViewerError::Send)?;
    Ok(stream)
}
