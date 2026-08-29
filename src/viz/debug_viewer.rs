//! Send live NGK topology and geometry values to the dedicated debug viewer.

use std::env;
use std::io::{Read, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::time::Duration;

use nalgebra::{UnitVector3, Vector3};
use serde::Serialize;
use thiserror::Error;

use crate::geometry::{
    Bounded, Circle, Curve, Cylinder, Line, NurbsCurve, NurbsSurface, Plane, Point3, RuledSurface,
    Surface, SurfaceOfRevolution,
};
use crate::topology::edge::Edge;
use crate::topology::face::Face;
use crate::topology::gmap::{Dart, GMap, MergeTopology};
use crate::topology::payload::StandardPayload;
use crate::topology::profile::Profile;
use crate::topology::shape::{EdgeTag, FaceTag, ProfileTag, Shape, SheetTag, SolidTag, VertexTag};
use crate::topology::sheet::Sheet;
use crate::topology::solid::Solid;
use crate::topology::vertex::Vertex;

const DEFAULT_HOST: &str = "127.0.0.1";
const DEFAULT_PORT: u16 = 3941;
const DEFAULT_ENDPOINT: &str = "/__ngk_debug/dumps";

#[derive(Debug, Error)]
pub enum DebugViewerError {
    #[error("failed to serialize debug viewer object: {0}")]
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
    #[error("failed to send debug viewer object: {0}")]
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

/// Transport envelope understood by the browser debug viewer.
///
/// Topology entries contain a complete serialized standard-payload map.
/// Geometry entries contain the serde representation of the real kernel value.
/// The browser hydrates both through the NGK WASM bindings.
#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct DebugViewerPayload {
    pub kind: String,
    pub name: String,
    pub objects: Vec<SerializedDebugObject>,
}

/// One debug value and the information required to restore its real WASM
/// object in the browser.
#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct SerializedDebugObject {
    pub kind: DebugObjectKind,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub primary_dart: Option<u32>,
    pub serialized: String,
}

/// The concrete JavaScript class to resolve after deserialization.
#[derive(Debug, Clone, Copy, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum DebugObjectKind {
    GMap,
    Vertex,
    Edge,
    Profile,
    Face,
    Sheet,
    Solid,
    Point,
    Vector,
    Plane,
    Curve,
    Surface,
}

/// Values that can be transferred to the debug viewer as real NGK objects.
///
/// Topology transfer deliberately targets StandardPayload. Arbitrary custom
/// Rust payload types cannot be reconstructed by the browser's statically
/// compiled WASM module.
pub trait DebugDisplay {
    /// Appends serialized objects for browser-side hydration.
    fn append_debug_objects(
        &self,
        objects: &mut Vec<SerializedDebugObject>,
    ) -> Result<(), serde_json::Error>;
}

impl<T: DebugDisplay + ?Sized> DebugDisplay for &T {
    fn append_debug_objects(
        &self,
        objects: &mut Vec<SerializedDebugObject>,
    ) -> Result<(), serde_json::Error> {
        (*self).append_debug_objects(objects)
    }
}

impl<T: DebugDisplay> DebugDisplay for Vec<T> {
    fn append_debug_objects(
        &self,
        objects: &mut Vec<SerializedDebugObject>,
    ) -> Result<(), serde_json::Error> {
        self.as_slice().append_debug_objects(objects)
    }
}

impl<T: DebugDisplay> DebugDisplay for [T] {
    fn append_debug_objects(
        &self,
        objects: &mut Vec<SerializedDebugObject>,
    ) -> Result<(), serde_json::Error> {
        for item in self {
            item.append_debug_objects(objects)?;
        }
        Ok(())
    }
}

impl<T: DebugDisplay, const N: usize> DebugDisplay for [T; N] {
    fn append_debug_objects(
        &self,
        objects: &mut Vec<SerializedDebugObject>,
    ) -> Result<(), serde_json::Error> {
        self.as_slice().append_debug_objects(objects)
    }
}

impl DebugDisplay for GMap<StandardPayload> {
    fn append_debug_objects(
        &self,
        objects: &mut Vec<SerializedDebugObject>,
    ) -> Result<(), serde_json::Error> {
        objects.push(serialize_topology(self, DebugObjectKind::GMap, None)?);
        Ok(())
    }
}

macro_rules! impl_owned_shape_display {
    ($tag:ty, $kind:expr, $view:ident, $dart:expr) => {
        impl DebugDisplay for Shape<$tag, StandardPayload> {
            fn append_debug_objects(
                &self,
                objects: &mut Vec<SerializedDebugObject>,
            ) -> Result<(), serde_json::Error> {
                let view = self.$view();
                objects.push(serialize_topology(self.map(), $kind, Some($dart(&view)))?);
                Ok(())
            }
        }
    };
}

impl_owned_shape_display!(
    VertexTag,
    DebugObjectKind::Vertex,
    vertex,
    |view: &Vertex<'_, StandardPayload>| view.dart
);
impl_owned_shape_display!(EdgeTag, DebugObjectKind::Edge, edge, |view: &Edge<
    '_,
    StandardPayload,
>| view.dart());
impl_owned_shape_display!(
    ProfileTag,
    DebugObjectKind::Profile,
    profile,
    |view: &Profile<'_, StandardPayload>| view.dart
);
impl_owned_shape_display!(FaceTag, DebugObjectKind::Face, face, |view: &Face<
    '_,
    StandardPayload,
>| view.dart());
impl_owned_shape_display!(SheetTag, DebugObjectKind::Sheet, sheet, |view: &Sheet<
    '_,
    StandardPayload,
>| view.dart);
impl_owned_shape_display!(SolidTag, DebugObjectKind::Solid, solid, |view: &Solid<
    '_,
    StandardPayload,
>| view.dart());

macro_rules! impl_view_display {
    ($view:ty, $kind:expr) => {
        impl DebugDisplay for $view {
            fn append_debug_objects(
                &self,
                objects: &mut Vec<SerializedDebugObject>,
            ) -> Result<(), serde_json::Error> {
                append_isolated_topology(self, $kind, objects)
            }
        }
    };
}

impl_view_display!(Vertex<'_, StandardPayload>, DebugObjectKind::Vertex);
impl_view_display!(Edge<'_, StandardPayload>, DebugObjectKind::Edge);
impl_view_display!(Profile<'_, StandardPayload>, DebugObjectKind::Profile);
impl_view_display!(Face<'_, StandardPayload>, DebugObjectKind::Face);
impl_view_display!(Sheet<'_, StandardPayload>, DebugObjectKind::Sheet);
impl_view_display!(Solid<'_, StandardPayload>, DebugObjectKind::Solid);

macro_rules! impl_geometry_display {
    ($value:ty, $kind:expr) => {
        impl DebugDisplay for $value {
            fn append_debug_objects(
                &self,
                objects: &mut Vec<SerializedDebugObject>,
            ) -> Result<(), serde_json::Error> {
                objects.push(serialize_geometry(self, $kind)?);
                Ok(())
            }
        }
    };
}

impl_geometry_display!(Point3, DebugObjectKind::Point);
impl_geometry_display!(Vector3<f64>, DebugObjectKind::Vector);
impl_geometry_display!(Plane, DebugObjectKind::Plane);
impl_geometry_display!(Curve, DebugObjectKind::Curve);
impl_geometry_display!(Surface, DebugObjectKind::Surface);

impl DebugDisplay for UnitVector3<f64> {
    fn append_debug_objects(
        &self,
        objects: &mut Vec<SerializedDebugObject>,
    ) -> Result<(), serde_json::Error> {
        objects.push(serialize_geometry(
            &(*self).into_inner(),
            DebugObjectKind::Vector,
        )?);
        Ok(())
    }
}

macro_rules! impl_curve_display {
    ($value:ty, $variant:ident) => {
        impl DebugDisplay for $value {
            fn append_debug_objects(
                &self,
                objects: &mut Vec<SerializedDebugObject>,
            ) -> Result<(), serde_json::Error> {
                let curve = Curve::$variant(self.clone());
                objects.push(serialize_geometry(&curve, DebugObjectKind::Curve)?);
                Ok(())
            }
        }
    };
}

impl_curve_display!(Line, Line);
impl_curve_display!(Circle, Circle);
impl_curve_display!(NurbsCurve, Nurbs);

impl DebugDisplay for Bounded<Curve> {
    fn append_debug_objects(
        &self,
        objects: &mut Vec<SerializedDebugObject>,
    ) -> Result<(), serde_json::Error> {
        let curve = Curve::Bounded(Box::new(self.clone()));
        objects.push(serialize_geometry(&curve, DebugObjectKind::Curve)?);
        Ok(())
    }
}

macro_rules! impl_surface_display {
    ($value:ty, $variant:ident) => {
        impl DebugDisplay for $value {
            fn append_debug_objects(
                &self,
                objects: &mut Vec<SerializedDebugObject>,
            ) -> Result<(), serde_json::Error> {
                let surface = Surface::$variant(self.clone());
                objects.push(serialize_geometry(&surface, DebugObjectKind::Surface)?);
                Ok(())
            }
        }
    };
}

impl_surface_display!(Cylinder, Cylinder);
impl_surface_display!(RuledSurface, Ruled);
impl_surface_display!(SurfaceOfRevolution, Revolution);
impl_surface_display!(NurbsSurface, Nurbs);

/// Sends an object to the debug viewer using the default connection options.
pub fn show<T: DebugDisplay + ?Sized>(display: &T) -> Result<(), DebugViewerError> {
    show_with_options(display, &DebugViewerOptions::default())
}

/// Sends an object to the debug viewer using explicit connection options.
pub fn show_with_options<T: DebugDisplay + ?Sized>(
    display: &T,
    options: &DebugViewerOptions,
) -> Result<(), DebugViewerError> {
    let payload = payload_for_display(display, options)?;
    send_payload(&payload, options)
}

/// Sends a complete standard-payload map to the debug viewer.
pub fn show_gmap(gmap: &GMap<StandardPayload>) -> Result<(), DebugViewerError> {
    show_gmap_with_options(gmap, &DebugViewerOptions::default())
}

/// Sends a complete standard-payload map with explicit connection options.
pub fn show_gmap_with_options(
    gmap: &GMap<StandardPayload>,
    options: &DebugViewerOptions,
) -> Result<(), DebugViewerError> {
    show_with_options(gmap, options)
}

/// Builds the serialized object envelope without sending it.
pub fn payload_for_display<T: DebugDisplay + ?Sized>(
    display: &T,
    options: &DebugViewerOptions,
) -> Result<DebugViewerPayload, DebugViewerError> {
    let mut objects = Vec::new();
    display.append_debug_objects(&mut objects)?;
    Ok(DebugViewerPayload {
        kind: "ngk.debug.v3".to_owned(),
        name: clean_name(&options.name),
        objects,
    })
}

/// Builds the serialized object envelope for a complete map without sending it.
pub fn payload_for_gmap(
    gmap: &GMap<StandardPayload>,
    options: &DebugViewerOptions,
) -> Result<DebugViewerPayload, DebugViewerError> {
    payload_for_display(gmap, options)
}

/// Sends an already-built debug viewer payload.
pub fn send_payload(
    payload: &DebugViewerPayload,
    options: &DebugViewerOptions,
) -> Result<(), DebugViewerError> {
    let json = serde_json::to_string(payload)?;
    post_json(options, &json)
}

fn append_isolated_topology<T>(
    topology: T,
    kind: DebugObjectKind,
    objects: &mut Vec<SerializedDebugObject>,
) -> Result<(), serde_json::Error>
where
    T: MergeTopology<StandardPayload>,
{
    let (gmap, primary_dart) = GMap::isolate(topology);
    objects.push(serialize_topology(&gmap, kind, Some(primary_dart))?);
    Ok(())
}

fn serialize_topology(
    gmap: &GMap<StandardPayload>,
    kind: DebugObjectKind,
    primary_dart: Option<Dart>,
) -> Result<SerializedDebugObject, serde_json::Error> {
    Ok(SerializedDebugObject {
        kind,
        primary_dart: primary_dart.map(|dart| dart.id() as u32),
        serialized: serde_json::to_string(gmap)?,
    })
}

fn serialize_geometry(
    value: &impl Serialize,
    kind: DebugObjectKind,
) -> Result<SerializedDebugObject, serde_json::Error> {
    Ok(SerializedDebugObject {
        kind,
        primary_dart: None,
        serialized: serde_json::to_string(value)?,
    })
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
