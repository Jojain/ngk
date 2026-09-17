//! Send live NGK topology and geometry values to the dedicated debug viewer.

use std::env;
use std::io::{Read, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::time::Duration;

use nalgebra::{UnitVector3, Vector3};
use serde::Serialize;
use thiserror::Error;

use crate::geometry::{
    Circle, Cone, Curve, Cylinder, Line, NurbsCurve, NurbsSurface, Plane, Point3, RuledSurface,
    Sphere, Surface, SurfaceOfRevolution, Torus,
};
use crate::model::{MergeTopology, Model};
use crate::topology::edge::Edge;
use crate::topology::face::Face;
use crate::topology::gmap::Dart;
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
/// Topology entries contain a complete serialized standard-payload model.
/// Geometry entries contain the serde representation of the real kernel value.
/// The browser hydrates both through the NGK WASM bindings.
#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct DebugViewerPayload {
    pub kind: String,
    pub name: String,
    pub nodes: Vec<DebugNode>,
}

/// One entry of the viewer's object tree.
///
/// A leaf carries the single value it transports; a group carries children
/// instead, and the viewer shows or hides a whole group at once. Which one a
/// node is, is said by whether `object` is present.
#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct DebugNode {
    pub name: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub object: Option<SerializedDebugObject>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub children: Vec<DebugNode>,
}

impl DebugNode {
    /// A node holding one transported value.
    pub fn leaf(name: impl Into<String>, object: SerializedDebugObject) -> Self {
        Self {
            name: name.into(),
            object: Some(object),
            children: Vec::new(),
        }
    }

    /// A node the viewer toggles as a whole.
    pub fn group(name: impl Into<String>, children: Vec<DebugNode>) -> Self {
        Self {
            name: name.into(),
            object: None,
            children,
        }
    }
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
    Model,
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

impl DebugDisplay for Model<StandardPayload> {
    fn append_debug_objects(
        &self,
        objects: &mut Vec<SerializedDebugObject>,
    ) -> Result<(), serde_json::Error> {
        objects.push(serialize_topology(self, DebugObjectKind::Model, None)?);
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
                objects.push(serialize_topology(self.model(), $kind, $dart(&view))?);
                Ok(())
            }
        }
    };
}

impl_owned_shape_display!(
    VertexTag,
    DebugObjectKind::Vertex,
    vertex,
    |view: &Vertex<'_, StandardPayload>| Some(view.dart)
);
impl_owned_shape_display!(EdgeTag, DebugObjectKind::Edge, edge, |view: &Edge<
    '_,
    StandardPayload,
>| Some(view.dart()));
impl_owned_shape_display!(
    ProfileTag,
    DebugObjectKind::Profile,
    profile,
    |view: &Profile<'_, StandardPayload>| Some(view.dart)
);
impl_owned_shape_display!(FaceTag, DebugObjectKind::Face, face, |view: &Face<
    '_,
    StandardPayload,
>| Some(view.dart()));
impl_owned_shape_display!(SheetTag, DebugObjectKind::Sheet, sheet, |view: &Sheet<
    '_,
    StandardPayload,
>| Some(
    view.dart()
));
impl_owned_shape_display!(SolidTag, DebugObjectKind::Solid, solid, |view: &Solid<
    '_,
    StandardPayload,
>| Some(
    view.dart()
));

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
impl_surface_display!(Sphere, Sphere);
impl_surface_display!(Cone, Cone);
impl_surface_display!(Torus, Torus);
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

/// Sends a complete standard-payload model to the debug viewer.
pub fn show_model(model: &Model<StandardPayload>) -> Result<(), DebugViewerError> {
    show_model_with_options(model, &DebugViewerOptions::default())
}

/// Sends a complete standard-payload model with explicit connection options.
pub fn show_model_with_options(
    model: &Model<StandardPayload>,
    options: &DebugViewerOptions,
) -> Result<(), DebugViewerError> {
    show_with_options(model, options)
}

/// Builds the serialized object envelope without sending it.
pub fn payload_for_display<T: DebugDisplay + ?Sized>(
    display: &T,
    options: &DebugViewerOptions,
) -> Result<DebugViewerPayload, DebugViewerError> {
    let mut objects = Vec::new();
    display.append_debug_objects(&mut objects)?;
    let name = clean_name(&options.name);
    Ok(DebugViewerPayload {
        kind: "ngk.debug.v4".to_owned(),
        nodes: nodes_for_objects(&name, objects),
        name,
    })
}

/// Names the transported values so the viewer's tree can address them.
///
/// One value is one leaf under the caller's name. Several — a slice of faces,
/// a `Vec` of edges — become one group carrying that name, so the viewer hides
/// them together while still reaching each on its own.
fn nodes_for_objects(name: &str, objects: Vec<SerializedDebugObject>) -> Vec<DebugNode> {
    if let [_] = objects.as_slice() {
        return objects
            .into_iter()
            .map(|object| DebugNode::leaf(name, object))
            .collect();
    }
    let children = objects
        .into_iter()
        .enumerate()
        .map(|(index, object)| DebugNode::leaf(format!("{name}[{index}]"), object))
        .collect();
    vec![DebugNode::group(name, children)]
}

/// Builds the serialized object envelope for a complete model without sending it.
pub fn payload_for_model(
    model: &Model<StandardPayload>,
    options: &DebugViewerOptions,
) -> Result<DebugViewerPayload, DebugViewerError> {
    payload_for_display(model, options)
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
    let (model, handle) = Model::isolate(topology);
    objects.push(serialize_topology(&model, kind, Some(handle))?);
    Ok(())
}

fn serialize_topology(
    model: &Model<StandardPayload>,
    kind: DebugObjectKind,
    primary_dart: Option<Dart>,
) -> Result<SerializedDebugObject, serde_json::Error> {
    Ok(SerializedDebugObject {
        kind,
        primary_dart: primary_dart.map(|dart| dart.id() as u32),
        serialized: serde_json::to_string(model)?,
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

#[cfg(test)]
mod tests {
    use super::*;

    fn payload(display: &(impl DebugDisplay + ?Sized)) -> DebugViewerPayload {
        payload_for_display(
            display,
            &DebugViewerOptions {
                name: "corners".to_owned(),
                ..DebugViewerOptions::default()
            },
        )
        .expect("serializing a point cannot fail")
    }

    #[test]
    fn one_value_is_one_named_leaf() {
        let sent = payload(&Point3::new(1.0, 2.0, 3.0));

        let [node] = sent.nodes.as_slice() else {
            panic!("expected one node, got {}", sent.nodes.len());
        };
        assert_eq!(node.name, "corners");
        assert!(node.object.is_some());
        assert!(node.children.is_empty());
    }

    #[test]
    fn several_values_share_one_group_the_viewer_can_hide_at_once() {
        let sent = payload(&vec![Point3::origin(), Point3::new(1.0, 0.0, 0.0)]);

        let [node] = sent.nodes.as_slice() else {
            panic!("expected one node, got {}", sent.nodes.len());
        };
        assert_eq!(node.name, "corners");
        assert!(node.object.is_none());
        assert_eq!(
            node.children
                .iter()
                .map(|child| child.name.as_str())
                .collect::<Vec<_>>(),
            ["corners[0]", "corners[1]"]
        );
        assert!(node.children.iter().all(|child| child.object.is_some()));
    }
}
