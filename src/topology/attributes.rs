use crate::geometry::curves::Curve;
use crate::geometry::surfaces::Surface;
use crate::geometry::utils::Point3;
use crate::topology::dart::Dart;
use crate::topology::edge::Edge;
use crate::topology::face::Face;
use crate::topology::gmap::GMap;
use crate::topology::payload::Payload;
use crate::topology::vertex::Vertex;

#[derive(Clone)]

pub struct VertexAttr<T> {
    pub dart: Dart,
    pub point: Point3,
    pub data: T,
}

impl<T> VertexAttr<T> {
    pub fn new(dart: Dart, point: Point3, data: T) -> Self {
        Self { dart, point, data }
    }

    pub fn vertex<'a, P: Payload>(&self, gmap: &'a GMap<P>) -> Vertex<'a, P> {
        Vertex::new(gmap, self.dart)
    }
}

#[derive(Clone)]

pub struct EdgeAttr<T> {
    pub dart: Dart,
    pub curve: Curve,
    pub data: T,
}

impl<T> EdgeAttr<T> {
    pub fn new(dart: Dart, curve: Curve, data: T) -> Self {
        Self { dart, curve, data }
    }

    pub fn edge<'a, P: Payload>(&self, gmap: &'a GMap<P>) -> Edge<'a, P> {
        Edge::new(gmap, self.dart)
    }
}

#[derive(Clone)]
pub struct FaceAttr<T> {
    pub surface: Surface,
    pub data: T,
    pub outer_loop: Dart,
    pub inner_loops: Vec<Dart>,
}

impl<T> FaceAttr<T> {
    pub fn new(surface: Surface, data: T, outer_loop: Dart, inner_loops: Vec<Dart>) -> Self {
        Self {
            surface,
            data,
            outer_loop,
            inner_loops,
        }
    }
    pub fn face<'a, P: Payload<F = T>>(&'a self, gmap: &'a GMap<P>) -> Face<'a, P> {
        Face::new(gmap, self)
    }
}

#[derive(Clone)]
pub struct SolidAttr<T> {
    pub data: T,
    pub outer_shell: Dart,
    pub inner_shells: Option<Vec<Dart>>,
}

impl<T> SolidAttr<T> {
    pub fn new(data: T, outer_shell: Dart, inner_shells: Option<Vec<Dart>>) -> Self {
        Self {
            data,
            outer_shell,
            inner_shells,
        }
    }
}
