use crate::geometry::curves::Curve;
use crate::geometry::surfaces::Surface;
use crate::geometry::utils::Point3;
use crate::topology::dart::Dart;

#[derive(Clone)]

pub struct VertexAttr<T> {
    pub point: Point3,
    pub data: T,
}
pub type SVertexAttr = VertexAttr<()>;

#[derive(Clone)]

pub struct EdgeAttr<T> {
    pub curve: Curve,
    pub data: T,
}

pub type SEdgeAttr = EdgeAttr<()>;

/// Domain face payload: surface, user data, and representative darts for the
/// outer boundary loop and inner hole loops (same as stored in [`crate::topology::gmap::GMap`]).
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
}

pub type SFaceAttr = FaceAttr<()>;

/// Domain solid payload: user data and representative darts for outer and
/// optional inner closed shells.
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
