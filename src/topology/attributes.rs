use crate::geometry::curves::Curve;
use crate::geometry::surfaces::Surface;
use crate::geometry::utils::Point3;

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

#[derive(Clone)]
pub struct FaceAttr<T> {
    pub surface: Surface,
    pub data: T,
}
pub type SFaceAttr = FaceAttr<()>;
