use slotmap::new_key_type;
new_key_type! {pub struct VertexKey;}
new_key_type! {pub struct EdgeKey;}
new_key_type! {pub struct LoopKey;}
new_key_type! {pub struct FaceKey;}
new_key_type! {pub struct ShellKey;}
new_key_type! {pub struct SolidKey;}

pub enum ShapeKey {
    Vertex(VertexKey),
    Edge(EdgeKey),
    Face(FaceKey),
}
