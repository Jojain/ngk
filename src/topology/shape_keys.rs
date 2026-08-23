use slotmap::new_key_type;

new_key_type! {
    /// Stable key for a stored vertex attribute.
    pub struct VertexKey;
}
new_key_type! {
    /// Stable key for a stored edge attribute.
    pub struct EdgeKey;
}
new_key_type! {
    /// Stable key for a stored profile identity and default orientation.
    pub struct ProfileKey;
}
new_key_type! {
    /// Stable key for a stored face attribute.
    pub struct FaceKey;
}
new_key_type! {
    /// Stable key for a stored sheet identity and default orientation.
    pub struct SheetKey;
}
new_key_type! {
    /// Stable key for a stored solid attribute.
    pub struct SolidKey;
}

/// Dimension-erased key for keyed domain topology.
pub enum ShapeKey {
    /// Vertex key variant.
    Vertex(VertexKey),
    /// Edge key variant.
    Edge(EdgeKey),
    /// Face key variant.
    Face(FaceKey),
}
