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
    /// Stable key reserved for future loop attributes.
    pub struct LoopKey;
}
new_key_type! {
    /// Stable key for shared geometric data attached to a 2-cell.
    pub struct FacetKey;
}
new_key_type! {
    /// Stable key for an oriented trimmed face occurrence.
    pub struct FaceKey;
}
new_key_type! {
    /// Stable key reserved for future shell attributes.
    pub struct ShellKey;
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
