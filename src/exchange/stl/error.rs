use crate::tessellate::TessellationError;

/// Why a surface could not be written as binary STL.
#[derive(Debug, thiserror::Error)]
pub enum StlError {
    #[error(transparent)]
    Tessellation(#[from] TessellationError),
    #[error("STL sampling requires at least four curve segments and positive surface subdivisions")]
    InvalidSampling,
    #[error("STL requires a nonempty list of complete triangles")]
    InvalidTriangles,
    #[error("STL can contain at most u32::MAX triangles")]
    TooManyTriangles,
    #[error("triangle {triangle} references a missing position")]
    MissingPosition { triangle: usize },
    #[error("triangle {triangle} contains a coordinate outside finite 32-bit range")]
    InvalidCoordinate { triangle: usize },
    #[error("triangle {triangle} collapses at STL's 32-bit precision")]
    DegenerateTriangle { triangle: usize },
    #[error("STL triangles do not form a closed, consistently oriented surface")]
    NotClosed,
    #[error("STL output failed: {0}")]
    Io(#[from] std::io::Error),
}
