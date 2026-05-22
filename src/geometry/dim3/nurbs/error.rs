use thiserror::Error;

#[derive(Debug, Clone, Error, PartialEq)]
pub enum NurbsError {
    #[error("NURBS degree must be >= 1")]
    DegreeZero,
    #[error("knot vector must be non-decreasing")]
    UnsortedKnots,
    #[error("knot vector is empty")]
    EmptyKnotVector,
    #[error("control polygon is empty")]
    EmptyControlPolygon,
    #[error("knot count mismatch: expected {expected} (= n + p + 1), got {got}")]
    KnotCountMismatch { expected: usize, got: usize },
    #[error("parameter {u} is outside domain [{min}, {max}]")]
    ParameterOutOfRange { u: f64, min: f64, max: f64 },
    #[error("weight count mismatch: expected {expected}, got {got}")]
    WeightCountMismatch { expected: usize, got: usize },
    #[error("control net dimension mismatch: expected {expected} entries, got {got}")]
    ControlNetDimensionMismatch { expected: usize, got: usize },
}
