#[derive(Debug, Clone, PartialEq)]
pub enum NurbsError {
    DegreeZero,
    UnsortedKnots,
    EmptyKnotVector,
    EmptyControlPolygon,
    KnotCountMismatch { expected: usize, got: usize },
    ParameterOutOfRange { u: f64, min: f64, max: f64 },
    WeightCountMismatch { expected: usize, got: usize },
    ControlNetDimensionMismatch { expected: usize, got: usize },
}

impl std::fmt::Display for NurbsError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::DegreeZero => write!(f, "NURBS degree must be >= 1"),
            Self::UnsortedKnots => write!(f, "knot vector must be non-decreasing"),
            Self::EmptyKnotVector => write!(f, "knot vector is empty"),
            Self::EmptyControlPolygon => write!(f, "control polygon is empty"),
            Self::KnotCountMismatch { expected, got } => write!(
                f,
                "knot count mismatch: expected {} (= n + p + 1), got {}",
                expected, got
            ),
            Self::ParameterOutOfRange { u, min, max } => {
                write!(f, "parameter {} is outside domain [{}, {}]", u, min, max)
            }
            Self::WeightCountMismatch { expected, got } => write!(
                f,
                "weight count mismatch: expected {}, got {}",
                expected, got
            ),
            Self::ControlNetDimensionMismatch { expected, got } => write!(
                f,
                "control net dimension mismatch: expected {} entries, got {}",
                expected, got
            ),
        }
    }
}

impl std::error::Error for NurbsError {}
