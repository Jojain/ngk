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
    #[error("Bezier control point count mismatch: expected {expected}, got {got}")]
    BezierControlPointCountMismatch { expected: usize, got: usize },
    #[error("parameter interval is degenerate: [{start}, {end}]")]
    DegenerateInterval { start: f64, end: f64 },
    #[error("interpolation requires at least {minimum} points, got {got}")]
    InsufficientInterpolationPoints { minimum: usize, got: usize },
    #[error("interpolation samples have no measurable extent")]
    DegenerateInterpolationSamples,
    #[error("interpolation parameter count mismatch: expected {expected}, got {got}")]
    InterpolationParameterCountMismatch { expected: usize, got: usize },
    #[error("interpolation parameters must be strictly increasing")]
    InvalidInterpolationParameters,
    #[error("interpolation system is singular")]
    SingularInterpolationSystem,
    #[error("degree {from} cannot be lowered to {to}: elevation is exact, reduction approximates")]
    DegreeReductionRefused { from: usize, to: usize },
    #[error("skinning needs at least {minimum} sections, got {got}")]
    InsufficientSkinningSections { minimum: usize, got: usize },
    #[error("section {index} is not compatible with section 0: {reason}")]
    IncompatibleSkinningSection {
        index: usize,
        reason: SkinningIncompatibility,
    },
    #[error("v-degree {degree} needs at least {degree} + 1 sections, got {sections}")]
    SkinningDegreeTooHigh { degree: usize, sections: usize },
}

/// How one section of a skin fails to match the first.
///
/// Skinning interpolates one grid of control points across the sections, so
/// the sections must first agree on everything that decides what a control
/// point index *means*. Each variant names one of those agreements, because a
/// bare "incompatible" leaves the caller nothing to act on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SkinningIncompatibility {
    /// The degrees differ, so the basis functions are not the same functions.
    Degree,
    /// The control point counts differ, so the grid is not rectangular.
    ControlPointCount,
    /// The knot vectors differ, so equal indices name unequal basis functions.
    KnotVector,
}

impl std::fmt::Display for SkinningIncompatibility {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str(match self {
            Self::Degree => "degrees differ",
            Self::ControlPointCount => "control point counts differ",
            Self::KnotVector => "knot vectors differ",
        })
    }
}
