use crate::{
    geometry::NurbsError,
    topology::{Dart, gmap::Dim, shape_keys::FaceKey},
};

#[derive(Debug, Clone, PartialEq)]
pub enum ExtrudeError {
    EmptyProfile,
    MissingVertexPoint { dart: Dart },
    MissingEdgeCurve { dart: Dart },
    ZeroDirection,
    ZeroLengthEdge { dart: Dart },
    DegenerateSweep { dart: Dart },
    SewFailed { dim: Dim, first: Dart, second: Dart },
    CurveTranslationFailed { dart: Dart, source: NurbsError },
    SurfaceTranslationFailed { dart: Dart, source: NurbsError },
    MissingFace { dart: FaceKey },
}
