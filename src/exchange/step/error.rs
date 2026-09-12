//! Errors, one enum per layer, nested.
//!
//! A message says which layer failed and a caller can match on the layer
//! rather than on sixty variants. Every error that names a position carries
//! the entity id or the cell key it happened at, because a STEP error a user
//! cannot locate is not actionable.

use std::path::PathBuf;

use thiserror::Error;

use crate::topology::Dart;
use crate::topology::shape_keys::{EdgeKey, FaceKey, SolidKey, VertexKey};

use super::part21::{SyntaxError, WriteError};

/// Anything that can go wrong reading or writing a STEP file.
#[derive(Debug, Error)]
pub enum StepError {
    /// **L1** — the text is not a well-formed Part 21 exchange structure.
    #[error("STEP syntax: {0}")]
    Syntax(#[from] SyntaxError),

    /// **L1** — a value has no Part 21 spelling, or the sink failed.
    #[error("STEP output: {0}")]
    Output(#[from] WriteError),

    /// **L3** — geometry NGK holds that this stage cannot yet map.
    #[error("STEP geometry: {0}")]
    Geometry(#[from] GeometryError),

    /// **L4** — topology NGK holds that this stage cannot yet map, or that
    /// STEP cannot represent at all.
    #[error("STEP topology: {0}")]
    Topology(#[from] TopologyError),

    /// The file could not be read from or written to disk.
    ///
    /// Carries the path, because "no such file" without one is not actionable.
    #[error("{path}: {source}")]
    Io {
        /// The file involved.
        path: PathBuf,
        /// What the filesystem said.
        #[source]
        source: std::io::Error,
    },

    /// A direction of the mapping that has not been built yet.
    ///
    /// Distinct from the `Unsupported*` variants below, which are geometry
    /// *kinds* a working direction cannot yet carry. This one says the whole
    /// direction is absent, and names the stage of `plan/step_interop.md` that
    /// closes it so the message is a pointer rather than a dead end.
    #[error("{what} is not implemented yet (stage {stage} of the STEP interop plan)")]
    NotImplemented {
        /// What was asked for.
        what: &'static str,
        /// The stage that delivers it.
        stage: u8,
    },
}

/// **L3** — a curve or surface that does not reach an entity.
///
/// Every variant here is a *gap*, not a corruption: the geometry is valid and
/// the mapping for it simply has not landed yet. The NURBS fallback (D2)
/// closes the two `Unsupported` variants once it exists, at which point these
/// become unreachable for anything NGK can hold.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum GeometryError {
    /// A surface kind this stage does not write.
    #[error("surface kind `{kind}` is not written yet")]
    UnsupportedSurface {
        /// The `Surface` variant met.
        kind: &'static str,
    },

    /// A curve kind this stage does not write.
    #[error("curve kind `{kind}` is not written yet")]
    UnsupportedCurve {
        /// The `Curve` variant met.
        kind: &'static str,
    },

    /// A line whose direction vector has no length, which has no `VECTOR`.
    #[error("a line with no extent has no STEP representation")]
    DegenerateLine,
}

/// **L4** — topology that does not reach a shell.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum TopologyError {
    /// The map holds no solid under the key asked for.
    #[error("solid {solid:?} is not registered in this map")]
    UnknownSolid {
        /// The key asked for.
        solid: SolidKey,
    },

    /// A boundary dart belonging to no registered edge.
    #[error("dart {dart:?} belongs to no registered edge")]
    UnregisteredEdge {
        /// The dart met.
        dart: Dart,
    },

    /// An edge closing on itself, which needs a seam to be written (stage 4).
    #[error("edge {edge:?} closes on itself, which needs seam synthesis")]
    ClosedEdge {
        /// The edge met.
        edge: EdgeKey,
    },

    /// An edge carrying no curve, which cannot become an `EDGE_CURVE`.
    #[error("edge {edge:?} carries no curve")]
    MissingCurve {
        /// The edge met.
        edge: EdgeKey,
    },

    /// A vertex carrying no point, which cannot become a `VERTEX_POINT`.
    #[error("vertex {vertex:?} carries no point")]
    MissingVertexPoint {
        /// The vertex met.
        vertex: VertexKey,
    },

    /// A face with no loops at all: a whole sphere or torus (stage 5).
    #[error("face {face:?} is boundaryless, which needs a synthesized seam")]
    BoundarylessFace {
        /// The face met.
        face: FaceKey,
    },

    /// A loop that closes on the periodic quotient rather than in parameter
    /// space, which STEP writes cut open along a seam (stage 4).
    #[error("face {face:?} carries a {kind} loop, which needs seam synthesis")]
    PeriodicLoop {
        /// The face met.
        face: FaceKey,
        /// The loop kind met, named as `LoopKind` spells it.
        kind: &'static str,
    },

    /// A solid with cavities, which needs `BREP_WITH_VOIDS` (stage 5).
    #[error("solid {solid:?} has {count} inner shell(s), which need BREP_WITH_VOIDS")]
    InnerShells {
        /// The solid met.
        solid: SolidKey,
        /// How many cavities it has.
        count: usize,
    },

    /// A face whose orientation relative to its surface could not be read.
    ///
    /// NGK derives a face's sense from its boundary winding rather than
    /// storing it, so a boundary that cannot be sampled leaves `same_sense`
    /// with no answer. Guessing would invert the normal and surface much
    /// later as a failed orientation validation, so it is refused here.
    #[error("face {face:?} has no readable boundary winding, so its sense is unknown")]
    UnreadableFaceSense {
        /// The face met.
        face: FaceKey,
    },
}
