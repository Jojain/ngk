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

use super::part21::{EntityId, SyntaxError, WriteError};
use super::schema::resolver::{Origin, SchemaError};

/// Anything that can go wrong reading or writing a STEP file.
#[derive(Debug, Error)]
pub enum StepError {
    /// **L1** — the text is not a well-formed Part 21 exchange structure.
    #[error("STEP syntax: {0}")]
    Syntax(#[from] SyntaxError),

    /// **L1** — a value has no Part 21 spelling, or the sink failed.
    #[error("STEP output: {0}")]
    Output(#[from] WriteError),

    /// **L2** — the entity model is not what the schema requires.
    #[error("STEP schema: {0}")]
    Schema(#[from] SchemaError),

    /// **L3** — a curve or surface that does not reach an entity.
    #[error("STEP geometry: {0}")]
    Geometry(#[from] GeometryError),

    /// **L4** — topology that does not reach a shell, or that STEP cannot
    /// represent at all.
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
}

/// **L3** — a curve or surface that does not reach an entity.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum GeometryError {
    /// A surface kind with no entry in the writer's dispatch.
    #[error("surface kind `{kind}` has no STEP entity in this build")]
    UnsupportedSurface {
        /// The `Surface` variant met.
        kind: &'static str,
    },

    /// A curve kind with no entry in the writer's dispatch.
    #[error("curve kind `{kind}` has no STEP entity in this build")]
    UnsupportedCurve {
        /// The `Curve` variant met.
        kind: &'static str,
    },

    /// A surface entity with no entry in the reader's dispatch.
    #[error("{origin}: surface entity `{keyword}` has no mapping in this build")]
    UnreadableSurface {
        /// The entity keyword met, as the file spells it.
        keyword: String,
        /// Where it was.
        origin: Origin,
    },

    /// A curve entity with no entry in the reader's dispatch.
    #[error("{origin}: curve entity `{keyword}` has no mapping in this build")]
    UnreadableCurve {
        /// The entity keyword met, as the file spells it.
        keyword: String,
        /// Where it was.
        origin: Origin,
    },

    /// A curve that would not project into its face's parameter space.
    ///
    /// A parameter curve is rebuilt from the 3D curve rather than taken from
    /// the file, and on a plane that projection is exact — so this fires only
    /// for geometry too degenerate to carry a control polygon.
    #[error("{origin}: curve does not project onto the face's plane: {detail}")]
    UnprojectableCurve {
        /// Where the `EDGE_CURVE` was.
        origin: Origin,
        /// What the conversion said.
        detail: String,
    },

    /// A line whose direction vector has no length, which has no `VECTOR`.
    #[error("a line with no extent has no STEP representation")]
    DegenerateLine,

    /// A cone whose generatrix runs along the axis or perpendicular to it.
    ///
    /// `CONICAL_SURFACE` requires a semi-angle strictly between zero and a
    /// right angle. At zero the surface is a cylinder and at a right angle it
    /// is a plane through the apex, and in both cases the substitution
    /// relating STEP's axial `v` to NGK's generatrix `v` has no finite answer.
    #[error("a cone of half angle {half_angle} radians is not a `CONICAL_SURFACE`")]
    DegenerateCone {
        /// The half angle met, in radians.
        half_angle: f64,
    },
}

/// **L4** — topology that does not reach a shell, or a shell that does not
/// reach a map.
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

    /// An edge closing on itself.
    ///
    /// Such an edge has two uses on one face and no distinct end vertices, so
    /// there is no `EDGE_CURVE` direction to write it under until the face's
    /// parameterization is cut open along a synthesized seam.
    #[error("edge {edge:?} closes on itself, so it has no directed STEP spelling")]
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

    /// A face with no loops at all: a whole sphere or torus.
    ///
    /// STEP has no boundaryless face, so writing one means synthesizing a
    /// boundary for it out of the surface's own domain.
    #[error("face {face:?} is boundaryless, so it has no STEP bounds to write")]
    BoundarylessFace {
        /// The face met.
        face: FaceKey,
    },

    /// A face whose parameter domain could not be cut open.
    ///
    /// STEP bounds every face by loops that close in parameter space, so a
    /// periodic face is written along a synthesized cut. Deriving that cut
    /// reads the face's own parameter curves, and a face missing one — or
    /// whose boundary the cut does not account for — has no seamed spelling.
    #[error("face {face:?} has no seamed boundary: {detail}")]
    UncuttableFace {
        /// The face met.
        face: FaceKey,
        /// What the unwrapped domain said.
        detail: String,
    },

    /// A cut this build has no analytic curve for.
    ///
    /// A synthesized seam runs along a parameter line, and on the supports
    /// with a closed form for theirs it is a line or a circle. Anything else
    /// would have to be approximated, which would put a boundary on the file
    /// that is not quite on the surface — so the face is refused by name.
    #[error("face {face:?} needs a seam its support has no analytic curve for")]
    UnwritableSeam {
        /// The face met.
        face: FaceKey,
    },

    /// A solid with cavities, which needs `BREP_WITH_VOIDS`.
    #[error("solid {solid:?} has {count} inner shell(s), which `MANIFOLD_SOLID_BREP` cannot carry")]
    InnerShells {
        /// The solid met.
        solid: SolidKey,
        /// How many cavities it has.
        count: usize,
    },

    /// An edge used by more than two faces.
    ///
    /// NGK is a 3-GMap and has no way to hold a non-manifold edge, so the
    /// solid carrying it is refused by name rather than sewn into something
    /// that is not the shape the file described.
    #[error("{brep}: edge {edge} is used by more than two faces")]
    NonManifoldShell {
        /// Where the `MANIFOLD_SOLID_BREP` was.
        brep: Origin,
        /// The `EDGE_CURVE` with too many uses.
        edge: EntityId,
    },

    /// An edge used by exactly one face, leaving the shell open.
    ///
    /// Only an error under [`StepReadOptions::strict`]; a lenient read builds
    /// the map anyway and records it, since an open shell is still most of a
    /// shape.
    ///
    /// [`StepReadOptions::strict`]: super::options::StepReadOptions::strict
    #[error("{brep}: edge {edge} is used by only one face")]
    OpenShell {
        /// Where the `MANIFOLD_SOLID_BREP` was.
        brep: Origin,
        /// The `EDGE_CURVE` with one use.
        edge: EntityId,
    },

    /// A shell whose faces would not sew into a map.
    ///
    /// The detail is the topology layer's own message. The sewing happens in
    /// a transaction, which restores its snapshot on failure, so what reaches
    /// here is a solid that was not built rather than a half-built one.
    #[error("{brep} could not be sewn: {detail}")]
    UnsewableShell {
        /// Where the `MANIFOLD_SOLID_BREP` was.
        brep: Origin,
        /// What the topology layer said.
        detail: String,
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
