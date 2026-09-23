//! How a sweep refuses.

use thiserror::Error;

use crate::geometry::NurbsError;
use crate::topology::edit::ModelEditError;
use crate::topology::gmap::{Dart, Dim};
use crate::topology::shape_keys::{EdgeKey, FaceKey, ProfileKey};

/// Everything a sweep declines to build, and why.
#[derive(Debug, Error)]
pub enum SweepError {
    /// No path segment was supplied.
    #[error("a sweep needs at least one spine segment")]
    EmptySpine,

    /// Two consecutive path segments do not share their junction point.
    #[error("spine segments do not meet at junction {junction}")]
    DisconnectedSpine { junction: usize },

    /// The section plane is not perpendicular to the first spine tangent.
    #[error("the section normal is not parallel to the spine's first tangent")]
    SectionNotNormalToSpine,

    /// The section's plane runs along the spine instead of across it.
    ///
    /// An [`Axial`](super::SweepFrame::Axial) sweep carries the section
    /// rigidly, so the section need not stand square to the spine -- but one
    /// lying along it sweeps no volume.
    #[error("the section lies along the spine and sweeps no volume")]
    SectionAlongSpine,

    /// The spine touches the axis of an [`Axial`](super::SweepFrame::Axial)
    /// sweep.
    ///
    /// The frame's `x` runs out from the axis to the spine, and a point on the
    /// axis names no direction out from it.
    #[error("the spine meets the sweep axis at fraction {fraction}")]
    SpineMeetsTheAxis { fraction: f64 },

    /// The spine stands still where a tangent was asked for.
    ///
    /// A section is carried perpendicular to the spine, so a point where the
    /// spine has no direction names no frame to carry it into.
    #[error("the spine has no direction at fraction {fraction}")]
    SpineHasNoTangent { fraction: f64 },

    /// A Frenet sweep met a stretch the spine does not curve on.
    ///
    /// The principal normal is the direction the tangent turns towards, and a
    /// straight stretch turns towards nothing. Sweep it
    /// [`Parallel`](super::SweepFrame::Parallel) instead, which needs no
    /// curvature, or extrude it.
    #[error("a Frenet sweep needs curvature, and the spine is straight at fraction {fraction}")]
    SpineDoesNotCurve { fraction: f64 },

    /// The spine doubles back between two samples.
    ///
    /// Parallel transport carries the section by the shortest rotation from
    /// one tangent to the next, and two opposite tangents name no shortest
    /// one — every axis through them turns the first onto the second.
    #[error("the spine reverses direction between fractions {from} and {to}")]
    SpineReverses { from: f64, to: f64 },

    /// The spine returns the section to where it started.
    ///
    /// A sweep round a closed spine has no two ends to cap: the section meets
    /// itself, and the two ends have to be joined to each other instead.
    #[error("the spine closes, and a sweep round a closed spine has no ends to cap")]
    SpineClosesOnItself,

    /// The spine does not run smoothly through one of its junctions.
    ///
    /// The frame arriving and the frame leaving differ, so the section would
    /// have to turn there. Sweep with
    /// [`SweepTransition::Rounded`](super::SweepTransition::Rounded) to turn it,
    /// or give the spine a fillet so the junction is smooth.
    #[error("the spine turns {angle} radians at junction {junction}")]
    SpineTurnsACorner { junction: usize, angle: f64 },

    /// A section edge has no NURBS form to skin from.
    ///
    /// The path never needs one — it is sampled, not converted — but the
    /// section is transformed control point by control point, so a support
    /// with no finite NURBS form cannot be swept.
    #[error("edge {key:?} has no NURBS form to sweep")]
    SectionHasNoNurbsForm { key: EdgeKey, source: NurbsError },

    /// A section edge runs along the direction it is being carried in.
    ///
    /// It sweeps no area, so there is no wall there to build — and none of
    /// the ways of describing a wall would have made one.
    #[error("edge {key:?} runs along the sweep and sweeps no area")]
    SectionRunsAlongTheSweep { key: EdgeKey },

    /// A section edge meets the turning axis in an unsupported way.
    ///
    /// An edge wholly on the axis sweeps no wall and is omitted. An edge with
    /// one endpoint on the axis sweeps a triangular wall. An edge that crosses
    /// the axis, or leaves it and returns to it, makes the section sweep into
    /// itself and is refused.
    ///
    /// The usual cause is a section centred on the spine at a sharp corner,
    /// where the corner's axis runs through the middle of it. A corner with
    /// no radius has no room to turn a section in; give the spine a fillet
    /// wider than the section reaches and sweep it
    /// [`Smooth`](super::SweepTransition::Smooth).
    #[error("edge {key:?} crosses or returns to the axis it is turned about")]
    SectionMeetsTheTurningAxis { key: EdgeKey },

    /// Turning this section about the sharp corner would run it back through
    /// one of the adjacent straight wall groups.
    ///
    /// A zero-radius rounded transition can revolve only the side of a section
    /// outside the turn. Geometry on the inside needs the two straight wall groups
    /// intersected and trimmed. This builder performs a pure revolution, so it
    /// refuses that configuration instead of committing a self-intersecting
    /// solid.
    #[error("edge {key:?} lies inside the turn, so the rounded transition would self-intersect")]
    RoundedTransitionWouldSelfIntersect { key: EdgeKey },

    /// Skinning the sampled sections into a surface failed.
    #[error("skinning the swept wall of edge {key:?} failed")]
    Skinning { key: EdgeKey, source: NurbsError },

    /// Reading an isocurve back off a swept wall failed.
    #[error("reading the boundary of the swept wall of edge {key:?} failed")]
    Boundary { key: EdgeKey, source: NurbsError },

    /// The named profile is not in the model.
    #[error("no profile {key:?}")]
    MissingProfile { key: ProfileKey },

    /// The named face is not in the model.
    #[error("no face {key:?}")]
    MissingFace { key: FaceKey },

    /// A boundary dart carries no edge.
    #[error("dart {dart:?} carries no edge")]
    MissingEdge { dart: Dart },

    /// A face offered as a section has no outer boundary to sweep.
    #[error("a face with no outer loop names no section")]
    FaceHasNoOuterLoop,

    /// Two walls of the same sweep could not be joined.
    #[error("darts {first:?} and {second:?} are not sewable in dimension {dim:?}")]
    SewFailed { dim: Dim, first: Dart, second: Dart },

    /// The edit rejected the topology the sweep built.
    #[error("building the swept topology failed")]
    ModelEditFailed(#[from] ModelEditError),
}
