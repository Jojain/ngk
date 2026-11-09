use crate::geometry::{Interval, TrimmedCurve, TrimmedCurve2};
use crate::topology::shape_keys::{EdgeKey, VertexKey};

/// A curve carried onto a target, as the pieces the target gave it.
///
/// The pieces are never glued into one curve: each [`ProjectedChain`] is a run
/// of segments the projection knows to be continuous, and the chains of one
/// result are whatever the source split into.
#[derive(Debug, Clone, PartialEq)]
pub struct CurveProjection<Hit> {
    /// The runs of continuous segments, in the order the source is walked.
    pub chains: Vec<ProjectedChain<Hit>>,
}

impl<Hit> CurveProjection<Hit> {
    /// Iterates every segment of every chain.
    pub fn segments(&self) -> impl Iterator<Item = &ProjectedSegment<Hit>> {
        self.chains.iter().flat_map(|chain| chain.segments.iter())
    }

    /// Iterates the 3D curve of every segment.
    pub fn curves(&self) -> impl Iterator<Item = &TrimmedCurve> {
        self.segments().map(|segment| &segment.curve)
    }

    /// Iterates the pcurve of every segment.
    pub fn pcurves(&self) -> impl Iterator<Item = &TrimmedCurve2> {
        self.segments().map(|segment| &segment.pcurve)
    }
}

/// A run of segments that each begin where the previous one ends.
#[derive(Debug, Clone, PartialEq)]
pub struct ProjectedChain<Hit> {
    /// The segments in order, never empty.
    pub segments: Vec<ProjectedSegment<Hit>>,
    /// Why the chain begins where it does.
    pub start: ChainEnd,
    /// Why the chain stops where it does.
    pub end: ChainEnd,
}

impl<Hit> ProjectedChain<Hit> {
    /// Whether the chain runs round to where it began.
    pub fn closed(&self) -> bool {
        self.start == ChainEnd::Closed && self.end == ChainEnd::Closed
    }

    /// Iterates the 3D curve of every segment.
    pub fn curves(&self) -> impl Iterator<Item = &TrimmedCurve> {
        self.segments.iter().map(|segment| &segment.curve)
    }

    /// Iterates the pcurve of every segment.
    pub fn pcurves(&self) -> impl Iterator<Item = &TrimmedCurve2> {
        self.segments.iter().map(|segment| &segment.pcurve)
    }
}

/// One piece of the projection: the image of one span of the source on one
/// part of the target.
///
/// `curve` and `pcurve` are the same piece in 3D and in the target's parameter
/// space; both run in the direction the source was walked, and a point at a
/// given fraction of one is the point at that fraction of the other.
#[derive(Debug, Clone, PartialEq)]
pub struct ProjectedSegment<Hit> {
    /// The piece in 3D, on the target.
    pub curve: TrimmedCurve,
    /// The piece in the target's parameter space.
    pub pcurve: TrimmedCurve2,
    /// What the target reports as touched.
    pub hit: Hit,
    /// The part of the source this piece is the image of.
    pub source: SourceSpan,
    /// Whether `curve` and `pcurve` are the image itself or a fit to it.
    pub fit: Exactness,
}

/// What a projected segment is the image of.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SegmentSource {
    /// The curve that was projected, which is not an edge of the model.
    Curve,
    /// An edge of the model.
    Edge(EdgeKey),
}

/// A source and the span of its parameters a segment covers.
///
/// The span is directed: a source walked against its own parameter has
/// `start` past `end`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SourceSpan {
    /// What was projected.
    pub source: SegmentSource,
    /// The native parameters of the source's support that the segment covers.
    pub interval: Interval,
}

/// Why a chain stops, or begins, where it does.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChainEnd {
    /// The chain runs round to its own other end.
    Closed,
    /// The source itself ends here.
    SourceEnd,
    /// The source branches or meets itself at this vertex, so every chain
    /// there ends on it.
    SourceVertex(VertexKey),
    /// The projection stops existing here: the source passes behind the target
    /// or past it.
    Miss,
    /// The projection leaps to a different part of the target here.
    Jump,
}

/// How closely a projected segment follows the image of its source.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Exactness {
    /// The segment is the image itself, in closed form.
    Exact,
    /// The segment is a fit to the image, off it by at most `error`.
    Approx {
        /// The largest distance between the fit and the image.
        error: f64,
    },
}
