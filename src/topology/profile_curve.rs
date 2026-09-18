//! One directed traversal of a profile, parametrized over `[0, 1]`.
//!
//! A profile answers "which edges", and each edge answers over *its own*
//! native span. Nothing in the kernel answers "where is this section at
//! fraction `t`" — which is the only question that makes two sections with
//! unlike edge counts comparable at all.
//!
//! Joining a section's edges into one NURBS curve would answer it and throw
//! away what the answer is wanted for: which edge and which vertex a parameter
//! belongs to. So the adaptor stays topology-aware. Each edge keeps its own
//! exact support and its own slice of `[0, 1]`, and the edge structure becomes
//! an annotation on a shared parameter line rather than the section's
//! interface.
//!
//! # A traversal, not a profile
//!
//! *A profile says which edges, a loop says how a face runs along them.* A
//! section is the second of those: it has a start, an order and a sense. So
//! this is built over a walk, and both kinds of walk the kernel has —
//! [`Profile`] and [`Loop`] — can supply one.
//!
//! A walk that emits one edge twice, such as a slit or a marked edge passed on
//! both sides, has no single-valued parametrization, and is refused by name
//! rather than flattened into one.
//!
//! # Span allocation is by arc length
//!
//! Each edge gets a slice of `[0, 1]` proportional to its length. An equal
//! slice per edge would make a section's parametrization a function of how
//! finely it happens to be divided: two sections describing the same shape,
//! one of them carrying an extra vertex from an earlier split, would then
//! correspond wrongly along their whole length. Arc length is a property of
//! the shape; edge count is a property of its history.
//!
//! # Where this sits in the fraction rule
//!
//! Only a [`TrimmedCurve`] produces or consumes a [`Fraction`], because only a
//! [`TrimmedCurve`] carries a span to be a fraction *of*. A `ProfileCurve`
//! carries an ordered sequence of spans, so it is entitled to the brand on the
//! same grounds and is the third member of that family. Its `Fraction` is a
//! fraction of the *traversal*, and [`ProfileCurve::locate`] is the conversion
//! down to a fraction of one span.

use std::collections::HashSet;

use nalgebra::Vector3;
use thiserror::Error;

use crate::geometry::parameter::{Fraction, Normalized};
use crate::geometry::{Interval, LINEAR_TOLERANCE, Point3, TrimmedCurve};
use crate::topology::closed::Closeable;
use crate::topology::edge::Edge;
use crate::topology::face::Loop;
use crate::topology::payload::{Payload, StandardPayload};
use crate::topology::profile::Profile;
use crate::topology::shape_keys::EdgeKey;

/// How many uniform probes a seam search adds to the stated breakpoints.
///
/// Two sections need not share a corner anywhere — a circle against a square
/// has none to line up — so the search cannot be driven by breakpoints alone.
/// The grid is what gives such a pair a rotation to prefer at all.
const SEAM_PROBE_COUNT: usize = 32;

/// How a traversal fails to be one curve.
#[derive(Debug, Clone, Error, PartialEq)]
pub enum ProfileCurveError {
    /// The walk emitted no edge, so there is nothing to parametrize.
    #[error("a traversal with no edges has no parametrization")]
    EmptyTraversal,
    /// The walk emitted one edge twice.
    ///
    /// A slit, or a marked edge a face passes on both sides. Two visits to one
    /// edge give two fractions the same point, so the traversal is not a
    /// single-valued curve and cannot be corresponded against another.
    #[error("edge {key:?} appears twice in the traversal, which is not single-valued")]
    RepeatedEdge { key: EdgeKey },
    /// An edge of the walk has no measurable length.
    #[error("edge {key:?} has no measurable length, so it claims no part of the traversal")]
    DegenerateEdge { key: EdgeKey },
    /// The whole walk has no measurable length.
    #[error("the traversal has no measurable length")]
    DegenerateTraversal,
    /// A rotation was asked of an open traversal.
    ///
    /// Only a closed traversal has an arbitrary start to move. An open one's
    /// start is an end of the section, and moving it would describe a
    /// different section.
    #[error("an open traversal has no seam to rotate")]
    OpenTraversalRotation,
    /// A subdivision was asked for with fewer than two boundaries.
    #[error("a subdivision needs at least two boundaries, got {got}")]
    TooFewBoundaries { got: usize },
    /// One requested piece spans more than one edge of the traversal.
    ///
    /// Splitting is exact only inside one edge, because a piece crossing a
    /// corner has two supports and no single [`TrimmedCurve`] carries both.
    /// A boundary set that contains this traversal's own breakpoints never
    /// asks for one.
    #[error("the piece from {from} to {to} crosses a corner of the traversal")]
    PieceCrossesCorner { from: f64, to: f64 },
}

/// One edge's part of a traversal.
///
/// The section is already oriented the way the traversal runs, so its
/// fraction `0` is where the traversal enters the edge whichever way the
/// edge's own stored direction points.
pub struct ProfileSpan<'a, P: Payload = StandardPayload> {
    edge: Edge<'a, P>,
    section: TrimmedCurve,
    extent: Interval<Normalized>,
    /// Whether a corner sits where this part begins.
    ///
    /// True of every boundary between two edges, because two edges meet at a
    /// vertex. False in the two cases where a part's start is not a meeting
    /// place: the closure point of an unmarked edge, and the cut a rotation
    /// makes inside one.
    starts_at_corner: bool,
}

impl<'a, P: Payload> Clone for ProfileSpan<'a, P> {
    fn clone(&self) -> Self {
        Self {
            edge: self.edge,
            section: self.section.clone(),
            extent: self.extent,
            starts_at_corner: self.starts_at_corner,
        }
    }
}

impl<'a, P: Payload> ProfileSpan<'a, P> {
    /// The edge this part of the traversal runs along.
    pub fn edge(&self) -> Edge<'a, P> {
        self.edge
    }

    /// The geometry of this part, oriented the way the traversal runs.
    pub fn section(&self) -> &TrimmedCurve {
        &self.section
    }

    /// This part's slice of the traversal's `[0, 1]`.
    pub fn extent(&self) -> Interval<Normalized> {
        self.extent
    }

    /// Whether a corner sits where this part begins.
    pub fn starts_at_corner(&self) -> bool {
        self.starts_at_corner
    }
}

/// One directed traversal of a profile, parametrized over `[0, 1]`.
///
/// See the [module documentation](self) for what this is for and why it keeps
/// the edges rather than joining them.
pub struct ProfileCurve<'a, P: Payload = StandardPayload> {
    spans: Vec<ProfileSpan<'a, P>>,
    closed: bool,
    length: f64,
}

impl<'a, P: Payload> Clone for ProfileCurve<'a, P> {
    fn clone(&self) -> Self {
        Self {
            spans: self.spans.clone(),
            closed: self.closed,
            length: self.length,
        }
    }
}

impl<'a, P: Payload> ProfileCurve<'a, P> {
    /// Parametrizes a profile in its own traversal order and sense.
    pub fn from_profile(profile: &Profile<'a, P>) -> Result<Self, ProfileCurveError> {
        let edges = profile.edges();
        let closed = profile.is_closed();
        Self::from_edges(edges, closed)
    }

    /// Parametrizes a face's boundary loop in the loop's own direction.
    pub fn from_loop(loop_: &Loop<'a, P>) -> Result<Self, ProfileCurveError> {
        Self::from_edges(loop_.edges(), true)
    }

    /// Builds a traversal from its oriented edges, allocating `[0, 1]` by arc
    /// length.
    fn from_edges(edges: Vec<Edge<'a, P>>, closed: bool) -> Result<Self, ProfileCurveError> {
        if edges.is_empty() {
            return Err(ProfileCurveError::EmptyTraversal);
        }
        let mut seen = HashSet::with_capacity(edges.len());
        for edge in &edges {
            if !seen.insert(edge.key()) {
                return Err(ProfileCurveError::RepeatedEdge { key: edge.key() });
            }
        }

        // A walk arrives at the first edge's own dart, and every shape of edge
        // but one carries a corner there: a bounded edge has one at each end,
        // a marked edge has its single corner wherever it is read from. An
        // unmarked edge has none anywhere, so a closed traversal of one starts
        // on no corner at all. Every later part begins where two edges meet,
        // which is a vertex by definition.
        let parts = edges
            .iter()
            .enumerate()
            .map(|(index, edge)| Part {
                edge: *edge,
                section: edge.trimmed_curve(),
                starts_at_corner: index > 0 || !matches!(edge, Edge::Unmarked(_)),
            })
            .collect::<Vec<_>>();
        Self::from_parts(parts, closed)
    }

    /// Lays parts out over `[0, 1]` by arc length, refusing a degenerate one.
    fn from_parts(parts: Vec<Part<'a, P>>, closed: bool) -> Result<Self, ProfileCurveError> {
        let lengths = parts
            .iter()
            .map(|part| {
                let length = part.section.length();
                (length > LINEAR_TOLERANCE).then_some(length).ok_or(
                    ProfileCurveError::DegenerateEdge {
                        key: part.edge.key(),
                    },
                )
            })
            .collect::<Result<Vec<_>, _>>()?;
        let length = lengths.iter().sum::<f64>();
        if length <= LINEAR_TOLERANCE {
            return Err(ProfileCurveError::DegenerateTraversal);
        }
        Ok(Self {
            spans: allocate_spans(parts, &lengths, length),
            closed,
            length,
        })
    }

    /// Whether the traversal returns to where it started.
    pub fn is_closed(&self) -> bool {
        self.closed
    }

    /// The arc length of the whole traversal.
    pub fn length(&self) -> f64 {
        self.length
    }

    /// The parts of the traversal, in order.
    pub fn spans(&self) -> &[ProfileSpan<'a, P>] {
        &self.spans
    }

    /// The fraction below which two parameters of this traversal name one
    /// place.
    ///
    /// A fraction is not a distance, so this is derived from the traversal's
    /// own length: a merge at this tolerance means "these corners are closer
    /// than the kernel can tell apart in space", which is the question a
    /// breakpoint union actually asks.
    pub fn merge_tolerance(&self) -> f64 {
        LINEAR_TOLERANCE / self.length
    }

    /// The point at a fraction of the traversal.
    ///
    /// A closed traversal wraps, so a fraction outside `[0, 1]` names a point
    /// on it like any other; an open one clamps, because there is nothing
    /// beyond its ends.
    pub fn point_at(&self, t: Fraction) -> Point3 {
        let (index, local) = self.locate(t);
        self.spans[index].section.point_at(local)
    }

    /// Which span a fraction falls in, and where in that span.
    ///
    /// The conversion from a fraction of the traversal down to a fraction of
    /// one span, as [`Interval::at`] is the conversion between a fraction and
    /// a native parameter. A fraction on a boundary between two spans resolves
    /// to the *later* one, at its fraction `0`.
    pub fn locate(&self, t: Fraction) -> (usize, Fraction) {
        let value = self.wrapped(t);
        let tolerance = self.merge_tolerance();
        let index = self
            .spans
            .iter()
            .position(|span| value < span.extent.end.value() - tolerance)
            .unwrap_or(self.spans.len() - 1);
        (index, self.local_fraction(index, value))
    }

    /// Which span a fraction *ends* in, and where in that span.
    ///
    /// The twin of [`Self::locate`] for the far end of a piece: a fraction on
    /// a boundary resolves to the *earlier* span, at its fraction `1`. A piece
    /// is then named by one span whenever both its ends land on one, which is
    /// what makes a subdivision at the traversal's own breakpoints exact.
    fn locate_end(&self, t: Fraction) -> (usize, Fraction) {
        let value = self.wrapped_end(t);
        let tolerance = self.merge_tolerance();
        let index = self
            .spans
            .iter()
            .rposition(|span| value > span.extent.start.value() + tolerance)
            .unwrap_or(0);
        (index, self.local_fraction(index, value))
    }

    /// Where `value` falls within span `index`, confined to that span.
    fn local_fraction(&self, index: usize, value: f64) -> Fraction {
        self.spans[index]
            .extent
            .fraction_of(Fraction::new(value))
            .clamped_to_unit()
    }

    /// `t` brought onto the traversal: wrapped when closed, clamped when open.
    fn wrapped(&self, t: Fraction) -> f64 {
        let value = t.value();
        if self.closed && !(0.0..=1.0).contains(&value) {
            value.rem_euclid(1.0)
        } else {
            value.clamp(0.0, 1.0)
        }
    }

    /// As [`Self::wrapped`], reading an exact `0` at the far end of a closed
    /// traversal as the `1` it also is.
    fn wrapped_end(&self, t: Fraction) -> f64 {
        let value = self.wrapped(t);
        if self.closed && value == 0.0 && t.value() > 0.0 {
            1.0
        } else {
            value
        }
    }

    /// The fractions at which the traversal meets a corner.
    ///
    /// Fraction `1` is listed for an open traversal, where it is an end of the
    /// section, and not for a closed one, where it is fraction `0` again.
    /// Fraction `0` is listed whenever a corner is there — which a rotation
    /// can move it off, and an unmarked closed edge never had.
    pub fn breakpoints(&self) -> Vec<Fraction> {
        let mut breaks = self
            .spans
            .iter()
            .filter(|span| span.starts_at_corner)
            .map(|span| span.extent.start)
            .collect::<Vec<_>>();
        if !self.closed {
            breaks.push(Fraction::END);
        }
        breaks
    }

    /// The pieces of the traversal between consecutive boundaries.
    ///
    /// `at` names the boundaries in increasing order, and the result has one
    /// piece per consecutive pair. Splitting costs nothing geometrically:
    /// [`TrimmedCurve::sub`] narrows the stored interval without touching the
    /// support, so a quarter of a circle stays an exact circle.
    ///
    /// Every piece must lie within one edge. A boundary set containing this
    /// traversal's own [`breakpoints`](Self::breakpoints) guarantees that, and
    /// one that does not is refused rather than answered with a piece that
    /// silently drops a corner.
    pub fn subdivided(&self, at: &[Fraction]) -> Result<Vec<TrimmedCurve>, ProfileCurveError> {
        if at.len() < 2 {
            return Err(ProfileCurveError::TooFewBoundaries { got: at.len() });
        }
        at.windows(2)
            .map(|pair| self.piece(pair[0], pair[1]))
            .collect()
    }

    /// The one piece of the traversal between two boundaries.
    fn piece(&self, from: Fraction, to: Fraction) -> Result<TrimmedCurve, ProfileCurveError> {
        let (start, start_fraction) = self.locate(from);
        let (end, end_fraction) = self.locate_end(to);
        if start != end {
            return Err(ProfileCurveError::PieceCrossesCorner {
                from: from.value(),
                to: to.value(),
            });
        }
        Ok(self.spans[start]
            .section
            .sub(Interval::new(start_fraction, end_fraction)))
    }

    /// The same closed traversal, re-anchored so that `t` becomes fraction
    /// `0`.
    ///
    /// The edge `t` falls inside is cut in two, its tail becoming the first
    /// part and its head the last, so the result runs over the same geometry
    /// from a different start. Only a closed traversal has a start to move.
    pub fn rotated_to(&self, t: Fraction) -> Result<Self, ProfileCurveError> {
        if !self.closed {
            return Err(ProfileCurveError::OpenTraversalRotation);
        }
        let t = Fraction::new(t.value().rem_euclid(1.0));
        if let Some(rotated) = self.with_rotated_interval(t) {
            return rotated;
        }
        let (index, fraction) = self.locate(t);
        let on_corner = fraction.value() <= self.merge_tolerance();

        // Whole parts from the one `t` lands in, wrapping round. When `t`
        // falls inside a part rather than on its start, that part is cut and
        // its two pieces become the first and the last of the result — and
        // the cut is a place nothing meets, so neither piece's start is a
        // corner it did not already have.
        let count = self.spans.len();
        let cut = &self.spans[index];
        let mut parts = Vec::with_capacity(count + 1);
        let first_whole = if on_corner {
            0
        } else {
            parts.push(Part {
                edge: cut.edge,
                section: cut.section.sub(Interval::new(fraction, Fraction::END)),
                starts_at_corner: false,
            });
            1
        };
        for offset in first_whole..count {
            let span = &self.spans[(index + offset) % count];
            parts.push(Part {
                edge: span.edge,
                section: span.section.clone(),
                starts_at_corner: span.starts_at_corner,
            });
        }
        if !on_corner {
            parts.push(Part {
                edge: cut.edge,
                section: cut.section.sub(Interval::new(Fraction::START, fraction)),
                starts_at_corner: cut.starts_at_corner,
            });
        }
        Self::from_parts(parts, true)
    }

    /// A closed traversal of one corner-free edge, re-anchored by sliding its
    /// span rather than by cutting it.
    ///
    /// Cutting would leave two parts meeting where nothing meets, and a later
    /// subdivision crossing that place would be refused as a corner crossing
    /// it is not. A closed edge runs a whole period of its support, so moving
    /// the span's ends by the same amount says the same thing exactly and
    /// leaves one part. `None` where that does not apply: with a corner to
    /// carry, or with more than one edge, the parts are real and the cut is
    /// the only way to move the start.
    fn with_rotated_interval(&self, t: Fraction) -> Option<Result<Self, ProfileCurveError>> {
        let [span] = self.spans.as_slice() else {
            return None;
        };
        if span.starts_at_corner {
            return None;
        }
        let interval = span.section.interval();
        let start = interval.at(t);
        Some(Self::from_parts(
            vec![Part {
                edge: span.edge,
                section: TrimmedCurve::new(
                    span.section.curve().clone(),
                    Interval::new(start, start + interval.delta()),
                ),
                starts_at_corner: false,
            }],
            true,
        ))
    }

    /// The same geometry traversed the other way.
    ///
    /// Fraction `0` stays where it was on a closed traversal, because `1` and
    /// `0` are the same place there; on an open one it moves to the far end.
    pub fn reversed(&self) -> Self {
        // Part `j` of the reverse begins where part `n - j` of this traversal
        // began, so that is where its corner flag comes from; part `0` begins
        // at the far end, which on a closed traversal is fraction `0` again
        // and on an open one is an end of the section — a corner either way
        // that part `0` already records.
        let count = self.spans.len();
        let parts = self
            .spans
            .iter()
            .rev()
            .enumerate()
            .map(|(index, span)| Part {
                edge: span.edge,
                section: span.section.reversed(),
                starts_at_corner: if index == 0 {
                    self.spans[0].starts_at_corner
                } else {
                    self.spans[count - index].starts_at_corner
                },
            })
            .collect();
        Self::from_parts(parts, self.closed)
            .expect("reversing keeps every part's length, so none becomes degenerate")
    }

    /// The traversal's centre of mass, sampled uniformly in parameter.
    fn centroid(&self) -> Point3 {
        let samples = self.samples(SEAM_PROBE_COUNT);
        let sum = samples
            .iter()
            .fold(Vector3::zeros(), |acc, point| acc + point.coords);
        Point3::from(sum / samples.len() as f64)
    }

    /// `count` points spaced uniformly over the traversal, the last end
    /// excluded on a closed one because it repeats the first.
    fn samples(&self, count: usize) -> Vec<Point3> {
        let divisor = if self.closed { count } else { count - 1 };
        (0..count)
            .map(|step| self.point_at(Fraction::new(step as f64 / divisor as f64)))
            .collect()
    }

    /// The direction the traversal's enclosed area faces, by Newell's method.
    ///
    /// Its sign is what says which way round the traversal turns, which is
    /// what direction agreement compares between sections and what a sweep
    /// between them reads to decide which side of its faces is outside. An
    /// open traversal is closed up by the chord between its ends, which is
    /// enough to give the same answer.
    pub fn turning_normal(&self) -> Vector3<f64> {
        let samples = self.samples(SEAM_PROBE_COUNT);
        let mut normal = Vector3::zeros();
        for (index, from) in samples.iter().enumerate() {
            let to = samples[(index + 1) % samples.len()];
            normal.x += (from.y - to.y) * (from.z + to.z);
            normal.y += (from.z - to.z) * (from.x + to.x);
            normal.z += (from.x - to.x) * (from.y + to.y);
        }
        normal
    }
}

/// One part of a traversal before it knows its slice of `[0, 1]`.
struct Part<'a, P: Payload> {
    edge: Edge<'a, P>,
    section: TrimmedCurve,
    starts_at_corner: bool,
}

/// Lays `parts` out over `[0, 1]` in proportion to `lengths`.
fn allocate_spans<'a, P: Payload>(
    parts: Vec<Part<'a, P>>,
    lengths: &[f64],
    length: f64,
) -> Vec<ProfileSpan<'a, P>> {
    let count = parts.len();
    let mut start = 0.0;
    parts
        .into_iter()
        .zip(lengths)
        .enumerate()
        .map(|(index, (part, piece))| {
            let end = if index + 1 == count {
                1.0
            } else {
                start + piece / length
            };
            let extent = Interval::new(Fraction::new(start), Fraction::new(end));
            start = end;
            ProfileSpan {
                edge: part.edge,
                section: part.section,
                extent,
                starts_at_corner: part.starts_at_corner,
            }
        })
        .collect()
}

/// Reverses whichever sections run against the chain, anchored on section 0.
///
/// Sections that run opposite ways loft into a self-intersecting bowtie, and
/// that is a valid map holding the wrong shape rather than a failure anything
/// downstream can notice. Each section's own area normal is tested against the
/// vector to the next section, and the sign that answer has to have is read
/// once off section 0 — not off the previous section, because `N` locally
/// consistent pairings can still spiral.
///
/// A chain whose sections all lie in one plane containing the chain direction
/// has no sign to read, and nothing is reversed.
pub fn agree_directions<P: Payload>(sections: &mut [ProfileCurve<'_, P>]) {
    if sections.len() < 2 {
        return;
    }
    let centroids = sections
        .iter()
        .map(ProfileCurve::centroid)
        .collect::<Vec<_>>();
    let last = sections.len() - 1;
    let advance = |index: usize| {
        if index < last {
            centroids[index + 1] - centroids[index]
        } else {
            centroids[last] - centroids[last - 1]
        }
    };

    let reference = sections[0].turning_normal().dot(&advance(0));
    if reference.abs() <= LINEAR_TOLERANCE {
        return;
    }
    for index in 1..sections.len() {
        let sense = sections[index].turning_normal().dot(&advance(index));
        if sense * reference < 0.0 {
            let reversed = sections[index].reversed();
            sections[index] = reversed;
        }
    }
}

/// Rotates each closed section's seam to line up with section 0's.
///
/// A closed section's fraction `0` is arbitrary. If two sections start at
/// unrelated places every column is skewed and the shape arrives twisted, so
/// each is re-anchored at the offset minimizing the summed squared distance
/// between corresponding points. The comparison is against section 0 rather
/// than against the previous section, so a long chain cannot accumulate drift
/// one pairing at a time.
///
/// Candidate offsets are a section's own breakpoints together with a uniform
/// grid: a circle against a square shares no corner, and the grid is what
/// gives such a pair a rotation to prefer.
pub fn align_seams<P: Payload>(
    sections: &mut [ProfileCurve<'_, P>],
) -> Result<(), ProfileCurveError> {
    if sections.len() < 2 || !sections[0].is_closed() {
        return Ok(());
    }
    let probes = probe_fractions(&sections[0]);
    let reference = probes
        .iter()
        .map(|probe| sections[0].point_at(*probe))
        .collect::<Vec<_>>();

    for index in 1..sections.len() {
        if !sections[index].is_closed() {
            continue;
        }
        let best = probe_fractions(&sections[index])
            .into_iter()
            .map(|offset| {
                (
                    seam_cost(&sections[index], offset, &probes, &reference),
                    offset,
                )
            })
            .min_by(|first, second| first.0.total_cmp(&second.0))
            .map(|(_, offset)| offset);
        let Some(offset) = best else { continue };
        if offset.value().abs() <= sections[index].merge_tolerance() {
            continue;
        }
        sections[index] = sections[index].rotated_to(offset)?;
    }
    Ok(())
}

/// The offsets and sample points a seam search works over: a section's own
/// breakpoints, plus a uniform grid.
fn probe_fractions<P: Payload>(section: &ProfileCurve<'_, P>) -> Vec<Fraction> {
    let mut probes = section.breakpoints();
    probes.extend(
        (0..SEAM_PROBE_COUNT).map(|step| Fraction::new(step as f64 / SEAM_PROBE_COUNT as f64)),
    );
    probes
}

/// How far `section` rotated by `offset` sits from `reference` at `probes`.
fn seam_cost<P: Payload>(
    section: &ProfileCurve<'_, P>,
    offset: Fraction,
    probes: &[Fraction],
    reference: &[Point3],
) -> f64 {
    probes
        .iter()
        .zip(reference)
        .map(|(probe, expected)| {
            (section.point_at(Fraction::new(offset.value() + probe.value())) - expected)
                .norm_squared()
        })
        .sum()
}
