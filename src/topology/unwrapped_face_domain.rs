//! Unwrapped face domains: simply-connected views of face parameter domains.
//!
//! A seam is a property of an unwrapped domain, not of a shape. A periodic
//! support has no distinguished place where its closed direction "starts"; an algorithm that
//! needs a planar domain — a winding test, a triangulation grid, a NURBS patch
//! — cuts one open, and that cut belongs to the algorithm, not to the model.
//!
//! An [`UnwrappedFaceDomain`] is that cut, made once and shared. It reads a
//! face's boundary loops, places every pcurve of a loop so the chain runs continuously rather
//! than jumping a whole period whenever the stored loop crosses the cut, and
//! records where the cut fell. Callers then work in ordinary planar parameter
//! space, and translate a query back with [`UnwrappedFaceDomain::images`].
//!
//! Nothing here is stored on the map: an unwrapped domain is derived from the
//! face every time it is needed, which is what lets the same face be read on a different
//! cut tomorrow.

use thiserror::Error;

use crate::geometry::parameter::Fraction;
use crate::geometry::{
    Axis2, DomainSide, LINEAR_TOLERANCE, Point2, Surface, SurfacePeriodicity, TrimmedCurve,
    TrimmedCurve2, Vector2,
};
use crate::topology::attributes::LoopKind;
use crate::topology::face::Face;
use crate::topology::face::Loop;
use crate::topology::gmap::Dart;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, FaceKey};

/// Failure while cutting a face's parameter domain open.
#[derive(Debug, Error, Clone, Copy, PartialEq, Eq)]
pub enum UnwrappedFaceDomainError {
    /// A boundary edge carries no pcurve on the face, so its loop has no
    /// parameter-space image to place.
    #[error("face {face:?} has no pcurve for boundary edge {edge:?}")]
    MissingPcurve { face: FaceKey, edge: EdgeKey },
}

/// One pcurve of a boundary loop, placed in an unwrapped domain.
///
/// The stored pcurve is kept as written on the face — it is the exact geometry
/// every intersection answers against — with the whole-period translation that
/// places it in the unwrapped domain carried alongside, so that neither can drift from
/// the other.
#[derive(Debug, Clone, PartialEq)]
pub struct UnwrappedFaceDomainCurve {
    /// The boundary dart of the edge this pcurve was read from.
    dart: Dart,
    curve: TrimmedCurve2,
    /// The edge this pcurve lies under, in 3D, traversed as the loop runs.
    span: TrimmedCurve,
    offset: Vector2,
    corners: Vec<Point2>,
}

impl UnwrappedFaceDomainCurve {
    /// Returns the boundary dart of the edge this pcurve was placed from.
    ///
    /// Carried rather than recovered by position: a loop is placed from
    /// whichever of its corners cuts the domain cleanly, so its curves need
    /// not come out in the order the face lists its edges.
    pub fn dart(&self) -> Dart {
        self.dart
    }

    /// Returns the pcurve as stored on the face, in its own branch.
    pub fn curve(&self) -> &TrimmedCurve2 {
        &self.curve
    }

    /// Returns the edge this pcurve lies under, in 3D, as the loop runs it.
    pub fn span(&self) -> &TrimmedCurve {
        &self.span
    }

    /// Returns the whole-period translation placing this pcurve in the unwrapped domain.
    pub fn offset(&self) -> Vector2 {
        self.offset
    }

    /// Returns the points the loop turns through before this pcurve, in order.
    ///
    /// Part of a face boundary can carry no pcurve at all, and the loop still
    /// travels it: a row of the domain that collapses to a single surface
    /// point — a sphere's pole — or the unwrapped domain's own cut, joining two
    /// wrapping loops that each close only on the quotient. The corners are
    /// where the loop turns in that gap; without them it never closes in the
    /// unwrapped domain. Crossing a degenerate row takes two of them — out to
    /// the row and back along it — which is why this is a list.
    pub fn corners(&self) -> &[Point2] {
        &self.corners
    }

    /// Evaluates this pcurve in the unwrapped domain, at a fraction of its span.
    pub fn point_at(&self, fraction: f64) -> Point2 {
        self.curve.point_at(Fraction::new(fraction)) + self.offset
    }

    /// Returns this pcurve's start in the unwrapped domain.
    pub fn start(&self) -> Point2 {
        self.point_at(0.0)
    }

    /// Returns this pcurve's end in the unwrapped domain.
    pub fn end(&self) -> Point2 {
        self.point_at(1.0)
    }
}

/// One boundary loop of a face, placed in an unwrapped domain.
#[derive(Debug, Clone, PartialEq)]
pub struct UnwrappedFaceDomainLoop {
    curves: Vec<UnwrappedFaceDomainCurve>,
}

impl UnwrappedFaceDomainLoop {
    /// Returns this loop's pcurves in boundary order.
    pub fn curves(&self) -> &[UnwrappedFaceDomainCurve] {
        &self.curves
    }

    /// Returns whether the loop has no pcurves at all.
    pub fn is_empty(&self) -> bool {
        self.curves.is_empty()
    }

    /// Flattens the loop into an unwrapped-domain polyline with `segments` points per pcurve.
    ///
    /// The polyline is cyclic: the final point does not repeat the first.
    pub fn polyline(&self, segments: usize) -> Vec<Point2> {
        self.flatten(|curve| curve.curve.sample(segments.max(1)))
    }

    /// Flattens the loop into an unwrapped-domain polyline through the points
    /// its edges are drawn at.
    ///
    /// Each edge is cut into as many uniform pieces as `segments` asks of it,
    /// in 3D, and each cut is found on the pcurve: near the same fraction of
    /// it, at the point `surface` puts closest to the cut. A pcurve need not
    /// share its edge's parameterization -- a straight line along a cylinder,
    /// uniform in angle, under a rational circle that is not -- so reading it
    /// at the edge's own fractions would land beside the edge, not on it. The
    /// search stays on the pcurve and in order along it, so the boundary keeps
    /// the shape the face was trimmed to.
    ///
    /// The polyline is cyclic: the final point does not repeat the first.
    pub fn polyline_on(
        &self,
        surface: &Surface,
        segments: impl Fn(&TrimmedCurve) -> usize,
    ) -> Vec<Point2> {
        self.flatten(|curve| {
            let count = segments(&curve.span).max(1);
            let step = 1.0 / count as f64;
            let mut floor = 0.0_f64;
            (0..=count)
                .map(|index| {
                    let target = curve.span.point_at(Fraction::new(index as f64 * step));
                    let distance = |fraction: f64| {
                        let uv = curve.curve.point_at(Fraction::new(fraction));
                        (surface.point_at(uv.x, uv.y) - target).norm()
                    };
                    let fraction = match index {
                        0 => 0.0,
                        _ if index == count => 1.0,
                        _ => nearest_on(
                            distance,
                            floor.max(index as f64 * step - step),
                            (index as f64 * step + step).min(1.0),
                        ),
                    };
                    floor = fraction;
                    curve.curve.point_at(Fraction::new(fraction))
                })
                .collect()
        })
    }

    /// Flattens the loop into an unwrapped-domain polyline within `chord` of the pcurves.
    ///
    /// The polyline is cyclic: the final point does not repeat the first.
    pub fn adaptive_polyline(&self, chord: f64, max_depth: usize) -> Vec<Point2> {
        self.flatten(|curve| {
            curve
                .curve
                .adaptive_samples(chord, max_depth)
                .into_iter()
                .map(|(_, point)| point)
                .collect()
        })
    }

    /// Walks the loop, dropping each pcurve's last sample and inserting the
    /// degenerate corners the loop turns through.
    fn flatten(&self, samples: impl Fn(&UnwrappedFaceDomainCurve) -> Vec<Point2>) -> Vec<Point2> {
        let mut polyline = Vec::new();
        for curve in &self.curves {
            polyline.extend(curve.corners.iter().copied());
            let mut points = samples(curve);
            // The last sample is the next pcurve's first, and the last pcurve
            // closes the loop back onto its start.
            points.pop();
            polyline.extend(points.into_iter().map(|point| point + curve.offset));
        }
        polyline
    }
}

/// A face's boundary loops placed in one simply-connected parameter domain.
#[derive(Debug, Clone, PartialEq)]
pub struct UnwrappedFaceDomain {
    loops: Vec<UnwrappedFaceDomainLoop>,
    periods: [Option<f64>; 2],
    cut: [Option<f64>; 2],
    min: Point2,
    max: Point2,
}

impl UnwrappedFaceDomain {
    /// How many points per pcurve size the unwrapped domain's extent.
    ///
    /// The extent sizes things — a flattening budget, a probe point — rather
    /// than bounding them, so a handful of samples per pcurve is enough.
    const EXTENT_SAMPLES: usize = 4;

    /// Cuts `face`'s parameter domain open and places its loops in the unwrapped domain.
    ///
    /// Wrapping loops are fused into the unwrapped domain's outer boundary: each runs one
    /// whole period and closes only on the quotient, so joining them across a
    /// synthesized cut is what makes the boundary a closed polygon again. That
    /// polygon is exactly the one a stored seam used to spell out.
    ///
    /// A lone capping loop is closed the same way against the degenerate row on
    /// its far side — the row a stored model would spell out as a pole vertex
    /// and a seam running up to it.
    pub fn of_face<P: Payload>(face: &Face<'_, P>) -> Result<Self, UnwrappedFaceDomainError> {
        let periods = periods_of(face.surface());
        let mut outer: Vec<UnwrappedFaceDomainCurve> = Vec::new();
        let mut outer_offset = Vector2::zeros();
        let mut holes: Vec<UnwrappedFaceDomainLoop> = Vec::new();
        let mut capped = None;
        let loops = face.loops();
        // Holes first: where they lie decides where a ring may be cut open.
        for loop_ in loops.iter().filter(|loop_| loop_.kind() == LoopKind::Inner) {
            let mut curves = Vec::new();
            let mut offset = Vector2::zeros();
            place_loop(face, loop_, periods, None, &mut curves, &mut offset)?;
            rebranch_across_degenerate_row(face.surface(), &mut curves, periods);
            close_loop(&mut curves);
            holes.push(UnwrappedFaceDomainLoop { curves });
        }
        let cut = ring_cut(face, &loops, &holes);
        if let Some(cut) = cut {
            for hole in &mut holes {
                cut.bring_inside(hole);
            }
        }
        for loop_ in &loops {
            match loop_.kind() {
                LoopKind::Wrapping { .. } => {
                    place_loop(face, loop_, periods, cut, &mut outer, &mut outer_offset)?;
                }
                LoopKind::Outer => {
                    place_loop(face, loop_, periods, None, &mut outer, &mut outer_offset)?;
                }
                LoopKind::Capping { axis, side } => {
                    place_loop(face, loop_, periods, None, &mut outer, &mut outer_offset)?;
                    capped = Some((axis, side));
                }
                LoopKind::Inner => {}
            }
        }
        match capped {
            Some((axis, side)) => close_capping_loop(face.surface(), axis, side, &mut outer),
            None => {
                rebranch_across_degenerate_row(face.surface(), &mut outer, periods);
                close_loop(&mut outer);
            }
        }

        let mut loops = Vec::with_capacity(1 + holes.len());
        loops.push(UnwrappedFaceDomainLoop { curves: outer });
        loops.extend(holes);

        let (min, max) = extent(&loops);
        let cut = [0, 1].map(|axis| periods[axis].map(|_| min[axis]));
        Ok(Self {
            loops,
            periods,
            cut,
            min,
            max,
        })
    }

    /// Returns the face's boundary loops, outer first, placed in the unwrapped domain.
    pub fn loops(&self) -> &[UnwrappedFaceDomainLoop] {
        &self.loops
    }

    /// Returns the support's period along `axis`, if it has one.
    pub fn period(&self, axis: Axis2) -> Option<f64> {
        self.periods[axis.index()]
    }

    /// Returns the support's periods, in parameter order.
    pub fn periods(&self) -> [Option<f64>; 2] {
        self.periods
    }

    /// Returns where this unwrapped domain cuts `axis` open, if that axis is periodic.
    ///
    /// The unwrapped domain spans `[cut, cut + period]` along a periodic axis, so the cut
    /// is the one parameter value the unwrapped domain does not cover twice. It is the
    /// synthesized seam: a fact about this reading of the face, not about the
    /// face.
    pub fn cut(&self, axis: Axis2) -> Option<f64> {
        self.cut[axis.index()]
    }

    /// Returns the corner bounds of the loops as placed in this unwrapped domain.
    pub fn bounds(&self) -> (Point2, Point2) {
        (self.min, self.max)
    }

    /// Returns the diagonal of [`Self::bounds`], or `0.0` with no extent.
    pub fn diagonal(&self) -> f64 {
        let diagonal = (self.max - self.min).norm();
        if diagonal.is_finite() { diagonal } else { 0.0 }
    }

    /// Returns `point` plus every whole-period translate of it the loops reach.
    ///
    /// A query lives on the surface, where a periodic parameter names the same
    /// point at either end of its period; the unwrapped domain wrote the loops on one
    /// branch. Asking the same question once per branch the loops could have
    /// been written on answers in the quotient without leaving planar
    /// arithmetic.
    ///
    /// A loop need not stay within one period. A band winding round a
    /// cylinder -- the strip a thread covers on its core -- is written as one
    /// long parallelogram running over several periods of `u`, one turn per
    /// period, and a point on its fifth turn is found only on the branch four
    /// periods along. So the translates run over the whole extent of the
    /// loops, and one period past it on either side, which is all a domain
    /// written within a single period ever needed.
    pub fn images(&self, point: Point2) -> Vec<Point2> {
        let mut images = vec![point];
        for (axis, period) in self.periods.iter().enumerate() {
            let Some(period) = *period else {
                continue;
            };
            let (first, last) = self.translates(point[axis], axis, period);
            let existing = images.clone();
            for turn in (first..=last).filter(|turn| *turn != 0) {
                images.extend(existing.iter().map(|image| {
                    let mut moved = *image;
                    moved[axis] += turn as f64 * period;
                    moved
                }));
            }
        }
        images
    }

    /// The whole-period shifts along `axis` that carry `value` into the loops'
    /// extent, widened by one either side and always including `-1..=1`.
    fn translates(&self, value: f64, axis: usize, period: f64) -> (i64, i64) {
        let (min, max) = (self.min[axis], self.max[axis]);
        if !(min.is_finite() && max.is_finite() && value.is_finite()) {
            return (-1, 1);
        }
        let first = ((min - value) / period).floor() as i64 - 1;
        let last = ((max - value) / period).ceil() as i64 + 1;
        (first.min(-1), last.max(1))
    }
}

/// Where `distance` is least on `[low, high]`, by golden-section search.
///
/// The window is one sample either side of where the point would be if the
/// pcurve shared its edge's parameterization, which is narrow enough to hold
/// one minimum.
fn nearest_on(distance: impl Fn(f64) -> f64, low: f64, high: f64) -> f64 {
    const RATIO: f64 = 0.618_033_988_749_894_9;
    const STEPS: usize = 40;
    let (mut low, mut high) = (low, high);
    let mut left = high - RATIO * (high - low);
    let mut right = low + RATIO * (high - low);
    let (mut at_left, mut at_right) = (distance(left), distance(right));
    for _ in 0..STEPS {
        if at_left <= at_right {
            high = right;
            right = left;
            at_right = at_left;
            left = high - RATIO * (high - low);
            at_left = distance(left);
        } else {
            low = left;
            left = right;
            at_left = at_right;
            right = low + RATIO * (high - low);
            at_right = distance(right);
        }
    }
    0.5 * (low + high)
}

/// Reads a support's periods as a parameter-indexed pair.
fn periods_of(surface: &Surface) -> [Option<f64>; 2] {
    match surface.periodicity() {
        SurfacePeriodicity::None => [None, None],
        SurfacePeriodicity::UPeriodic(period) => [Some(period), None],
        SurfacePeriodicity::VPeriodic(period) => [None, Some(period)],
        SurfacePeriodicity::UVPeriodic(u, v) => [Some(u), Some(v)],
    }
}

/// Places one boundary loop's pcurves onto the end of `curves`.
///
/// `offset` and `curves` carry across calls so that consecutive wrapping loops
/// are placed as one continuous walk rather than each from scratch.
fn place_loop<P: Payload>(
    face: &Face<'_, P>,
    loop_: &Loop<'_, P>,
    periods: [Option<f64>; 2],
    cut: Option<RingCut>,
    curves: &mut Vec<UnwrappedFaceDomainCurve>,
    offset: &mut Vector2,
) -> Result<(), UnwrappedFaceDomainError> {
    let mut edges = loop_.edges();
    let lone = edges.len() == 1;
    if let Some(cut) = cut.filter(|_| !lone) {
        let start = cut.start_index(face, &edges);
        edges.rotate_left(start);
    }
    for edge in edges {
        let curve =
            face.pcurve(edge.dart())
                .ok_or_else(|| UnwrappedFaceDomainError::MissingPcurve {
                    face: face.key(),
                    edge: edge.key(),
                })?;
        let span = edge.trimmed_curve();
        // A loop that is one closed edge has no corner to start from, so it
        // is read from the ring's cut instead of from where its pcurve
        // happens to begin.
        let (curve, span) = match cut.filter(|_| lone) {
            Some(cut) => cut.anchor(face.surface(), curve, span),
            None => (curve, span),
        };
        let corner = curves
            .last()
            .map(UnwrappedFaceDomainCurve::end)
            .and_then(|previous| {
                place_after(previous, curve.start(), face.surface(), periods, offset)
            });
        curves.push(UnwrappedFaceDomainCurve {
            dart: edge.dart(),
            curve,
            span,
            offset: *offset,
            corners: corner.into_iter().collect(),
        });
    }
    Ok(())
}

/// Where a ring face's two wrapping loops are joined across the unwrapped domain.
///
/// The domain closes a ring by running from the end of one wrapping loop to the
/// start of the next, and back. Those two runs are the cut, and they are only
/// straight across the ring when both loops start at the same place along the
/// axis they wrap. A loop whose first corner sits anywhere else turns the cut
/// into a slant, and a slant can run through a notch the other loop takes out
/// of the face -- the boundary then crosses itself and bounds nothing.
///
/// A loop with corners starts at one of them, and moving it would reorder its
/// edges. A loop that is one closed edge -- a rim nothing has cut -- starts
/// wherever its pcurve was written, and is free to start anywhere. So the cut
/// is taken at the first corner of a cornered loop, and every lone closed loop
/// is read from there.
#[derive(Debug, Clone, Copy)]
struct RingCut {
    axis: Axis2,
    at: f64,
    period: f64,
}

impl RingCut {
    /// Re-reads a lone closed loop's pcurve, and its edge, from the cut.
    ///
    /// Left as it was where the pcurve does not run once round the ring at a
    /// steady pace along the axis, since the place to start would then be a
    /// guess.
    fn anchor(
        self,
        surface: &Surface,
        curve: TrimmedCurve2,
        span: TrimmedCurve,
    ) -> (TrimmedCurve2, TrimmedCurve) {
        let axis = self.axis.index();
        let (start, end) = (curve.start()[axis], curve.end()[axis]);
        let run = end - start;
        if ((run.abs() - self.period).abs()) > LINEAR_TOLERANCE {
            return (curve, span);
        }
        let along = (self.at - start) / run;
        let along = along - along.floor();
        let at = curve.point_at(Fraction::new(along));
        let turns = (at[axis] - self.at) / self.period;
        if (turns - turns.round()).abs() * self.period > LINEAR_TOLERANCE {
            return (curve, span);
        }
        let reanchored = |interval: crate::geometry::Interval, fraction: f64| {
            let delta = interval.delta();
            let start = interval.start + fraction * delta;
            crate::geometry::Interval::new(start, start + delta)
        };
        let point = surface.point_at(at.x, at.y);
        let span_along = span.parameter_at(point).value();
        (
            TrimmedCurve2::new(curve.curve().clone(), reanchored(curve.interval(), along)),
            TrimmedCurve::new(
                span.curve().clone(),
                reanchored(span.interval(), span_along),
            ),
        )
    }
}

/// The cut a ring face's lone closed loops are read from, if the face is a ring.
///
/// A loop with corners fixes the cut at its first one. Where every wrapping
/// loop is one closed edge the cut is free, and it is kept off the holes: a
/// hole the cut runs through is split across the two ends of the unwrapped
/// domain and sticks out of it at one of them. The place the rims were
/// written from is kept while it is clear, and otherwise the middle of the
/// widest gap the holes leave round the ring is taken.
fn ring_cut<P: Payload>(
    face: &Face<'_, P>,
    loops: &[Loop<'_, P>],
    holes: &[UnwrappedFaceDomainLoop],
) -> Option<RingCut> {
    let periods = periods_of(face.surface());
    let wrapping = loops.iter().filter_map(|loop_| match loop_.kind() {
        LoopKind::Wrapping { axis } => Some((loop_, axis)),
        _ => None,
    });
    let mut lone = None;
    for (loop_, axis) in wrapping {
        let period = periods[axis.index()]?;
        let edges = loop_.edges();
        let first = face.pcurve(edges.first()?.dart())?;
        let at = first.start()[axis.index()].min(first.end()[axis.index()]);
        let cut = RingCut { axis, at, period };
        if edges.len() >= 2 {
            let corners = edges
                .iter()
                .filter_map(|edge| face.pcurve(edge.dart()))
                .map(|pcurve| pcurve.start())
                .collect::<Vec<_>>();
            let mut boundary = edges
                .iter()
                .filter_map(|edge| face.pcurve(edge.dart()))
                .map(|pcurve| pcurve.sample(RingCut::CUT_SAMPLES))
                .collect::<Vec<_>>();
            boundary.extend(holes.iter().map(|hole| {
                let mut points = hole.polyline(RingCut::CUT_SAMPLES);
                points.extend(points.first().copied());
                points
            }));
            // The cut runs from the corner across to the next wrapping loop,
            // which lies at one height across the ring.
            let toward = loops
                .iter()
                .filter(|other| {
                    !std::ptr::eq(*other, loop_)
                        && matches!(other.kind(), LoopKind::Wrapping { .. })
                })
                .find_map(|other| face.pcurve(other.edges().first()?.dart()))
                .map(|pcurve| pcurve.start()[axis.transverse().index()]);
            let corner = corners
                .iter()
                .copied()
                .find(|corner| cut.runs_clear(*corner, toward, &boundary))
                .unwrap_or(first.start());
            return Some(RingCut {
                at: corner[axis.index()],
                ..cut
            });
        }
        lone.get_or_insert(cut);
    }
    let cut = lone?;
    Some(RingCut {
        at: cut.clear_of(holes),
        ..cut
    })
}

impl RingCut {
    /// Samples per pcurve when checking where a cut may run.
    const CUT_SAMPLES: usize = 32;

    /// The wrapped offset of `value` from the cut, in `(-period / 2, period / 2]`.
    fn offset_of(self, value: f64) -> f64 {
        let offset = (value - self.at).rem_euclid(self.period);
        if offset > 0.5 * self.period {
            offset - self.period
        } else {
            offset
        }
    }

    /// Whether a cut straight across the ring from `corner` to the height
    /// `toward` meets the boundary only at `corner` itself.
    ///
    /// A loop's corners are not all equally good places to cut it open: a
    /// notch that overhangs one of them, or a hole standing over it, is run
    /// through by a cut from there, and the boundary then crosses itself.
    /// Only the stretch the cut covers counts; with no height to run to, the
    /// whole line does.
    fn runs_clear(self, corner: Point2, toward: Option<f64>, boundary: &[Vec<Point2>]) -> bool {
        let cut = RingCut {
            at: corner[self.axis.index()],
            ..self
        };
        let (along, across) = (self.axis.index(), self.axis.transverse().index());
        let off_cut = |point: Point2| {
            let near_corner = cut.offset_of(point[along]).abs() <= LINEAR_TOLERANCE
                && (point[across] - corner[across]).abs() <= LINEAR_TOLERANCE;
            let beyond = toward.is_some_and(|toward| {
                let (low, high) = if toward < corner[across] {
                    (toward, corner[across])
                } else {
                    (corner[across], toward)
                };
                point[across] < low - LINEAR_TOLERANCE || point[across] > high + LINEAR_TOLERANCE
            });
            near_corner || beyond
        };
        boundary.iter().all(|polyline| {
            polyline.windows(2).all(|pair| {
                let (p, q) = (pair[0], pair[1]);
                if (q[along] - p[along]).abs() > 0.5 * self.period {
                    return true;
                }
                let (fp, fq) = (cut.offset_of(p[along]), cut.offset_of(q[along]));
                if fp.abs() <= LINEAR_TOLERANCE {
                    return off_cut(p);
                }
                // Offsets that change sign across half a period have passed
                // the point opposite the cut, not the cut.
                if fp * fq >= 0.0
                    || fq.abs() <= LINEAR_TOLERANCE
                    || (fp - fq).abs() > 0.5 * self.period
                {
                    return true;
                }
                let t = fp / (fp - fq);
                off_cut(p + (q - p) * t)
            })
        })
    }

    /// Which of a cornered loop's edges starts at the cut, or the first.
    fn start_index<P: Payload>(
        self,
        face: &Face<'_, P>,
        edges: &[crate::topology::edge::Edge<'_, P>],
    ) -> usize {
        edges
            .iter()
            .position(|edge| {
                face.pcurve(edge.dart()).is_some_and(|pcurve| {
                    self.offset_of(pcurve.start()[self.axis.index()]).abs() <= LINEAR_TOLERANCE
                })
            })
            .unwrap_or(0)
    }

    /// Each hole's extent along the ring, as `(start, length)`.
    fn hole_spans(self, holes: &[UnwrappedFaceDomainLoop]) -> Vec<(f64, f64)> {
        let axis = self.axis.index();
        holes
            .iter()
            .filter_map(|hole| {
                let points = hole.polyline(Self::HOLE_SAMPLES);
                let low = points.iter().map(|point| point[axis]).reduce(f64::min)?;
                let high = points.iter().map(|point| point[axis]).reduce(f64::max)?;
                Some((low, high - low))
            })
            .collect()
    }

    const HOLE_SAMPLES: usize = 16;

    /// How far `at` is outside every hole along the ring, negative inside one.
    fn clearance(self, at: f64, spans: &[(f64, f64)]) -> f64 {
        spans
            .iter()
            .map(|&(start, length)| {
                let into = (at - start).rem_euclid(self.period);
                if into < length {
                    -(into.min(length - into))
                } else {
                    (into - length).min(self.period - into)
                }
            })
            .fold(f64::INFINITY, f64::min)
    }

    /// `self.at` if no hole covers it, else the middle of the widest gap.
    fn clear_of(self, holes: &[UnwrappedFaceDomainLoop]) -> f64 {
        let spans = self.hole_spans(holes);
        if spans.iter().any(|(_, length)| *length >= self.period)
            || self.clearance(self.at, &spans) > LINEAR_TOLERANCE
        {
            return self.at;
        }
        const CANDIDATES: usize = 720;
        (0..CANDIDATES)
            .map(|step| self.at + self.period * step as f64 / CANDIDATES as f64)
            .max_by(|a, b| {
                self.clearance(*a, &spans)
                    .total_cmp(&self.clearance(*b, &spans))
            })
            .unwrap_or(self.at)
    }

    /// Moves `hole` by whole periods to lie between the cut and one period on.
    fn bring_inside(self, hole: &mut UnwrappedFaceDomainLoop) {
        let Some(&(start, _)) = self.hole_spans(std::slice::from_ref(hole)).first() else {
            return;
        };
        let turns = ((start - self.at) / self.period).floor();
        if turns == 0.0 {
            return;
        }
        let mut shift = Vector2::zeros();
        shift[self.axis.index()] = -turns * self.period;
        slide_tail(&mut hole.curves, 0, shift);
        for corner in hole
            .curves
            .first_mut()
            .map(|curve| &mut curve.corners)
            .into_iter()
            .flatten()
        {
            *corner += shift;
        }
    }
}

/// Extends `offset` so a pcurve starting at `start` continues from `previous`.
///
/// The shift is chosen per pcurve rather than per sample because a pcurve is
/// continuous by construction, however far it travels: a cylinder wall's base
/// runs a whole period in one straight pcurve, and a per-sample rule would fold
/// that onto itself.
///
/// Returns the point the loop turns through when a gap remains after placement
/// — see [`UnwrappedFaceDomainCurve::corner`].
fn place_after(
    previous: Point2,
    start: Point2,
    surface: &Surface,
    periods: [Option<f64>; 2],
    offset: &mut Vector2,
) -> Option<Point2> {
    // A collapsed row is a gap in the loop, not a seam crossing: aligning
    // across it would translate the rest of the loop onto the wrong branch.
    if !(is_degenerate(surface, previous) && is_degenerate(surface, start + *offset)) {
        for (axis, period) in periods.iter().enumerate() {
            let Some(period) = *period else {
                continue;
            };
            let gap = start[axis] + offset[axis] - previous[axis];
            offset[axis] -= (gap / period).round() * period;
        }
    }
    (start + *offset != previous).then_some(previous)
}

/// Puts a loop that turned through a degenerate row back on the branch that
/// closes it.
///
/// [`place_after`] refuses to align across a collapsed row, and it is right to:
/// the row is one point on the surface, so the parameter it is written at says
/// nothing, and aligning to it would drag the rest of the loop onto whichever
/// branch that meaningless value happened to name. But refusing to align is not
/// the same as being placed correctly — it leaves whatever offset the walk had
/// accumulated before the row, and that offset was chosen for the pcurves
/// *before* the pole, not the ones after it.
///
/// A half-cap on a sphere is where the two come apart. Its rim runs the far half
/// of the domain, so the walk picks up a whole turn getting there; it climbs to
/// the pole, and the meridian back down — stored on the near half, needing no
/// turn at all — inherits that turn and lands a period away from where it
/// started. The loop comes back as a sheared parallelogram twice the width of
/// the cap, and the mesh drawn inside it hangs across the solid.
///
/// What the pole does not say, closure does: a loop written on one branch ends
/// where it began. So a residue of whole periods between the two is the turn
/// that was inherited by mistake, and taking it back off everything after the
/// last collapsed row is what puts the loop back on one branch.
///
/// Except when the residue is the cut. A sphere written with a seam runs up one
/// side of it and down the other, pole to pole, and those two sides *are* a
/// period apart — that is what makes the boundary enclose the whole domain
/// rather than nothing. Closure cannot tell the two apart, because both end a
/// period from where they began. What tells them apart is what the correction
/// would do: on the half cap it slides the return meridian back over the cap it
/// belongs to, and the region survives; on the seam it lands the two sides on
/// top of each other and the region collapses to a line. So the shift is made,
/// and kept only if the loop still bounds something.
fn rebranch_across_degenerate_row(
    surface: &Surface,
    curves: &mut [UnwrappedFaceDomainCurve],
    periods: [Option<f64>; 2],
) {
    let (Some(first), Some(last)) = (curves.first(), curves.last()) else {
        return;
    };
    let gap = first.start() - last.end();

    // Only a residue that is whole periods is this mistake. A loop that ends
    // somewhere else entirely is a different fact about the face, and shifting
    // it would hide that rather than fix it.
    let mut shift = Vector2::zeros();
    for (axis, period) in periods.iter().enumerate() {
        let Some(period) = period.filter(|period| *period > 0.0) else {
            continue;
        };
        let turns = (gap[axis] / period).round();
        if turns != 0.0 && (gap[axis] - turns * period).abs() <= LINEAR_TOLERANCE {
            shift[axis] = turns * period;
        }
    }
    if shift == Vector2::zeros() {
        return;
    }

    // The last collapsed row is the one that could have carried the turn; rows
    // before it were already answered for by the pcurves that followed them.
    let Some(crossing) = (1..curves.len()).rev().find(|index| {
        is_degenerate(surface, curves[index - 1].end())
            && is_degenerate(surface, curves[*index].start())
    }) else {
        return;
    };

    slide_tail(curves, crossing, shift);

    // The region has to survive the correction. Where it does not, the residue
    // was the cut rather than an inherited turn, and the loop was right as it
    // stood.
    if placed_area(curves).abs() <= LINEAR_TOLERANCE {
        slide_tail(curves, crossing, -shift);
    }
}

/// Moves `curves[from..]` by `shift`, corners included.
fn slide_tail(curves: &mut [UnwrappedFaceDomainCurve], from: usize, shift: Vector2) {
    for (index, curve) in curves.iter_mut().enumerate().skip(from) {
        curve.offset += shift;
        // The corner at the crossing itself is the *previous* pcurve's end,
        // which is staying put: the loop steps across the collapsed row there,
        // which costs nothing because the row is one point. Corners after it
        // belong to pcurves that moved, and move with them.
        if index > from {
            for corner in &mut curve.corners {
                *corner += shift;
            }
        }
    }
}

/// The signed area the placed pcurves enclose, sampled.
///
/// Only ever asked whether it is zero, so a handful of samples per pcurve is
/// enough: a loop folded onto itself encloses nothing at any resolution.
fn placed_area(curves: &[UnwrappedFaceDomainCurve]) -> f64 {
    const SAMPLES: usize = 8;
    let points = curves
        .iter()
        .flat_map(|curve| {
            (0..SAMPLES).map(move |step| curve.point_at(step as f64 / SAMPLES as f64))
        })
        .collect::<Vec<_>>();
    let mut area = 0.0;
    for (index, point) in points.iter().enumerate() {
        let next = points[(index + 1) % points.len()];
        area += point.x * next.y - next.x * point.y;
    }
    0.5 * area
}

/// Records the point a loop turns through as it closes back onto its start.
///
/// The corner belongs before the first pcurve, which is where the loop reaches
/// it after the last one.
fn close_loop(curves: &mut [UnwrappedFaceDomainCurve]) {
    let (Some(end), Some(start)) = (
        curves.last().map(UnwrappedFaceDomainCurve::end),
        curves.first().map(UnwrappedFaceDomainCurve::start),
    ) else {
        return;
    };
    if end != start {
        curves
            .first_mut()
            .expect("a loop with an end has a first curve")
            .corners = vec![end];
    }
}

/// Closes a lone period-spanning loop against the degenerate row that bounds it.
///
/// The loop leaves off one period along `axis` from where it started, and the
/// face runs from it to the collapsed row on the `side` of the transverse axis
/// the loop names. The boundary is completed by travelling out to that row,
/// along it for the period, and back — the path a stored model spells out as a
/// seam up to a pole vertex, the pole itself, and a seam back down.
///
/// The row's parameter comes from the surface, never from the loop: a side is
/// stable under every edit, a number would have to be kept in step with the
/// support.
fn close_capping_loop(
    surface: &Surface,
    axis: Axis2,
    side: DomainSide,
    curves: &mut [UnwrappedFaceDomainCurve],
) {
    let (Some(last), Some(first)) = (
        curves.last().map(UnwrappedFaceDomainCurve::end),
        curves.first().map(UnwrappedFaceDomainCurve::start),
    ) else {
        return;
    };
    let transverse = axis.transverse();
    let Some(row) = side.nearest(transverse.of(first), surface.degenerate_rows(transverse)) else {
        // The support names no collapse on that side, so there is no row to
        // close against and the loop is left open rather than closed against a
        // guess.
        return;
    };
    let onto_row = |point: Point2| {
        let mut point = point;
        point[transverse.index()] = row;
        point
    };
    // `last` is here because every pcurve drops its final sample, on the rule
    // that the next one starts there. A capping loop breaks that rule: what
    // follows it is the walk out to the collapsed row, so the point it left off
    // at has to be put back or the boundary cuts the corner.
    curves
        .first_mut()
        .expect("a loop with an end has a first curve")
        .corners = vec![last, onto_row(last), onto_row(first)];
}

/// Corner bounds of every loop as placed, sized from uniform pcurve samples.
fn extent(loops: &[UnwrappedFaceDomainLoop]) -> (Point2, Point2) {
    let mut min = Point2::new(f64::INFINITY, f64::INFINITY);
    let mut max = Point2::new(f64::NEG_INFINITY, f64::NEG_INFINITY);
    let mut grow = |point: Point2| {
        min = Point2::new(min.x.min(point.x), min.y.min(point.y));
        max = Point2::new(max.x.max(point.x), max.y.max(point.y));
    };
    for curve in loops.iter().flat_map(UnwrappedFaceDomainLoop::curves) {
        grow(curve.start());
        for step in 1..=UnwrappedFaceDomain::EXTENT_SAMPLES {
            grow(
                curve.point_at(f64::from(step as u32) / UnwrappedFaceDomain::EXTENT_SAMPLES as f64),
            );
        }
        for corner in &curve.corners {
            grow(*corner);
        }
    }
    if min.x.is_finite() {
        (min, max)
    } else {
        (Point2::origin(), Point2::origin())
    }
}

/// Whether the support collapses at a parameter-space point.
fn is_degenerate(surface: &Surface, point: Point2) -> bool {
    surface.is_degenerate_at(point.x, point.y)
}
