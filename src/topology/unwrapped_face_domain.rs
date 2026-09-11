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

use crate::geometry::{
    Axis2, DomainEnd, Point2, Surface, SurfacePeriodicity, TrimmedCurve2, Vector2,
};
use crate::topology::attributes::LoopKind;
use crate::topology::face::Face;
use crate::topology::face::Loop;
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
    curve: TrimmedCurve2,
    offset: Vector2,
    corners: Vec<Point2>,
}

impl UnwrappedFaceDomainCurve {
    /// Returns the pcurve as stored on the face, in its own branch.
    pub fn curve(&self) -> &TrimmedCurve2 {
        &self.curve
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
        self.curve.point_at(fraction) + self.offset
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
        for loop_ in face.loops() {
            match loop_.kind() {
                LoopKind::Outer | LoopKind::Wrapping { .. } => {
                    place_loop(face, &loop_, periods, &mut outer, &mut outer_offset)?;
                }
                LoopKind::Capping { axis, end } => {
                    place_loop(face, &loop_, periods, &mut outer, &mut outer_offset)?;
                    capped = Some((axis, end));
                }
                LoopKind::Inner => {
                    let mut curves = Vec::new();
                    let mut offset = Vector2::zeros();
                    place_loop(face, &loop_, periods, &mut curves, &mut offset)?;
                    close_loop(&mut curves);
                    holes.push(UnwrappedFaceDomainLoop { curves });
                }
            }
        }
        match capped {
            Some((axis, end)) => close_capping_loop(face.surface(), axis, end, &mut outer),
            None => close_loop(&mut outer),
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

    /// Returns `point` plus every whole-period translate of it.
    ///
    /// A query lives on the surface, where a periodic parameter names the same
    /// point at either end of its period; the unwrapped domain wrote the loops on one
    /// branch. Asking the same question once per branch the loops could have
    /// been written on answers in the quotient without leaving planar
    /// arithmetic.
    pub fn images(&self, point: Point2) -> Vec<Point2> {
        let mut images = vec![point];
        for (axis, period) in self.periods.iter().enumerate() {
            let Some(period) = *period else {
                continue;
            };
            let existing = images.clone();
            for shift in [-period, period] {
                images.extend(existing.iter().map(|image| {
                    let mut moved = *image;
                    moved[axis] += shift;
                    moved
                }));
            }
        }
        images
    }
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
    curves: &mut Vec<UnwrappedFaceDomainCurve>,
    offset: &mut Vector2,
) -> Result<(), UnwrappedFaceDomainError> {
    for edge in loop_.edges() {
        let curve =
            face.pcurve(edge.dart())
                .ok_or_else(|| UnwrappedFaceDomainError::MissingPcurve {
                    face: face.key(),
                    edge: edge.key(),
                })?;
        let corner = curves
            .last()
            .map(UnwrappedFaceDomainCurve::end)
            .and_then(|previous| {
                place_after(previous, curve.start(), face.surface(), periods, offset)
            });
        curves.push(UnwrappedFaceDomainCurve {
            curve,
            offset: *offset,
            corners: corner.into_iter().collect(),
        });
    }
    Ok(())
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
/// face runs from it to the collapsed row at `end` of the transverse axis. The
/// boundary is completed by travelling out to that row, along it for the period,
/// and back — the path a stored model spells out as a seam up to a pole vertex,
/// the pole itself, and a seam back down.
fn close_capping_loop(
    surface: &Surface,
    axis: Axis2,
    end: DomainEnd,
    curves: &mut [UnwrappedFaceDomainCurve],
) {
    let (Some(last), Some(first)) = (
        curves.last().map(UnwrappedFaceDomainCurve::end),
        curves.first().map(UnwrappedFaceDomainCurve::start),
    ) else {
        return;
    };
    let transverse = axis.transverse();
    let (u, v) = surface.domain();
    let row = end.of(match transverse {
        Axis2::U => u,
        Axis2::V => v,
    });
    let onto_row = |point: Point2| {
        let mut point = point;
        point[transverse.index()] = row;
        point
    };
    curves
        .first_mut()
        .expect("a loop with an end has a first curve")
        .corners = vec![onto_row(last), onto_row(first)];
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
