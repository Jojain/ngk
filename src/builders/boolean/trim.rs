//! Trim-domain queries and exact pcurve crossings for Boolean branches.

use crate::geometry::TrimmedCurve2;
use crate::geometry::{
    CurveCurveIntersection2, CurveIntersectionError, CurveIntersectionOptions, IntersectionOptions,
    Interval, Point2,
};
use crate::topology::chart::{Chart, ChartCurve, ChartError};
use crate::topology::face::Face;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::EdgeKey;

use super::BooleanError;

/// Location of a parameter-space point relative to a trimmed face.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum TrimLocation {
    Inside { margin: f64 },
    Outside { margin: f64 },
    OnBoundary { loop_index: usize, parameter: f64 },
}

/// Relative chord budget used when flattening trim loops into winding polygons.
///
/// The polygons answer which side of the boundary a point falls on, never where
/// exactly the boundary runs -- that question is settled against the exact
/// pcurves. Scaling the flattening to the loops' own parameter extent keeps a
/// polygon a few hundred points wide at any model scale. Flattening to a
/// convergence tolerance such as `1e-10` instead asks a single circular pcurve
/// for roughly 157000 points, and every later query walks all of them.
const TRIM_CHORD_RATIO: f64 = 1.0e-4;

/// A face's trim, read on one synthesized chart cut.
///
/// The [`Chart`] places the loops; the polylines flattened from it answer
/// which side of the boundary a point falls on, and the loops' own pcurves
/// answer where the boundary exactly runs.
pub(crate) struct FaceTrimDomain {
    chart: Chart,
    polygons: Vec<Vec<Point2>>,
    tolerance: f64,
    /// Upper bound on how far `polygons` may stray from the chart's loops.
    chord: f64,
}

impl FaceTrimDomain {
    /// Reads all pcurves, including holes, without silently omitting missing geometry.
    pub(crate) fn new<P: Payload>(
        face: &Face<'_, P>,
        tolerance: f64,
    ) -> Result<Self, BooleanError> {
        super::diagnostics::count_trim_domain_built();
        let chart = Chart::of_face(face).map_err(|error| match error {
            ChartError::MissingPcurve { face, edge } => {
                BooleanError::MissingTrimCurve { face, edge }
            }
        })?;
        let chord = chord_budget(&chart, tolerance);
        let polygons = chart
            .loops()
            .iter()
            .map(|boundary| boundary.adaptive_polyline(chord, 20))
            .collect();
        Ok(Self {
            chart,
            polygons,
            tolerance,
            chord,
        })
    }

    /// Every pcurve of the face's trim, as stored.
    fn trim_curves(&self) -> impl Iterator<Item = &TrimmedCurve2> {
        self.chart
            .loops()
            .iter()
            .flat_map(|boundary| boundary.curves())
            .map(ChartCurve::curve)
    }

    /// How close to the boundary [`Self::boundary_distance`] stops discriminating.
    ///
    /// A caller rejecting boundary-grazing queries must compare against this
    /// rather than against its own tolerance: the polygons locate the boundary
    /// only to their own chord budget, so a smaller threshold would accept
    /// queries the polygons cannot actually place.
    pub(crate) fn boundary_epsilon(&self) -> f64 {
        self.chord.max(self.tolerance)
    }

    /// Center of the active parameter-space image of this face's outer trim.
    pub(crate) fn chart_center(&self) -> Point2 {
        let (min, max) = self.chart_bounds();
        Point2::from((min.coords + max.coords) * 0.5)
    }

    /// Corner bounds of the active parameter-space image of the outer trim.
    pub(crate) fn chart_bounds(&self) -> (Point2, Point2) {
        let Some(outer) = self.polygons.first() else {
            return (Point2::origin(), Point2::origin());
        };
        let mut min = Point2::new(f64::INFINITY, f64::INFINITY);
        let mut max = Point2::new(f64::NEG_INFINITY, f64::NEG_INFINITY);
        for point in outer {
            min = Point2::new(min.x.min(point.x), min.y.min(point.y));
            max = Point2::new(max.x.max(point.x), max.y.max(point.y));
        }
        (min, max)
    }

    /// Distance to the nearest polyline segment; exact for admitted polygonal faces.
    ///
    /// Measured in the periodic quotient, so a query a hair past the seam is
    /// as close to a loop written at the other end of the period as it looks
    /// on the surface.
    pub(crate) fn boundary_distance(&self, point: Point2) -> f64 {
        self.chart
            .images(point)
            .into_iter()
            .map(|image| self.planar_boundary_distance(image))
            .fold(f64::INFINITY, f64::min)
    }

    /// Distance to the nearest polyline segment, on one branch only.
    fn planar_boundary_distance(&self, point: Point2) -> f64 {
        self.polygons
            .iter()
            .flat_map(|polygon| {
                polygon
                    .iter()
                    .zip(polygon.iter().cycle().skip(1))
                    .take(polygon.len())
            })
            .map(|(a, b)| {
                let direction = b - a;
                let t = if direction.norm_squared() == 0.0 {
                    0.0
                } else {
                    ((point - a).dot(&direction) / direction.norm_squared()).clamp(0.0, 1.0)
                };
                (point - (a + direction * t)).norm()
            })
            .fold(f64::INFINITY, f64::min)
    }
    /// Classifies a point, refining boundary proximity against the exact pcurves.
    ///
    /// A periodic support is classified in its quotient: the query is asked on
    /// every branch it could occupy, and any branch that places it on or
    /// inside the trim answers for all of them, because they are one point on
    /// the surface.
    pub(crate) fn classify(&self, point: Point2) -> TrimLocation {
        let images = self.chart.images(point);
        for image in &images {
            for (loop_index, boundary) in self.chart.loops().iter().enumerate() {
                for curve in boundary.curves() {
                    if let Some(parameter) = curve.curve().try_parameter_at(*image, self.tolerance)
                    {
                        return TrimLocation::OnBoundary {
                            loop_index,
                            parameter,
                        };
                    }
                }
            }
        }
        let margin = self.boundary_distance(point);
        let inside = images.iter().any(|image| self.planar_contains(*image));
        if inside {
            TrimLocation::Inside { margin }
        } else {
            TrimLocation::Outside { margin }
        }
    }

    /// Winding membership against the polygons as written, on one branch.
    fn planar_contains(&self, point: Point2) -> bool {
        self.polygons
            .first()
            .is_some_and(|outer| winding_contains(outer, point))
            && self
                .polygons
                .iter()
                .skip(1)
                .all(|hole| !winding_contains(hole, point))
    }

    /// Strict interior membership; exact curve proximity excludes the boundary.
    pub(crate) fn contains(&self, point: Point2) -> bool {
        matches!(self.classify(point), TrimLocation::Inside { .. })
    }

    /// Intervals of an unbounded UV line whose midpoints lie inside this domain.
    ///
    /// The line is bounded over the adaptive trim extent, then intersected with
    /// the exact pcurves. The adaptive points only establish a conservative
    /// finite parameter range; they do not define crossing positions.
    pub(crate) fn line_intervals(
        &self,
        origin: Point2,
        direction: nalgebra::Vector2<f64>,
        options: IntersectionOptions,
    ) -> Result<Vec<Interval>, CurveIntersectionError> {
        let direction_norm = direction.norm();
        if direction_norm <= self.tolerance {
            return Ok(Vec::new());
        }
        let mut projected = self
            .polygons
            .iter()
            .flatten()
            .map(|point| (point - origin).dot(&direction) / direction.norm_squared());
        let Some(first) = projected.next() else {
            return Ok(Vec::new());
        };
        let (mut start, mut end) = (first, first);
        for parameter in projected {
            start = start.min(parameter);
            end = end.max(parameter);
        }
        let parameter_tolerance = self.tolerance / direction_norm;
        let padding =
            2.0 * parameter_tolerance + 64.0 * f64::EPSILON * start.abs().max(end.abs()).max(1.0);
        start -= padding;
        end += padding;

        let bounded_line =
            TrimmedCurve2::segment(origin + direction * start, origin + direction * end);
        let curve_options = CurveIntersectionOptions {
            linear_tolerance: options.parameter_tolerance,
            parameter_tolerance: options.parameter_tolerance,
            bbox_tolerance: options.parameter_tolerance,
            max_subdivision_depth: options.max_subdivision_depth,
            leaf_diagonal_tolerance: options.parameter_tolerance * 10.0,
            newton_max_iterations: options.newton_max_iterations,
        };
        let mut normalized = Vec::new();
        self.crossings(&bounded_line, curve_options, &mut normalized)?;
        let mut parameters = normalized
            .into_iter()
            .map(|parameter| start + parameter * (end - start))
            .collect::<Vec<_>>();
        parameters.sort_by(f64::total_cmp);
        parameters.dedup_by(|a, b| (*a - *b).abs() <= parameter_tolerance);
        Ok(parameters
            .windows(2)
            .filter_map(|pair| {
                let midpoint = 0.5 * (pair[0] + pair[1]);
                self.contains(origin + direction * midpoint)
                    .then_some(Interval::new(pair[0], pair[1]))
            })
            .collect())
    }

    /// Adds every exact trim crossing in the branch's normalized parameter space.
    pub(crate) fn crossings(
        &self,
        span: &TrimmedCurve2,
        options: CurveIntersectionOptions,
        parameters: &mut Vec<f64>,
    ) -> Result<(), CurveIntersectionError> {
        for boundary in self.trim_curves() {
            for contact in span.intersect_curve_with_options(boundary, options)? {
                match contact {
                    CurveCurveIntersection2::Point { u_a, .. } => {
                        parameters.push(u_a.clamp(0.0, 1.0))
                    }
                    CurveCurveIntersection2::Overlap { interval_a, .. } => {
                        parameters.extend([
                            interval_a.start.clamp(0.0, 1.0),
                            interval_a.end.clamp(0.0, 1.0),
                        ]);
                    }
                }
            }
        }
        Ok(())
    }
}

/// Chord budget for flattening, scaled to the extent the chart's loops span.
///
/// The chart's extent only needs to size the domain, not to bound it: the
/// budget sets the polygons' resolution, and [`FaceTrimDomain::boundary_epsilon`]
/// reports it so no caller reads the polygons finer than they were built. A
/// domain with no measurable extent falls back to `floor`.
fn chord_budget(chart: &Chart, floor: f64) -> f64 {
    let diagonal = chart.diagonal();
    if diagonal > 0.0 {
        (diagonal * TRIM_CHORD_RATIO).max(floor)
    } else {
        floor
    }
}

/// Even/odd winding against an adaptively sampled loop.
fn winding_contains(polygon: &[Point2], point: Point2) -> bool {
    let mut inside = false;
    for (a, b) in polygon
        .iter()
        .zip(polygon.iter().cycle().skip(1))
        .take(polygon.len())
    {
        if (a.y > point.y) != (b.y > point.y)
            && point.x < (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x
        {
            inside = !inside;
        }
    }
    inside
}

/// Locates the boundary edge of `face` that already realizes `pcurve`.
///
/// A contact section running along a face's own trim loop must never be
/// imprinted: the edge already carries that geometry, and splitting the face
/// along its own loop yields a degenerate fragment with no interior probe.
/// Such a section is realized on the existing edge instead.
pub(crate) fn boundary_edge_for<P: Payload>(
    face: &Face<'_, P>,
    pcurve: &TrimmedCurve2,
    tolerance: f64,
) -> Option<EdgeKey> {
    let samples = [0.0, 0.25, 0.5, 0.75, 1.0].map(|t| pcurve.point_at(t));
    face.edges()
        .into_iter()
        .find(|edge| {
            face.pcurve(edge.dart()).is_some_and(|boundary| {
                samples
                    .iter()
                    .all(|point| boundary.try_parameter_at(*point, tolerance).is_some())
            })
        })
        .map(|edge| edge.key())
}
