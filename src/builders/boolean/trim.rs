//! Trim-domain queries and exact pcurve crossings for Boolean branches.

use crate::geometry::{
    Curve2, CurveCurveIntersection2, CurveIntersectionError, CurveIntersectionOptions, Point2,
};
use crate::topology::face::Face;
use crate::topology::payload::Payload;

use super::BooleanError;

/// Cached oriented loops; polylines are used only for winding classification.
pub(crate) struct FaceTrimDomain {
    loops: Vec<Vec<Curve2>>,
    polygons: Vec<Vec<Point2>>,
    tolerance: f64,
}

impl FaceTrimDomain {
    /// Reads all pcurves, including holes, without silently omitting missing geometry.
    pub(crate) fn new<P: Payload>(
        face: &Face<'_, P>,
        tolerance: f64,
    ) -> Result<Self, BooleanError> {
        let mut loops = Vec::new();
        for boundary in face.loops() {
            let mut curves = Vec::new();
            for edge in boundary.edges() {
                curves.push(
                    face.pcurve(edge.dart())
                        .ok_or(BooleanError::MissingTrimCurve {
                            face: face.key(),
                            edge: edge.key(),
                        })?,
                );
            }
            loops.push(curves);
        }
        let polygons = loops
            .iter()
            .map(|curves| {
                curves
                    .iter()
                    .flat_map(|curve| {
                        let mut points = curve.adaptive_samples(tolerance, 20);
                        points.pop();
                        points.into_iter().map(|(_, point)| point)
                    })
                    .collect()
            })
            .collect();
        Ok(Self {
            loops,
            polygons,
            tolerance,
        })
    }

    /// Whether the winding polyline is exact (piecewise linear pcurves only).
    pub(crate) fn is_polygonal(&self) -> bool {
        self.loops.iter().flatten().all(|curve| {
            curve
                .to_nurbs()
                .is_ok_and(|curve| curve.degree().get() == 1)
        })
    }

    /// Distance to the nearest polyline segment; exact for admitted polygonal faces.
    pub(crate) fn boundary_distance(&self, point: Point2) -> f64 {
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
    /// Strict interior membership; exact curve proximity excludes the boundary.
    pub(crate) fn contains(&self, point: Point2) -> bool {
        if self
            .loops
            .iter()
            .flatten()
            .any(|curve| curve.parameter_at(point, self.tolerance).is_some())
        {
            return false;
        }
        self.polygons
            .first()
            .is_some_and(|outer| winding_contains(outer, point))
            && self
                .polygons
                .iter()
                .skip(1)
                .all(|hole| !winding_contains(hole, point))
    }

    /// Adds every exact trim crossing in the branch's normalized parameter space.
    pub(crate) fn crossings(
        &self,
        curve: &Curve2,
        options: CurveIntersectionOptions,
        parameters: &mut Vec<f64>,
    ) -> Result<(), CurveIntersectionError> {
        for boundary in self.loops.iter().flatten() {
            for contact in curve.intersect_curve_with_options(boundary, options)? {
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
