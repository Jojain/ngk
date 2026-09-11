mod analytic;
mod curve_curve;
mod curve_surface;
mod error;
mod options;
mod surface_surface;

use std::ops::Index;

use crate::geometry::{Interval, Point2, Point3, TrimmedCurve, TrimmedCurve2};

pub use analytic::{
    AnalyticSection, AnalyticSurfaceIntersection, PcurveFidelity, intersect_analytic_curve_surface,
    intersect_analytic_curves, intersect_analytic_surfaces, line_surface_is_analytic,
};
pub use curve_curve::{intersect_curves, intersect_curves_with_options};
pub use curve_surface::{
    PreparedCurve, PreparedSurface, intersect_curve_surface, intersect_curve_surface_with_options,
    intersect_prepared_curve_surface,
};
pub use error::IntersectionError;
pub use options::IntersectionOptions;
pub use surface_surface::{
    analytic_intersections as analytic_surface_intersections, intersect_prepared_surfaces,
    intersect_surfaces, intersect_surfaces_with_options,
};

/// Curve/curve observations plus an explicit coverage statement.
///
/// The subdivision search is bounded, so an empty result means "nothing was
/// found within the budget", not "the curves are disjoint". A caller that must
/// not miss an intersection reads [`Self::coverage`].
#[derive(Debug, Clone, PartialEq)]
pub struct CurveCurveIntersections {
    intersections: Vec<CurveCurveIntersection>,
    coverage: IntersectionCoverage,
}

impl CurveCurveIntersections {
    /// Creates a result set with its coverage status.
    pub fn new(intersections: Vec<CurveCurveIntersection>, coverage: IntersectionCoverage) -> Self {
        Self {
            intersections,
            coverage,
        }
    }

    /// Returns the ordered intersection observations.
    pub fn intersections(&self) -> &[CurveCurveIntersection] {
        &self.intersections
    }

    /// Returns the candidate-space coverage status.
    pub fn coverage(&self) -> &IntersectionCoverage {
        &self.coverage
    }

    /// Returns the number of intersection observations.
    pub fn len(&self) -> usize {
        self.intersections.len()
    }

    /// Returns whether no intersection observations were found.
    pub fn is_empty(&self) -> bool {
        self.intersections.is_empty()
    }

    /// Returns the observations as a slice.
    pub fn as_slice(&self) -> &[CurveCurveIntersection] {
        &self.intersections
    }

    /// Iterates over the observations by reference.
    pub fn iter(&self) -> std::slice::Iter<'_, CurveCurveIntersection> {
        self.intersections.iter()
    }
}

impl Index<usize> for CurveCurveIntersections {
    type Output = CurveCurveIntersection;

    fn index(&self, index: usize) -> &Self::Output {
        &self.intersections[index]
    }
}

impl IntoIterator for CurveCurveIntersections {
    type Item = CurveCurveIntersection;
    type IntoIter = std::vec::IntoIter<CurveCurveIntersection>;

    fn into_iter(self) -> Self::IntoIter {
        self.intersections.into_iter()
    }
}

impl<'a> IntoIterator for &'a CurveCurveIntersections {
    type Item = &'a CurveCurveIntersection;
    type IntoIter = std::slice::Iter<'a, CurveCurveIntersection>;

    fn into_iter(self) -> Self::IntoIter {
        self.intersections.iter()
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum CurveCurveIntersection {
    Point {
        point: Point3,
        u_a: f64,
        u_b: f64,
    },
    Overlap {
        interval_a: Interval,
        interval_b: Interval,
    },
}

#[derive(Debug, Clone, PartialEq)]
pub enum CurveSurfaceIntersection {
    Point {
        point: Point3,
        curve_u: f64,
        surface_u: f64,
        surface_v: f64,
    },
    Overlap {
        curve_interval: Interval,
    },
}

/// Curve/surface observations plus an explicit coverage statement.
///
/// Newton convergence on the observations found is not evidence that every
/// intersection was found, so callers that must not miss one have to read
/// [`Self::coverage`] rather than only the observation list.
#[derive(Debug, Clone, PartialEq)]
pub struct CurveSurfaceIntersections {
    intersections: Vec<CurveSurfaceIntersection>,
    coverage: IntersectionCoverage,
}

impl CurveSurfaceIntersections {
    /// Creates a result set with its coverage status.
    pub fn new(
        intersections: Vec<CurveSurfaceIntersection>,
        coverage: IntersectionCoverage,
    ) -> Self {
        Self {
            intersections,
            coverage,
        }
    }

    /// Returns the ordered intersection observations.
    pub fn intersections(&self) -> &[CurveSurfaceIntersection] {
        &self.intersections
    }

    /// Returns the candidate-space coverage status.
    pub fn coverage(&self) -> &IntersectionCoverage {
        &self.coverage
    }

    /// Returns the number of intersection observations.
    pub fn len(&self) -> usize {
        self.intersections.len()
    }

    /// Returns whether no intersection observations were found.
    pub fn is_empty(&self) -> bool {
        self.intersections.is_empty()
    }

    /// Returns the observations as a slice.
    pub fn as_slice(&self) -> &[CurveSurfaceIntersection] {
        &self.intersections
    }
}

impl Index<usize> for CurveSurfaceIntersections {
    type Output = CurveSurfaceIntersection;

    fn index(&self, index: usize) -> &Self::Output {
        &self.intersections[index]
    }
}

impl IntoIterator for CurveSurfaceIntersections {
    type Item = CurveSurfaceIntersection;
    type IntoIter = std::vec::IntoIter<CurveSurfaceIntersection>;

    fn into_iter(self) -> Self::IntoIter {
        self.intersections.into_iter()
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum SurfaceSurfaceIntersection {
    Point(SurfaceIntersectionPoint),
    Branch(SurfaceIntersectionBranch),
    OverlapCandidate(SurfaceOverlapCandidate),
}

/// A corrected point shared by both input surfaces.
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceIntersectionPoint {
    pub point: Point3,
    pub uv_a: Point2,
    pub uv_b: Point2,
    pub kind: SurfaceIntersectionPointKind,
    pub residual: f64,
}

/// The local contact classification at a corrected intersection point.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SurfaceIntersectionPointKind {
    Transverse,
    Tangent,
    Singular,
}

/// One connected, ordered surface/surface intersection branch.
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceIntersectionBranch {
    /// The 3D support curve and the span of it this branch covers, analytically
    /// simplified when requested and possible.
    ///
    /// Its normalized traversal is what the two pcurves are synchronized with:
    /// the same fraction of the section and of either pcurve is the same point.
    pub curve_3d: TrimmedCurve,
    /// The span of surface A's parameter space this branch traces.
    pub pcurve_a: TrimmedCurve2,
    /// The span of surface B's parameter space this branch traces.
    pub pcurve_b: TrimmedCurve2,
    pub samples: Vec<SurfaceIntersectionPoint>,
    pub closed: bool,
    pub kind: SurfaceIntersectionBranchKind,
    pub quality: IntersectionQuality,
}

impl SurfaceIntersectionBranch {
    /// Evaluates the 3D branch at a normalized traversal parameter.
    pub fn point_at(&self, parameter: f64) -> Point3 {
        self.curve_3d.point_at(parameter)
    }

    /// Projects a point to the branch's normalized traversal parameter.
    pub fn parameter_at(&self, point: Point3) -> f64 {
        self.curve_3d.parameter_at(point)
    }
}

/// Classification shared by the regular samples of a branch.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SurfaceIntersectionBranchKind {
    Transverse,
    Tangent,
    Singular,
}

/// Measured geometric quality of a fitted synchronized branch.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IntersectionQuality {
    pub max_residual: f64,
    pub max_fit_error: f64,
    pub certified: bool,
}

/// A possible two-dimensional common region requiring overlap resolution.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceOverlapCandidate {
    pub domain_a_u: Interval,
    pub domain_a_v: Interval,
    pub domain_b_u: Interval,
    pub domain_b_v: Interval,
}

/// Whether candidate-space coverage is complete for this result set.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum IntersectionCoverage {
    Complete,
    Incomplete(Vec<IntersectionIncompleteReason>),
}

/// Structured reasons why a result set cannot claim complete coverage.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IntersectionIncompleteReason {
    /// A candidate parameter box was left without a certificate that its
    /// intersection contains no closed loop, so a loop entirely inside both
    /// patches may be missing from the reported branches.
    LoopFreedomNotCertified,
    CoincidentRegionResolutionNotImplemented,
    TangentOrSingularContact,
    MinimumTraceStepReached,
    TraceBudgetExhausted,
    SynchronizedFitToleranceExceeded,
    UnsupportedControlPointWeights,
    /// Subdivision hit its depth limit with a candidate domain still unresolved,
    /// so roots inside that domain may be missing or merged.
    SubdivisionBudgetExhausted,
    /// A curve span stayed within tolerance of a non-planar patch. The reported
    /// interval is a candidate, not a certified overlap.
    UnresolvedOverlap,
    /// A candidate domain was isolated but Newton correction did not reach the
    /// residual tolerance, so its root is reported by no observation.
    RefinementFailed,
}

/// Surface/surface observations plus an explicit coverage statement.
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceSurfaceIntersections {
    intersections: Vec<SurfaceSurfaceIntersection>,
    coverage: IntersectionCoverage,
}

impl SurfaceSurfaceIntersections {
    /// Creates a result set with its coverage status.
    pub fn new(
        intersections: Vec<SurfaceSurfaceIntersection>,
        coverage: IntersectionCoverage,
    ) -> Self {
        Self {
            intersections,
            coverage,
        }
    }

    /// Returns the ordered intersection observations.
    pub fn intersections(&self) -> &[SurfaceSurfaceIntersection] {
        &self.intersections
    }

    /// Returns the candidate-space coverage status.
    pub fn coverage(&self) -> &IntersectionCoverage {
        &self.coverage
    }

    /// Returns the number of intersection observations.
    pub fn len(&self) -> usize {
        self.intersections.len()
    }

    /// Returns whether no intersection observations were found.
    pub fn is_empty(&self) -> bool {
        self.intersections.is_empty()
    }

    /// Returns the observations as a slice.
    pub fn as_slice(&self) -> &[SurfaceSurfaceIntersection] {
        &self.intersections
    }
}

impl Index<usize> for SurfaceSurfaceIntersections {
    type Output = SurfaceSurfaceIntersection;

    fn index(&self, index: usize) -> &Self::Output {
        &self.intersections[index]
    }
}
