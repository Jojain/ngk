mod curve_curve;
mod curve_surface;
mod error;
mod options;
mod surface_surface;

use std::ops::Index;

use crate::geometry::{Curve, Curve2, Interval, Point2, Point3};

pub use curve_curve::{intersect_curves, intersect_curves_with_options};
pub use curve_surface::{intersect_curve_surface, intersect_curve_surface_with_options};
pub use error::IntersectionError;
pub use options::IntersectionOptions;
pub use surface_surface::{intersect_surfaces, intersect_surfaces_with_options};

pub type CurveCurveIntersections = Vec<CurveCurveIntersection>;
pub type CurveSurfaceIntersections = Vec<CurveSurfaceIntersection>;
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
    /// The normalized 3D branch curve, analytically simplified when requested and possible.
    pub curve_3d: Curve,
    /// The normalized parameter-space curve on surface A.
    pub pcurve_a: Curve2,
    /// The normalized parameter-space curve on surface B.
    pub pcurve_b: Curve2,
    pub samples: Vec<SurfaceIntersectionPoint>,
    pub closed: bool,
    pub kind: SurfaceIntersectionBranchKind,
    pub quality: IntersectionQuality,
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
    Incomplete(Vec<SurfaceIntersectionIncompleteReason>),
}

/// Structured reasons why a result set cannot claim complete coverage.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SurfaceIntersectionIncompleteReason {
    InteriorLoopSearchNotImplemented,
    CoincidentRegionResolutionNotImplemented,
    TangentOrSingularContact,
    MinimumTraceStepReached,
    TraceBudgetExhausted,
    SynchronizedFitToleranceExceeded,
    UnsupportedControlPointWeights,
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
