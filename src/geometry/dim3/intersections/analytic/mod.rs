//! Closed-form intersections for pairs of recognized analytic supports.
//!
//! The general solvers convert both operands to NURBS and search. That is the
//! right answer for free-form geometry and the wrong one for a plane and a
//! sphere, whose section is a circle obtainable in a few lines of algebra.
//! This module answers the pairs it recognizes exactly and declines everything
//! else, so the search stays the fallback rather than the only path.
//!
//! # The decline contract
//!
//! Every entry point returns `Option<Result<..>>`. `None` means "this pair is
//! not in the table, use the general solver"; `Ok(..Empty)` means "these
//! surfaces provably do not meet". Conflating the two would turn a gap in the
//! table into a wrong answer, so a case a table entry recognizes but cannot
//! *represent* -- a plane cutting a cone in a parabola, which [`Curve`] has no
//! variant for -- returns `None` as well.
//!
//! [`Curve`]: crate::geometry::Curve

mod curve_curve;
mod curve_surface;
mod roots;
mod surface_surface;

pub use curve_curve::intersect_analytic_curves;
pub use curve_surface::{intersect_analytic_curve_surface, line_surface_is_analytic};
pub use surface_surface::intersect_analytic_surfaces;

use crate::geometry::{Point3, TrimmedCurve, TrimmedCurve2};

/// How faithfully a section's pcurve represents it in a support's parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PcurveFidelity {
    /// The pcurve is the section, in closed form.
    Exact,
    /// Interpolated from the exact 3D curve; deviation is measured, not assumed.
    ///
    /// A circle on a sphere is transcendental in longitude and latitude, so no
    /// closed-form pcurve exists for it. The fit is a 1D interpolation of a
    /// curve already known exactly, inverted through the support's own
    /// closed-form projection -- not a search -- but it is still a fit, and
    /// `deviation` is the largest observed departure in model units.
    Fitted { deviation: f64 },
}

impl PcurveFidelity {
    /// Returns the measured deviation, zero when the pcurve is exact.
    pub fn deviation(self) -> f64 {
        match self {
            PcurveFidelity::Exact => 0.0,
            PcurveFidelity::Fitted { deviation } => deviation,
        }
    }

    /// Returns the weaker of two fidelities, keeping the larger deviation.
    pub fn combined(self, other: Self) -> Self {
        match (self, other) {
            (PcurveFidelity::Exact, PcurveFidelity::Exact) => PcurveFidelity::Exact,
            _ => PcurveFidelity::Fitted {
                deviation: self.deviation().max(other.deviation()),
            },
        }
    }
}

/// One section computed in closed form from the supports' own parameterizations.
///
/// The support retains its native parameterization; `curve` names the portion
/// of it this section covers, and its normalized traversal is the one the two
/// pcurves are synchronized with.
#[derive(Debug, Clone, PartialEq)]
pub struct AnalyticSection {
    pub curve: TrimmedCurve,
    pub pcurve_a: TrimmedCurve2,
    pub pcurve_b: TrimmedCurve2,
    pub fidelity: PcurveFidelity,
}

impl AnalyticSection {
    /// Evaluates the section at a normalized traversal parameter.
    pub fn point_at(&self, parameter: f64) -> Point3 {
        self.curve.point_at(parameter)
    }

    /// Projects a point to this section's normalized traversal parameter.
    pub fn parameter_at(&self, point: Point3) -> f64 {
        self.curve.parameter_at(point)
    }
}

/// A closed-form answer for one surface pair.
#[derive(Debug, Clone, PartialEq)]
pub enum AnalyticSurfaceIntersection {
    /// Proven disjoint -- not "nothing found".
    Empty,
    Sections(Vec<AnalyticSection>),
    /// The supports touch at one point and share no section of any length.
    ///
    /// This is the case the subdivision solver is worst at -- both hulls keep
    /// overlapping however far they are split -- and the one it is cheapest to
    /// answer exactly, so it gets a variant of its own rather than being
    /// rounded to [`Self::Empty`].
    TangentPoint(Point3),
    /// The supports are the same surface; the shared region is not resolved here.
    Coincident,
}

impl AnalyticSurfaceIntersection {
    /// Returns the sections, or an empty slice for the other outcomes.
    pub fn sections(&self) -> &[AnalyticSection] {
        match self {
            AnalyticSurfaceIntersection::Sections(sections) => sections,
            _ => &[],
        }
    }
}
