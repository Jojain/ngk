//! The general row: a cross-section solved along the edge and skinned.
//!
//! Any two faces and any edge curve end up here when no closed form answers.
//! The cross-section is solved at evenly spread fractions of the edge — the
//! arc of the ball touching both faces for a fillet, the segment between the
//! two setback points for a chamfer — and the blend surface is skinned
//! through them: `u` runs along the edge over `[0, 1]`, `v` across the section
//! from side 0 (`v = 0`) to side 1 (`v = 1`).
//!
//! A fillet's section is written as the exact rational quadratic arc, so the
//! surface reproduces every sampled arc. Each row of control points is joined
//! along the edge by quintic Hermite segments matching the row's value and
//! first two derivatives at every sample, the derivatives differenced over
//! sections solved just either side of it. Matching curvature as well as
//! slope is what lets a handful of spans follow a blend round a whole turn:
//! every span a skin has is a span each later reader of it pays for.
//!
//! The sampling doubles until the section solved half way between two
//! samples lies on the skin, so the rails — the skin's `v = 0` and `v = 1`
//! isolines — stay on their faces between the samples as well as at them.
//! A closed edge gives a closed skin: its last sample is its first, so the
//! blend closes on itself along `u` with no seam in the geometry.

use nalgebra::Vector4;

use super::super::errors::BlendError;
use super::super::law::{BlendLaw, ChamferLaw, FilletLaw};
use super::contact::{BallSection, ball_at, setbacks_at};
use super::crease::{Crease, Foot};
use super::edge_section::{EdgeSection, SectionForm, SectionShape};
use crate::geometry::parameter::Fraction;
use crate::geometry::{
    ControlNet, Curve, Degree, HPoint, IntersectionOptions, KnotVector, LINEAR_TOLERANCE,
    NurbsError, NurbsSurface, Point3, PointCoincidence, Surface,
};

/// Spans the first skin is tried with.
const FIRST_SEGMENTS: usize = 4;

/// Spans past which a skin that still strays is refused.
const MAX_SEGMENTS: usize = 512;

/// The degree of the skin along the edge: a quintic matches value, slope and
/// curvature at both ends of every span.
const DEGREE_ALONG: usize = 5;

/// The share of the fitting tolerance the skin may stray from a section
/// solved between its samples.
///
/// A rail that strays from its face by this much still leaves room, inside
/// the fitting tolerance, for the pcurve fitted to it there.
const SKIN_SHARE: f64 = 0.25;

/// The fraction of the edge a row's derivatives are differenced over.
///
/// Wide enough that the sections' own residuals do not swamp a second
/// difference, narrow enough that five-point stencils are exact to far below
/// the skin's tolerance.
const DIFFERENCE_STEP: f64 = 1.0e-3;

/// Skins a general blend of the crease under `law`.
pub(super) fn swept_section(
    crease: &Crease<'_>,
    law: BlendLaw,
    convex: bool,
) -> Result<EdgeSection, BlendError> {
    let closed = crease
        .span
        .start()
        .coincides(crease.span.end(), LINEAR_TOLERANCE);
    let solver = SectionSolver {
        crease,
        law,
        convex,
        closed,
    };
    let mut segments = FIRST_SEGMENTS;
    loop {
        let skin = solver.skin(segments)?;
        if solver.fits(&skin, segments)? {
            return solver.finish(skin);
        }
        segments *= 2;
        if segments > MAX_SEGMENTS {
            return Err(BlendError::EdgeDoesNotFit {
                edge: crease.key,
                reason: "its blend bends too sharply to follow",
            });
        }
    }
}

/// Solves cross-sections of one crease under one law.
struct SectionSolver<'a> {
    crease: &'a Crease<'a>,
    law: BlendLaw,
    convex: bool,
    closed: bool,
}

/// One solved cross-section: its control points across the blend, as
/// homogeneous coordinates, and what the next section can start from.
#[derive(Debug, Clone, Copy)]
enum Section {
    Arc {
        ball: BallSection,
        row: [Vector4<f64>; 3],
    },
    Segment {
        feet: [Foot; 2],
        row: [Vector4<f64>; 2],
    },
}

/// A row of control points across the blend, with its first and second
/// derivatives along the edge.
struct Jet {
    value: Vec<Vector4<f64>>,
    slope: Vec<Vector4<f64>>,
    curvature: Vec<Vector4<f64>>,
}

impl Section {
    fn row(&self) -> &[Vector4<f64>] {
        match self {
            Self::Arc { row, .. } => row,
            Self::Segment { row, .. } => row,
        }
    }

    /// Where the section touches each face.
    fn contacts(&self) -> [Point3; 2] {
        match self {
            Self::Arc { ball, .. } => ball.contacts.map(|foot| foot.point),
            Self::Segment { feet, .. } => feet.map(|foot| foot.point),
        }
    }
}

impl SectionSolver<'_> {
    fn unsolved(&self) -> BlendError {
        BlendError::EdgeDoesNotFit {
            edge: self.crease.key,
            reason: "no blend touches both of its faces all along it",
        }
    }

    /// The cross-section at fraction `t`, searching from `near` when a section
    /// is known a short step away, and from the crease itself otherwise.
    fn section(&self, t: f64, near: Option<&Section>) -> Result<Section, BlendError> {
        let t = Fraction::new(if self.closed { t.rem_euclid(1.0) } else { t });
        match self.law {
            BlendLaw::Fillet(FilletLaw::Radius(radius)) => {
                let guess = match near {
                    Some(Section::Arc { ball, .. }) => Some(ball),
                    _ => None,
                };
                let ball = ball_at(self.crease, t, radius, self.convex, guess)
                    .ok_or_else(|| self.unsolved())?;
                Ok(Section::Arc {
                    ball,
                    row: arc_row(&ball, radius).ok_or_else(|| self.unsolved())?,
                })
            }
            BlendLaw::Chamfer(ChamferLaw::Distance(distance)) => {
                let guess = match near {
                    Some(Section::Segment { feet, .. }) => Some(feet),
                    _ => None,
                };
                let feet =
                    setbacks_at(self.crease, t, distance, guess).ok_or_else(|| self.unsolved())?;
                Ok(Section::Segment {
                    feet,
                    row: feet.map(|foot| HPoint::from_cartesian(foot.point, 1.0).0.coords),
                })
            }
        }
    }

    /// The skin through `segments + 1` evenly spread sections.
    fn skin(&self, segments: usize) -> Result<NurbsSurface, BlendError> {
        let parameters = (0..=segments)
            .map(|index| index as f64 / segments as f64)
            .collect::<Vec<_>>();
        let mut jets: Vec<Jet> = Vec::with_capacity(segments + 1);
        for (index, &t) in parameters.iter().enumerate() {
            if self.closed && index == segments {
                let first = &jets[0];
                jets.push(Jet {
                    value: first.value.clone(),
                    slope: first.slope.clone(),
                    curvature: first.curvature.clone(),
                });
                continue;
            }
            let section = self.section(t, None)?;
            jets.push(self.jet(t, &section)?);
        }

        let across = jets[0].value.len();
        let rows = (0..across)
            .map(|j| quintic_row(&jets, j, &parameters))
            .collect::<Vec<_>>();
        let along = rows[0].len();
        let mut points = Vec::with_capacity(along * across);
        for row in &rows {
            points.extend(row.iter().copied());
        }
        let build = || -> Result<NurbsSurface, NurbsError> {
            let knots_v = KnotVector::new(
                std::iter::repeat_n(0.0, across)
                    .chain(std::iter::repeat_n(1.0, across))
                    .collect(),
            )?;
            NurbsSurface::new(
                Degree::new(DEGREE_ALONG)?,
                Degree::new(across - 1)?,
                ControlNet::new(points, along, across)?,
                quintic_knots(&parameters)?,
                knots_v,
            )
        };
        build().map_err(|_| self.unsolved())
    }

    /// A section's row and its first two derivatives along the edge,
    /// differenced over sections solved either side of it.
    ///
    /// Five-point stencils throughout: centred where the edge runs on past
    /// both sides — always on a closed edge — and one-sided at the ends of a
    /// bounded one, which has no section beyond them.
    fn jet(&self, t: f64, section: &Section) -> Result<Jet, BlendError> {
        let h = DIFFERENCE_STEP;
        let rows = |offsets: [f64; 4]| -> Result<Vec<Vec<Vector4<f64>>>, BlendError> {
            offsets
                .iter()
                .map(|&offset| Ok(self.section(t + offset * h, Some(section))?.row().to_vec()))
                .collect()
        };
        let here = section.row().to_vec();
        let combine = |samples: &[&Vec<Vector4<f64>>], weights: [f64; 5], scale: f64| {
            (0..here.len())
                .map(|j| {
                    samples
                        .iter()
                        .zip(weights)
                        .map(|(row, weight)| row[j] * weight)
                        .sum::<Vector4<f64>>()
                        * scale
                })
                .collect::<Vec<_>>()
        };
        let centred = self.closed || (2.0 * h..=1.0 - 2.0 * h).contains(&t);
        let (samples, slope, curvature, direction) = if centred {
            let around = rows([-2.0, -1.0, 1.0, 2.0])?;
            let samples =
                [&around[0], &around[1], &here, &around[2], &around[3]].map(|row| row.clone());
            (
                samples,
                [1.0, -8.0, 0.0, 8.0, -1.0],
                [-1.0, 16.0, -30.0, 16.0, -1.0],
                1.0,
            )
        } else {
            let direction = if t < 0.5 { 1.0 } else { -1.0 };
            let ahead = rows([1.0, 2.0, 3.0, 4.0].map(|step| step * direction))?;
            let samples =
                [&here, &ahead[0], &ahead[1], &ahead[2], &ahead[3]].map(|row| row.clone());
            (
                samples,
                [-25.0, 48.0, -36.0, 16.0, -3.0],
                [35.0, -104.0, 114.0, -56.0, 11.0],
                direction,
            )
        };
        let samples = samples.iter().collect::<Vec<_>>();
        Ok(Jet {
            slope: combine(&samples, slope, direction / (12.0 * h)),
            curvature: combine(&samples, curvature, 1.0 / (12.0 * h * h)),
            value: here.clone(),
        })
    }

    /// Whether the skin through `segments` sections holds the section solved
    /// half way between each pair of them.
    fn fits(&self, skin: &NurbsSurface, segments: usize) -> Result<bool, BlendError> {
        let tolerance = IntersectionOptions::default().fit_tolerance * SKIN_SHARE;
        for index in 0..segments {
            let t = (index as f64 + 0.5) / segments as f64;
            let section = self.section(t, None)?;
            let contacts = section.contacts();
            let rails = [skin.point_at(t, 0.0), skin.point_at(t, 1.0)];
            if (0..2).any(|side| (rails[side] - contacts[side]).norm() > tolerance) {
                return Ok(false);
            }
        }
        Ok(true)
    }

    /// The section a finished skin makes.
    fn finish(&self, skin: NurbsSurface) -> Result<EdgeSection, BlendError> {
        let rails = [0.0, 1.0].map(|v| skin.isocurve_v(v).map(Curve::Nurbs));
        let [Ok(first), Ok(second)] = rails else {
            return Err(self.unsolved());
        };
        Ok(EdgeSection {
            surface: Surface::Nurbs(skin),
            rails: [first, second],
            convex: self.convex,
            form: SectionForm::Swept {
                shape: match self.law {
                    BlendLaw::Fillet(FilletLaw::Radius(radius)) => SectionShape::Arc { radius },
                    BlendLaw::Chamfer(_) => SectionShape::Segment,
                },
            },
        })
    }
}

/// The rational quadratic arc of a ball from its first contact to its
/// second, the short way round, as homogeneous control points.
fn arc_row(ball: &BallSection, radius: f64) -> Option<[Vector4<f64>; 3]> {
    let [first, second] = ball.contacts.map(|foot| foot.point - ball.centre);
    let cosine = (first.dot(&second) / (first.norm() * second.norm())).clamp(-1.0, 1.0);
    let half = cosine.acos() / 2.0;
    let weight = half.cos();
    let bisector = first + second;
    if weight <= LINEAR_TOLERANCE.sqrt() || bisector.norm() <= LINEAR_TOLERANCE {
        return None;
    }
    let middle = ball.centre + bisector.normalize() * (radius / weight);
    Some([
        HPoint::from_cartesian(ball.contacts[0].point, 1.0).0.coords,
        HPoint::from_cartesian(middle, weight).0.coords,
        HPoint::from_cartesian(ball.contacts[1].point, 1.0).0.coords,
    ])
}

/// Quintic Hermite control points of row `j` through `jets` at
/// `parameters`: one Bezier segment between each pair of samples.
fn quintic_row(jets: &[Jet], j: usize, parameters: &[f64]) -> Vec<HPoint> {
    let mut points = Vec::with_capacity(DEGREE_ALONG * (jets.len() - 1) + 1);
    points.push(jets[0].value[j]);
    for (index, pair) in jets.windows(2).enumerate() {
        let [from, to] = [&pair[0], &pair[1]];
        let span = parameters[index + 1] - parameters[index];
        let (slope, bend) = (span / 5.0, span * span / 20.0);
        points.push(from.value[j] + from.slope[j] * slope);
        points.push(from.value[j] + from.slope[j] * (2.0 * slope) + from.curvature[j] * bend);
        points.push(to.value[j] - to.slope[j] * (2.0 * slope) + to.curvature[j] * bend);
        points.push(to.value[j] - to.slope[j] * slope);
        points.push(to.value[j]);
    }
    points
        .into_iter()
        .map(|point| HPoint(point.into()))
        .collect()
}

/// The knots of [`quintic_row`]: clamped, each interior sample repeated to
/// the degree, so each segment is its own Bezier span.
fn quintic_knots(parameters: &[f64]) -> Result<KnotVector, NurbsError> {
    let ends = DEGREE_ALONG + 1;
    let interior = &parameters[1..parameters.len() - 1];
    KnotVector::new(
        std::iter::repeat_n(0.0, ends)
            .chain(
                interior
                    .iter()
                    .flat_map(|&t| std::iter::repeat_n(t, DEGREE_ALONG)),
            )
            .chain(std::iter::repeat_n(1.0, ends))
            .collect(),
    )
}
