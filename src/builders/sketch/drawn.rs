use super::SketchError;
use crate::geometry::Interval;
use crate::geometry::dim2::curves::{Circle2, Curve2};
use crate::geometry::dim2::trimmed::{Arc2, Segment2, TrimmedCurve2};
use crate::geometry::dim2::utils::Point2;
use crate::solver::{CurveId, PointRef, Solution2d, System2d};
use std::collections::HashSet;
use std::f64::consts::TAU;

/// A listed curve with its solved geometry.
pub(super) struct Drawn {
    pub curve: CurveId,
    pub geometry: Geometry,
}

pub(super) enum Geometry {
    Segment(Segment2),
    Circle(Circle2),
    Arc(Arc2),
}

impl Drawn {
    /// Its ends with their solved positions; a circle has none.
    pub fn ends(&self) -> Vec<(PointRef, Point2)> {
        match (&self.geometry, self.curve) {
            (Geometry::Segment(line), CurveId::Segment(id)) => {
                vec![(id.start(), line.start()), (id.end(), line.end())]
            }
            (Geometry::Arc(arc), CurveId::Arc(id)) => {
                vec![(id.start(), arc.start()), (id.end(), arc.end())]
            }
            _ => vec![],
        }
    }

    /// The curve as a span, a circle running once round.
    pub fn span(&self) -> TrimmedCurve2 {
        match &self.geometry {
            Geometry::Segment(line) => line.clone().into(),
            Geometry::Arc(arc) => arc.clone().into(),
            Geometry::Circle(circle) => {
                TrimmedCurve2::new(Curve2::Circle(circle.clone()), Interval::new(0.0, TAU))
            }
        }
    }
}

/// Reads the listed curves' solved geometry, refusing an unsolved system and a bad list.
pub(super) fn drawn_curves(
    system: &System2d,
    curves: &[CurveId],
) -> Result<Vec<Drawn>, SketchError> {
    let solution = system.solution().ok_or(SketchError::NotSolved)?;
    let mut seen = HashSet::new();
    curves
        .iter()
        .map(|curve| {
            if !seen.insert(*curve) {
                return Err(SketchError::DuplicateCurve(*curve));
            }
            let geometry =
                solved_geometry(solution, *curve).ok_or(SketchError::UnknownCurve(*curve))?;
            Ok(Drawn {
                curve: *curve,
                geometry,
            })
        })
        .collect()
}

fn solved_geometry(solution: &Solution2d, curve: CurveId) -> Option<Geometry> {
    match curve {
        CurveId::Segment(id) => solution.segment(id).ok().map(Geometry::Segment),
        CurveId::Circle(id) => solution.circle(id).ok().map(Geometry::Circle),
        CurveId::Arc(id) => solution.arc(id).ok().map(Geometry::Arc),
    }
}
