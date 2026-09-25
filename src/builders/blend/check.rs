//! Checks a blend's result before it is allowed to commit.
//!
//! Planning checks that each piece fits where it is placed; these check the
//! whole: that no face the surgery touched ends up with boundaries that cross
//! or run backwards, and that every solid it touched is still a closed,
//! outward shell. Any failure rolls the whole call back.

use std::collections::HashMap;

use nalgebra::Vector2;

use super::errors::BlendError;
use super::execute::Executed;
use super::surgery::Surgery;
use crate::geometry::parameter::Fraction;
use crate::geometry::{Curve2, LINEAR_TOLERANCE, Point2};
use crate::model::Model;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::FaceKey;
use crate::topology::validation::{validate_solid_manifold, validate_solid_orientation};

/// Chords per curved pcurve when a boundary is sampled for crossings.
const CURVED_SAMPLES: usize = 16;

/// Records the winding of every face the surgery can change, before it does.
///
/// A face a blend changes either meets a vertex it consumes, or runs along an
/// edge it cuts: a closed edge is cut with no vertex to consume, and a rail
/// run past the far rim of its face turns that face inside out without
/// crossing anything.
pub(crate) fn record_windings<P: Payload>(
    model: &Model<P>,
    surgery: &Surgery,
) -> HashMap<FaceKey, f64> {
    let mut faces = surgery
        .cuts
        .iter()
        .flat_map(|cut| cut.sides.iter().map(|side| side.face))
        .collect::<Vec<_>>();
    for &vertex in &surgery.consumed_vertices {
        if let Some(view) = model.vertex(vertex) {
            faces.extend(view.faces().iter().map(|face| face.key()));
        }
    }
    let mut windings = HashMap::new();
    for face in faces {
        if windings.contains_key(&face) {
            continue;
        }
        if let Some(area) = model.face_unchecked(face).boundary_signed_area() {
            windings.insert(face, area);
        }
    }
    windings
}

/// Refuses a result in which a touched face's boundary crosses itself, or a
/// changed face's outer boundary turned inside out.
pub(crate) fn check_faces<P: Payload>(
    model: &Model<P>,
    executed: &Executed,
    windings: &HashMap<FaceKey, f64>,
) -> Result<(), BlendError> {
    for &face in executed.changed_faces.iter().chain(&executed.faces) {
        let view = model.face_unchecked(face);
        let area = view.boundary_signed_area();
        if let (Some(before), Some(after)) = (windings.get(&face), area)
            && (after.abs() <= LINEAR_TOLERANCE || before.signum() != after.signum())
        {
            return Err(BlendError::FaceDoesNotFit { face });
        }
        let attr = model.face_attr_unchecked(face);
        let loops = view
            .loops()
            .iter()
            .map(|loop_| {
                let mut points = loop_
                    .darts()
                    .filter_map(|dart| attr.pcurves.get(&dart))
                    .flat_map(|pcurve| {
                        let segments = match pcurve.curve() {
                            Curve2::Line(_) => 1,
                            _ => CURVED_SAMPLES,
                        };
                        let mut points = pcurve.sample(segments);
                        points.pop();
                        points
                    })
                    .collect::<Vec<_>>();
                // A wrapping loop runs one period and closes only on the
                // surface: its last point is where it began, a period on.
                let closed = !loop_.is_wrapping();
                if !closed
                    && let Some(end) = loop_.darts().last().and_then(|dart| {
                        attr.pcurves
                            .get(&dart)
                            .map(|pcurve| pcurve.point_at(Fraction::END))
                    })
                {
                    points.push(end);
                }
                Polyline { points, closed }
            })
            .collect::<Vec<_>>();
        if boundaries_cross(&loops) {
            return Err(BlendError::FaceDoesNotFit { face });
        }
    }
    Ok(())
}

/// Refuses a result in which a touched solid is no longer a closed, outward
/// shell.
pub(crate) fn check_solids<P: Payload>(
    model: &Model<P>,
    executed: &Executed,
) -> Result<(), BlendError> {
    for &solid in &executed.solids {
        validate_solid_manifold(model, solid)
            .and_then(|()| validate_solid_orientation(model, solid))
            .map_err(|source| BlendError::InvalidResult { solid, source })?;
    }
    Ok(())
}

/// One boundary loop sampled in the parameter plane.
struct Polyline {
    points: Vec<Point2>,
    /// Whether the last point joins back to the first; a wrapping loop's
    /// does not.
    closed: bool,
}

/// Whether any two non-adjacent chords of the polylines meet.
fn boundaries_cross(loops: &[Polyline]) -> bool {
    let chords = loops
        .iter()
        .enumerate()
        .flat_map(|(loop_index, polyline)| {
            let points = &polyline.points;
            let count = if polyline.closed {
                points.len()
            } else {
                points.len().saturating_sub(1)
            };
            (0..count).map(move |index| {
                (
                    loop_index,
                    index,
                    points.len(),
                    points[index],
                    points[(index + 1) % points.len()],
                )
            })
        })
        .collect::<Vec<_>>();
    for (first_index, first) in chords.iter().enumerate() {
        for second in &chords[first_index + 1..] {
            let same_loop = first.0 == second.0;
            let adjacent = same_loop
                && ((first.1 + 1) % first.2 == second.1 || (second.1 + 1) % second.2 == first.1);
            if adjacent {
                continue;
            }
            if chords_meet(first.3, first.4, second.3, second.4) {
                return true;
            }
        }
    }
    false
}

/// Whether two chords share a point, within the linear tolerance.
fn chords_meet(a: Point2, b: Point2, c: Point2, d: Point2) -> bool {
    let tolerance = LINEAR_TOLERANCE.sqrt();
    let first = b - a;
    let second = d - c;
    let denominator = cross(first, second);
    if denominator.abs() <= LINEAR_TOLERANCE * first.norm() * second.norm() {
        // Parallel chords meet only if they overlap along one line.
        let offset = c - a;
        if cross(first, offset).abs() > tolerance * first.norm().max(1.0) {
            return false;
        }
        let length = first.norm_squared();
        if length <= LINEAR_TOLERANCE {
            return false;
        }
        let along = |point: Point2| (point - a).dot(&first) / length;
        let (low, high) = {
            let (s, t) = (along(c), along(d));
            (s.min(t), s.max(t))
        };
        return high >= -tolerance && low <= 1.0 + tolerance;
    }
    let s = cross(c - a, second) / denominator;
    let t = cross(c - a, first) / denominator;
    let slack_s = tolerance / first.norm().max(tolerance);
    let slack_t = tolerance / second.norm().max(tolerance);
    (-slack_s..=1.0 + slack_s).contains(&s) && (-slack_t..=1.0 + slack_t).contains(&t)
}

fn cross(a: Vector2<f64>, b: Vector2<f64>) -> f64 {
    a.x * b.y - a.y * b.x
}
