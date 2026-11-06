use super::SketchBuildError;
use super::joints::Joints;
use crate::builders::edges::{add_circle_edit, add_edge_edit};
use crate::builders::profiles::add_profile_from_edges_edit;
use crate::geometry::dim2::curves::{Circle2, Curve2};
use crate::geometry::dim2::intersections::CurveCurveIntersection2;
use crate::geometry::dim2::trimmed::{Arc2, Segment2, TrimmedCurve2};
use crate::geometry::dim2::utils::Point2;
use crate::geometry::{Curve, IntersectionCoverage, Interval, Plane, Point3};
use crate::model::{Model, OpResult, StaleResult};
use crate::solver::{ArcId, CircleId, PointRef, SegmentId, Solution2d, System2d};
use crate::topology::ModelEdit;
use crate::topology::edge::Edge;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, ProfileKey};
use std::collections::{BTreeMap, HashSet};
use std::f64::consts::TAU;

/// How far a reported crossing may sit from a joint and still be that joint. Curves leaving
/// a joint tangentially meet at a poorly located point, well outside the kernel's tolerance.
const JOINT_CONTACT: f64 = 1e-6;

/// A curve of a sketch that becomes an edge. Points and construction curves stay in the
/// system without being listed.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum SketchCurve {
    Segment(SegmentId),
    Circle(CircleId),
    Arc(ArcId),
}

impl From<SegmentId> for SketchCurve {
    fn from(id: SegmentId) -> Self {
        Self::Segment(id)
    }
}
impl From<CircleId> for SketchCurve {
    fn from(id: CircleId) -> Self {
        Self::Circle(id)
    }
}
impl From<ArcId> for SketchCurve {
    fn from(id: ArcId) -> Self {
        Self::Arc(id)
    }
}

/// One connected run of a sketch's curves.
#[derive(Clone, Debug)]
pub struct SketchProfile {
    pub profile: ProfileKey,
    pub curves: Vec<SketchCurve>,
}

/// What [`build_profiles`] added: the edge of each curve, and the profiles they form.
///
/// The keys hold within this model. What identifies a curve from one build to the next
/// is its id, which this result ties to the keys.
#[derive(Clone, Debug)]
pub struct SketchProfiles {
    pub edges: BTreeMap<SketchCurve, EdgeKey>,
    pub profiles: Vec<SketchProfile>,
    revision: Option<u64>,
}

impl OpResult for SketchProfiles {
    fn stamp(&mut self, revision: u64) {
        self.revision = Some(revision);
    }
}

impl SketchProfiles {
    /// Resolves a curve's edge at the revision that produced this result.
    pub fn edge<'m, P: Payload>(
        &self,
        model: &'m Model<P>,
        curve: impl Into<SketchCurve>,
    ) -> Result<Option<Edge<'m, P>>, StaleResult> {
        StaleResult::check(self.revision, model)?;
        Ok(self
            .edges
            .get(&curve.into())
            .map(|key| model.edge_unchecked(*key)))
    }
}

/// Adds the edges and profiles a solved system draws on `plane`, sketch coordinates
/// mapping through the plane's axes.
///
/// Curve ends are one vertex where the system's coincidences say so, never because they
/// happen to touch. Each connected run becomes a profile, open or closed. Curves that
/// branch, cross, overlap or touch without being joined are refused.
pub fn build_profiles<P: Payload>(
    model: &mut Model<P>,
    system: &System2d,
    plane: Plane,
    curves: &[SketchCurve],
) -> Result<SketchProfiles, SketchBuildError> {
    let drawn = drawn_curves(system, curves)?;
    let joints = Joints::of(system, &drawn)?;
    refuse_crossings(&drawn, &joints)?;
    model.transaction_result(|edit| add_profiles_edit(edit, &drawn, &joints, &plane))
}

/// A listed curve with its solved geometry.
pub(super) struct Drawn {
    pub curve: SketchCurve,
    geometry: Geometry,
}

enum Geometry {
    Segment(Segment2),
    Circle(Circle2),
    Arc(Arc2),
}

impl Drawn {
    /// Its ends with their solved positions; a circle has none.
    pub fn ends(&self) -> Vec<(PointRef, Point2)> {
        match (&self.geometry, self.curve) {
            (Geometry::Segment(line), SketchCurve::Segment(id)) => {
                vec![(id.start(), line.start()), (id.end(), line.end())]
            }
            (Geometry::Arc(arc), SketchCurve::Arc(id)) => {
                vec![(id.start(), arc.start()), (id.end(), arc.end())]
            }
            _ => vec![],
        }
    }

    fn span(&self) -> TrimmedCurve2 {
        match &self.geometry {
            Geometry::Segment(line) => line.clone().into(),
            Geometry::Arc(arc) => arc.clone().into(),
            Geometry::Circle(circle) => {
                TrimmedCurve2::new(Curve2::Circle(circle.clone()), Interval::new(0.0, TAU))
            }
        }
    }
}

fn drawn_curves(system: &System2d, curves: &[SketchCurve]) -> Result<Vec<Drawn>, SketchBuildError> {
    let solution = system.solution().ok_or(SketchBuildError::NotSolved)?;
    let mut seen = HashSet::new();
    curves
        .iter()
        .map(|curve| {
            if !seen.insert(*curve) {
                return Err(SketchBuildError::DuplicateCurve(*curve));
            }
            let geometry =
                solved_geometry(solution, *curve).ok_or(SketchBuildError::UnknownCurve(*curve))?;
            Ok(Drawn {
                curve: *curve,
                geometry,
            })
        })
        .collect()
}

fn solved_geometry(solution: &Solution2d, curve: SketchCurve) -> Option<Geometry> {
    match curve {
        SketchCurve::Segment(id) => solution.segment(id).ok().map(Geometry::Segment),
        SketchCurve::Circle(id) => solution.circle(id).ok().map(Geometry::Circle),
        SketchCurve::Arc(id) => solution.arc(id).ok().map(Geometry::Arc),
    }
}

/// Two listed curves may only meet where they are joined.
fn refuse_crossings(curves: &[Drawn], joints: &Joints) -> Result<(), SketchBuildError> {
    let spans: Vec<_> = curves.iter().map(Drawn::span).collect();
    for (a, first) in curves.iter().enumerate() {
        for (b, second) in curves.iter().enumerate().skip(a + 1) {
            let (first, second) = (first.curve, second.curve);
            let found = spans[a].intersect_curve(&spans[b]).map_err(|source| {
                SketchBuildError::Intersection {
                    first,
                    second,
                    source,
                }
            })?;
            let shared = shared_joints(&curves[a], &curves[b], joints);
            for intersection in found.iter() {
                match intersection {
                    CurveCurveIntersection2::Point { point, .. } => {
                        if !shared
                            .iter()
                            .any(|joint| (joint - point).norm() <= JOINT_CONTACT)
                        {
                            return Err(SketchBuildError::Crossing {
                                first,
                                second,
                                point: *point,
                            });
                        }
                    }
                    CurveCurveIntersection2::Overlap { .. } => {
                        return Err(SketchBuildError::Overlapping { first, second });
                    }
                }
            }
            if *found.coverage() != IntersectionCoverage::Complete {
                return Err(SketchBuildError::UncertainCrossing { first, second });
            }
        }
    }
    Ok(())
}

/// Where two curves are joined.
fn shared_joints(a: &Drawn, b: &Drawn, joints: &Joints) -> Vec<Point2> {
    let mut shared = vec![];
    for (end, _) in a.ends() {
        for (other, _) in b.ends() {
            if joints.joint(end) == joints.joint(other) {
                shared.push(joints.position(end));
            }
        }
    }
    shared
}

/// Adds every curve's edge, then one profile per connected run, inside the caller's edit.
fn add_profiles_edit<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    curves: &[Drawn],
    joints: &Joints,
    plane: &Plane,
) -> Result<SketchProfiles, SketchBuildError> {
    let mut edges = BTreeMap::new();
    let mut keys = Vec::with_capacity(curves.len());
    for curve in curves {
        let key = add_curve_edit(edit, curve, joints, plane)?;
        edges.insert(curve.curve, key);
        keys.push(key);
    }
    let mut profiles = vec![];
    for run in connected_runs(curves, joints) {
        let run_keys: Vec<_> = run.iter().map(|index| keys[*index]).collect();
        profiles.push(SketchProfile {
            profile: add_profile_from_edges_edit(edit, &run_keys)?,
            curves: run.iter().map(|index| curves[*index].curve).collect(),
        });
    }
    Ok(SketchProfiles {
        edges,
        profiles,
        revision: None,
    })
}

fn add_curve_edit<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    drawn: &Drawn,
    joints: &Joints,
    plane: &Plane,
) -> Result<EdgeKey, SketchBuildError> {
    let lift = |point: Point2| -> Point3 { plane.point_at(point.x, point.y) };
    let at = |center: Point2| Plane::new(lift(center), plane.x_dir(), plane.normal());
    let key = match &drawn.geometry {
        Geometry::Segment(_) => {
            let [(start, _), (end, _)] = drawn.ends()[..] else {
                unreachable!("a segment has two ends");
            };
            let (start, end) = (lift(joints.position(start)), lift(joints.position(end)));
            add_edge_edit(edit, start, end, Curve::line(start, end))?
        }
        Geometry::Arc(arc) => {
            let [(start, _), (end, _)] = drawn.ends()[..] else {
                unreachable!("an arc has two ends");
            };
            let circle = Curve::circle(at(arc.curve().center()), arc.curve().radius());
            let support = if arc.interval().delta() < 0. {
                circle.reversed()
            } else {
                circle
            };
            let (start, end) = (lift(joints.position(start)), lift(joints.position(end)));
            add_edge_edit(edit, start, end, support)?
        }
        Geometry::Circle(circle) => add_circle_edit(edit, at(circle.center()), circle.radius())?,
    };
    Ok(key)
}

/// The curves joined to one another, as lists of indices in the order given.
fn connected_runs(curves: &[Drawn], joints: &Joints) -> Vec<Vec<usize>> {
    let mut run_of: Vec<Option<usize>> = vec![None; curves.len()];
    let mut runs: Vec<Vec<usize>> = vec![];
    for start in 0..curves.len() {
        if run_of[start].is_some() {
            continue;
        }
        let run = runs.len();
        runs.push(vec![]);
        let mut pending = vec![start];
        run_of[start] = Some(run);
        while let Some(index) = pending.pop() {
            runs[run].push(index);
            for other in 0..curves.len() {
                if run_of[other].is_none()
                    && !shared_joints(&curves[index], &curves[other], joints).is_empty()
                {
                    run_of[other] = Some(run);
                    pending.push(other);
                }
            }
        }
        runs[run].sort_unstable();
    }
    runs
}
