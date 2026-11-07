use super::SketchError;
use super::drawn::{Drawn, Geometry, drawn_curves};
use super::joints::{JOINT_CONTACT, Joints};
use crate::builders::edges::{add_circle_edit, add_edge_edit};
use crate::builders::profiles::add_profile_from_edges_edit;
use crate::geometry::dim2::intersections::CurveCurveIntersection2;
use crate::geometry::dim2::utils::Point2;
use crate::geometry::{Curve, IntersectionCoverage, Plane, Point3};
use crate::model::{Model, OpResult, StaleResult};
use crate::solver::{CurveId, System2d};
use crate::topology::ModelEdit;
use crate::topology::edge::Edge;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, ProfileKey};
use std::collections::BTreeMap;

/// One connected run of a system's curves.
#[derive(Clone, Debug)]
pub struct SystemProfile {
    pub profile: ProfileKey,
    pub curves: Vec<CurveId>,
}

/// What [`add_profiles_from_system`] added: the edge of each curve, and the profiles they form.
///
/// The keys hold within this model. What identifies a curve from one build to the next
/// is its id, which this result ties to the keys.
#[derive(Clone, Debug)]
pub struct ProfilesFromSystem {
    pub edges: BTreeMap<CurveId, EdgeKey>,
    pub profiles: Vec<SystemProfile>,
    revision: Option<u64>,
}

impl OpResult for ProfilesFromSystem {
    fn stamp(&mut self, revision: u64) {
        self.revision = Some(revision);
    }
}

impl ProfilesFromSystem {
    /// Resolves a curve's edge at the revision that produced this result.
    pub fn edge<'m, P: Payload>(
        &self,
        model: &'m Model<P>,
        curve: impl Into<CurveId>,
    ) -> Result<Option<Edge<'m, P>>, StaleResult> {
        StaleResult::check(self.revision, model)?;
        Ok(self
            .edges
            .get(&curve.into())
            .map(|key| model.edge_unchecked(*key)))
    }
}

/// Adds the edges and profiles a solved system draws on `plane`, its coordinates
/// mapping through the plane's axes.
///
/// Curve ends are one vertex where the system's coincidences say so, never because they
/// happen to touch. Each connected run becomes a profile, open or closed. Curves that
/// branch, cross, overlap or touch without being joined are refused: a profile is a chain
/// or a cycle. [`add_sketch_from_system`](super::add_sketch_from_system) draws such curves
/// as the regions they enclose.
pub fn add_profiles_from_system<P: Payload>(
    model: &mut Model<P>,
    system: &System2d,
    plane: Plane,
    curves: &[CurveId],
) -> Result<ProfilesFromSystem, SketchError> {
    model.transaction_result(|edit| add_profiles_from_system_edit(edit, system, &plane, curves))
}

/// Refuses what cannot be drawn, then draws it inside the caller's transaction.
pub(crate) fn add_profiles_from_system_edit<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    system: &System2d,
    plane: &Plane,
    curves: &[CurveId],
) -> Result<ProfilesFromSystem, SketchError> {
    let drawn = drawn_curves(system, curves)?;
    let joints = Joints::of(system, &drawn)?;
    if let Some(curves) = joints.branching() {
        return Err(SketchError::Branching {
            curves: curves.to_vec(),
        });
    }
    refuse_crossings(&drawn, &joints)?;
    add_profiles_edit(edit, &drawn, &joints, plane)
}

/// Two listed curves may only meet where they are joined.
fn refuse_crossings(curves: &[Drawn], joints: &Joints) -> Result<(), SketchError> {
    let spans: Vec<_> = curves.iter().map(Drawn::span).collect();
    for (a, first) in curves.iter().enumerate() {
        for (b, second) in curves.iter().enumerate().skip(a + 1) {
            let (first, second) = (first.curve, second.curve);
            let found = spans[a].intersect_curve(&spans[b]).map_err(|source| {
                SketchError::Intersection {
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
                            return Err(SketchError::Crossing {
                                first,
                                second,
                                point: *point,
                            });
                        }
                    }
                    CurveCurveIntersection2::Overlap { .. } => {
                        return Err(SketchError::Overlapping { first, second });
                    }
                }
            }
            if *found.coverage() != IntersectionCoverage::Complete {
                return Err(SketchError::UncertainCrossing { first, second });
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
) -> Result<ProfilesFromSystem, SketchError> {
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
        profiles.push(SystemProfile {
            profile: add_profile_from_edges_edit(edit, &run_keys)?,
            curves: run.iter().map(|index| curves[*index].curve).collect(),
        });
    }
    Ok(ProfilesFromSystem {
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
) -> Result<EdgeKey, SketchError> {
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
