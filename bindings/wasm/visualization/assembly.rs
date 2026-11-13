//! A visualization adapter for the assembly solver: preset mechanisms whose bodies the
//! browser drags with the mouse, one `solve_toward` per animation frame.

use crate::geometry::dim3::frame::Frame;
use crate::geometry::dim3::utils::Point3;
use crate::geometry::transform::Rigid;
use crate::solver::assembly::{
    Assembly3d, Attachment, BodyId, Bound, Coordinate, Joint3d, JointId, JointKind, Limit,
    Solution3d, Target3d,
};
use nalgebra::Vector3;
use serde::Serialize;
use wasm_bindgen::prelude::*;

/// A drawn primitive in its body's own coordinates.
#[derive(Clone, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
enum Part {
    /// An axis-aligned box.
    Box { center: [f64; 3], size: [f64; 3] },
    /// A cylinder centered on `center`, its axis along `axis`.
    Cylinder {
        center: [f64; 3],
        axis: [f64; 3],
        radius: f64,
        height: f64,
    },
}

#[derive(Clone, Serialize)]
struct BodyView {
    id: u64,
    name: &'static str,
    color: &'static str,
    fixed: bool,
    parts: Vec<Part>,
}

#[derive(Serialize)]
struct Pose {
    id: u64,
    /// Column-major 4×4, as three.js `Matrix4.fromArray` reads it.
    matrix: [f64; 16],
}

#[derive(Serialize)]
struct CoordinateView {
    joint: &'static str,
    coordinate: &'static str,
    value: f64,
    unit: &'static str,
}

#[derive(Serialize)]
struct State {
    poses: Vec<Pose>,
    coordinates: Vec<CoordinateView>,
    mobility: usize,
    redundancy: usize,
    active_limits: Vec<String>,
    iterations: usize,
    reached: Option<bool>,
    error: Option<String>,
}

#[derive(Serialize)]
struct Loaded {
    bodies: Vec<BodyView>,
    state: State,
}

/// A preset mechanism held across drags. Ids are those the assembly issued.
#[wasm_bindgen]
pub struct AssemblyPlayground {
    assembly: Assembly3d,
    bodies: Vec<BodyView>,
    joints: Vec<(JointId, &'static str)>,
    last: Option<Solution3d>,
}

impl Default for AssemblyPlayground {
    fn default() -> Self {
        Self::new()
    }
}

#[wasm_bindgen]
impl AssemblyPlayground {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        Self {
            assembly: Assembly3d::default(),
            bodies: vec![],
            joints: vec![],
            last: None,
        }
    }

    /// Builds a preset (`slider-crank`, `arm`, `pendulum`), solves it and returns its
    /// bodies and their poses as JSON.
    pub fn load(&mut self, preset: &str) -> Result<String, JsValue> {
        *self = Self::new();
        match preset {
            "slider-crank" => self.slider_crank(),
            "arm" => self.arm(),
            "pendulum" => self.pendulum(),
            _ => return Err(JsValue::from_str("unknown preset")),
        }
        .map_err(|error| JsValue::from_str(&error))?;
        let state = self.solve_with(&[]);
        to_json(&Loaded {
            bodies: self.bodies.clone(),
            state,
        })
    }

    /// Pulls `local`, a point in the body's own coordinates, toward the world `target`.
    #[allow(clippy::too_many_arguments)]
    pub fn drag(
        &mut self,
        body: u64,
        local_x: f64,
        local_y: f64,
        local_z: f64,
        target_x: f64,
        target_y: f64,
        target_z: f64,
    ) -> Result<String, JsValue> {
        let target = Target3d::point(
            BodyId::new(body),
            Point3::new(local_x, local_y, local_z),
            Point3::new(target_x, target_y, target_z),
        );
        to_json(&self.solve_with(&[target]))
    }
}

impl AssemblyPlayground {
    fn solve_with(&mut self, targets: &[Target3d]) -> State {
        let (solution, error) = match self.assembly.solve_toward(targets) {
            Ok(solution) => {
                self.last = Some(solution.clone());
                (Some(solution), None)
            }
            Err(error) => (self.last.clone(), Some(error.to_string())),
        };
        let Some(solution) = solution else {
            return State {
                poses: vec![],
                coordinates: vec![],
                mobility: 0,
                redundancy: 0,
                active_limits: vec![],
                iterations: 0,
                reached: None,
                error,
            };
        };
        let report = solution.report();
        let name = |joint: JointId| {
            self.joints
                .iter()
                .find(|(id, _)| *id == joint)
                .map_or("?", |(_, name)| name)
        };
        let mut coordinates = vec![];
        for (joint, joint_name) in &self.joints {
            let kind = self.assembly.joint(*joint).map(|j| j.kind);
            for coordinate in kind.map_or(&[][..], |kind| kind.coordinates()) {
                if let Ok(value) = solution.coordinate(*joint, *coordinate) {
                    let angular = coordinate.angular();
                    coordinates.push(CoordinateView {
                        joint: joint_name,
                        coordinate: coordinate_name(*coordinate),
                        value: if angular { value.to_degrees() } else { value },
                        unit: if angular { "°" } else { "" },
                    });
                }
            }
        }
        State {
            poses: self
                .bodies
                .iter()
                .filter_map(|body| {
                    let pose = solution.pose(BodyId::new(body.id)).ok()?;
                    Some(Pose {
                        id: body.id,
                        matrix: matrix(pose),
                    })
                })
                .collect(),
            coordinates,
            mobility: report.mobility,
            redundancy: report.redundancy,
            active_limits: report
                .active_limits
                .iter()
                .map(|limit| format!("{} {}", name(limit.joint), bound_name(limit.bound)))
                .collect(),
            iterations: report.iterations,
            reached: report.targets.first().map(|target| target.reached),
            error,
        }
    }

    fn body(
        &mut self,
        name: &'static str,
        color: &'static str,
        fixed: bool,
        parts: Vec<Part>,
    ) -> Result<BodyId, String> {
        let id = self
            .assembly
            .add_body(Rigid::identity())
            .map_err(|e| e.to_string())?;
        if fixed {
            self.assembly.fix(id).map_err(|e| e.to_string())?;
        }
        self.bodies.push(BodyView {
            id: id.value(),
            name,
            color,
            fixed,
            parts,
        });
        Ok(id)
    }

    fn joint(&mut self, name: &'static str, joint: Joint3d) -> Result<JointId, String> {
        let id = self.assembly.add_joint(joint).map_err(|e| e.to_string())?;
        self.joints.push((id, name));
        Ok(id)
    }

    fn slider_crank(&mut self) -> Result<(), String> {
        let (crank, rod) = (1., 3.);
        let ground = self.body(
            "bâti",
            "#8a8f98",
            true,
            vec![
                cuboid([-0.8, -0.8, -0.3], [rod + crank + 1.2, 0.8, -0.2]),
                Part::Cylinder {
                    center: [0., 0., -0.1],
                    axis: [0., 0., 1.],
                    radius: 0.25,
                    height: 0.2,
                },
                cuboid([crank + 0.5, -0.15, -0.2], [rod + crank + 1.2, 0.15, -0.05]),
            ],
        )?;
        let crank_body = self.body(
            "manivelle",
            "#e8833a",
            false,
            vec![
                cuboid([0., -0.15, 0.], [crank, 0.15, 0.15]),
                upright(0., 0.2, 0., 0.15),
                upright(crank, 0.12, 0., 0.45),
            ],
        )?;
        let rod_body = self.body(
            "bielle",
            "#3a7be8",
            false,
            vec![cuboid([crank, -0.1, 0.2], [crank + rod, 0.1, 0.3])],
        )?;
        let x = crank + rod;
        let slider = self.body(
            "coulisseau",
            "#3aa865",
            false,
            vec![
                cuboid([x - 0.35, -0.25, -0.05], [x + 0.35, 0.25, 0.2]),
                upright(x, 0.1, 0.2, 0.25),
            ],
        )?;
        let rail = |x: f64| Frame::from_xz(Point3::new(x, 0., 0.), Vector3::y(), Vector3::x());
        self.joint(
            "pivot bâti",
            hinge(ground, crank_body, at(0., 0., 0.), None),
        )?;
        self.joint(
            "pivot maneton",
            hinge(crank_body, rod_body, at(crank, 0., 0.), None),
        )?;
        self.joint(
            "pivot coulisseau",
            hinge(rod_body, slider, at(x, 0., 0.), None),
        )?;
        self.joint(
            "glissière",
            Joint3d::new(
                JointKind::Prismatic,
                Attachment::new(ground, rail(0.)),
                Attachment::new(slider, rail(x)),
            ),
        )?;
        Ok(())
    }

    fn arm(&mut self) -> Result<(), String> {
        let (shoulder, upper, fore) = (0.8, 1.4, 1.2);
        let base = self.body("socle", "#8a8f98", true, vec![upright(0., 0.6, 0., 0.3)])?;
        let turret = self.body(
            "tourelle",
            "#e8833a",
            false,
            vec![
                upright(0., 0.35, 0.3, 0.3),
                cuboid([-0.2, -0.3, 0.6], [0.2, 0.3, shoulder + 0.15]),
            ],
        )?;
        let arm = self.body(
            "bras",
            "#3a7be8",
            false,
            vec![
                cuboid([0., -0.1, shoulder - 0.1], [upper, 0.1, shoulder + 0.1]),
                across(0., shoulder, 0.16, 0.65),
            ],
        )?;
        let forearm = self.body(
            "avant-bras",
            "#3aa865",
            false,
            vec![
                cuboid(
                    [upper, -0.08, shoulder - 0.08],
                    [upper + fore, 0.08, shoulder + 0.08],
                ),
                across(upper, shoulder, 0.13, 0.3),
                Part::Cylinder {
                    center: [upper + fore, 0., shoulder],
                    axis: [1., 0., 0.],
                    radius: 0.12,
                    height: 0.1,
                },
            ],
        )?;
        // Pitch axes along the world Y: rotating about them turns X toward -Z.
        let pitch =
            |x: f64| Frame::from_xz(Point3::new(x, 0., shoulder), Vector3::x(), Vector3::y());
        let degrees = |min: f64, max: f64| Limit::between(min.to_radians(), max.to_radians());
        self.joint(
            "lacet",
            hinge(base, turret, at(0., 0., 0.3), Some(degrees(-150., 150.))),
        )?;
        self.joint(
            "épaule",
            Joint3d::new(
                JointKind::Revolute,
                Attachment::new(turret, pitch(0.)),
                Attachment::new(arm, pitch(0.)),
            )
            .with_limit(Coordinate::Angle, degrees(-120., 30.)),
        )?;
        self.joint(
            "coude",
            Joint3d::new(
                JointKind::Revolute,
                Attachment::new(arm, pitch(upper)),
                Attachment::new(forearm, pitch(upper)),
            )
            .with_limit(Coordinate::Angle, degrees(-10., 150.)),
        )?;
        Ok(())
    }

    fn pendulum(&mut self) -> Result<(), String> {
        let top = 2.2;
        let gallows = self.body(
            "potence",
            "#8a8f98",
            true,
            vec![
                cuboid([-1.6, -0.6, -0.1], [0.6, 0.6, 0.]),
                upright(-1.2, 0.08, 0., top + 0.1),
                cuboid([-1.2, -0.06, top], [0.1, 0.06, top + 0.12]),
            ],
        )?;
        let bob = self.body(
            "pendule",
            "#e8833a",
            false,
            vec![
                cuboid([-0.04, -0.04, 0.55], [0.04, 0.04, top]),
                upright(0., 0.25, 0.3, 0.3),
            ],
        )?;
        // Z points down the pendulum, so the cone measures its swing from the vertical.
        let ball = Frame::from_xz(Point3::new(0., 0., top), Vector3::x(), -Vector3::z());
        self.joint(
            "rotule",
            Joint3d::new(
                JointKind::Spherical,
                Attachment::new(gallows, ball.clone()),
                Attachment::new(bob, ball),
            )
            .with_cone(45_f64.to_radians()),
        )?;
        Ok(())
    }
}

fn at(x: f64, y: f64, z: f64) -> Frame {
    Frame::at(Point3::new(x, y, z))
}

fn hinge(first: BodyId, second: BodyId, frame: Frame, limit: Option<Limit>) -> Joint3d {
    let joint = Joint3d::new(
        JointKind::Revolute,
        Attachment::new(first, frame.clone()),
        Attachment::new(second, frame),
    );
    match limit {
        Some(limit) => joint.with_limit(Coordinate::Angle, limit),
        None => joint,
    }
}

/// A box between two corners.
fn cuboid(low: [f64; 3], high: [f64; 3]) -> Part {
    Part::Box {
        center: [0, 1, 2].map(|i| (low[i] + high[i]) / 2.),
        size: [0, 1, 2].map(|i| high[i] - low[i]),
    }
}

/// An upright cylinder at `(x, 0)`, its bottom at `bottom`.
fn upright(x: f64, radius: f64, bottom: f64, height: f64) -> Part {
    Part::Cylinder {
        center: [x, 0., bottom + height / 2.],
        axis: [0., 0., 1.],
        radius,
        height,
    }
}

/// A cylinder across the arm, along Y, at `(x, z)`.
fn across(x: f64, z: f64, radius: f64, height: f64) -> Part {
    Part::Cylinder {
        center: [x, 0., z],
        axis: [0., 1., 0.],
        radius,
        height,
    }
}

fn matrix(pose: Rigid) -> [f64; 16] {
    let x = pose.apply_vector(Vector3::x());
    let y = pose.apply_vector(Vector3::y());
    let z = pose.apply_vector(Vector3::z());
    let t = pose.apply(Point3::origin());
    [
        x.x, x.y, x.z, 0., y.x, y.y, y.z, 0., z.x, z.y, z.z, 0., t.x, t.y, t.z, 1.,
    ]
}

fn coordinate_name(coordinate: Coordinate) -> &'static str {
    match coordinate {
        Coordinate::Angle => "angle",
        Coordinate::Slide => "course",
        Coordinate::X => "x",
        Coordinate::Y => "y",
        Coordinate::Roll => "roulis",
        Coordinate::Alpha => "α",
        Coordinate::Beta => "β",
    }
}

fn bound_name(bound: Bound) -> String {
    match bound {
        Bound::Min(coordinate) => format!("{} min", coordinate_name(coordinate)),
        Bound::Max(coordinate) => format!("{} max", coordinate_name(coordinate)),
        Bound::Cone => "cône".to_owned(),
    }
}

fn to_json(value: &impl Serialize) -> Result<String, JsValue> {
    serde_json::to_string(value).map_err(|error| JsValue::from_str(&error.to_string()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_preset_loads_and_drags() {
        for preset in ["slider-crank", "arm", "pendulum"] {
            let mut playground = AssemblyPlayground::new();
            playground.load(preset).unwrap();
            let moving = playground.bodies.iter().find(|b| !b.fixed).unwrap().id;
            let state = playground.drag(moving, 0.5, 0., 0.5, 1., 1., 1.).unwrap();
            assert!(!state.contains("\"error\":\""), "{preset}: {state}");
        }
    }
}
