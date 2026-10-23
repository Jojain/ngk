use crate::geometry::dim2::{curves::Circle2, trimmed::TrimmedCurve2, utils::Point2};
use crate::solver::{
    ArcKey, CircleKey, CircularKey, Constraint2d as C, ConstraintKey, GeometryKey, PointKey,
    PointRef, SegmentKey, SolveError, SolveReport, System2d, TangencyMode, Target2d,
};
use nalgebra::Vector2;
use serde::{Deserialize, Serialize};
use wasm_bindgen::prelude::*;

/// Stateful visualization adapter. Browser IDs are local to this playground session.
#[wasm_bindgen]
pub struct SolverPlayground {
    system: System2d,
    geometry: Vec<(u32, Shape)>,
    constraints: Vec<(u32, ConstraintKey, Relation)>,
    next_id: u32,
    report: Option<SolveReport>,
    status: String,
    message: String,
    suspects: Vec<u32>,
}

#[derive(Clone, Copy)]
enum Shape {
    Point(PointKey),
    Segment(SegmentKey),
    Circle(CircleKey),
    Arc(ArcKey),
}
impl Shape {
    fn key(self) -> GeometryKey {
        match self {
            Self::Point(k) => k.into(),
            Self::Segment(k) => k.into(),
            Self::Circle(k) => k.into(),
            Self::Arc(k) => k.into(),
        }
    }
}

#[derive(Clone, Deserialize, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
enum Geometry {
    Point {
        position: [f64; 2],
    },
    Segment {
        start: [f64; 2],
        end: [f64; 2],
    },
    Circle {
        center: [f64; 2],
        radius: f64,
    },
    Arc {
        center: [f64; 2],
        radius: f64,
        start_angle: f64,
        sweep: f64,
    },
}

#[derive(Clone, Copy, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
enum Role {
    Whole,
    Point,
    Start,
    End,
    Center,
}
#[derive(Clone, Copy, Deserialize, Serialize)]
struct Reference {
    id: u32,
    role: Role,
}

#[derive(Clone, Copy, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
enum RelationKind {
    Coincident,
    Distance,
    OffsetX,
    OffsetY,
    PointOnLine,
    PointOnCircle,
    DistanceToLine,
    Horizontal,
    Vertical,
    Length,
    Radius,
    EqualLength,
    EqualRadius,
    Parallel,
    Perpendicular,
    Angle,
    OrientedAngle,
    Concentric,
    Midpoint,
    SymmetricAboutLine,
    SymmetricAboutPoint,
    FixedPoint,
    Fixed,
    TangentLineCircle,
    TangentExternal,
    TangentInternal,
    TangentAutomatic,
}
#[derive(Clone, Deserialize, Serialize)]
struct Relation {
    kind: RelationKind,
    a: Reference,
    b: Option<Reference>,
    c: Option<Reference>,
    value: Option<f64>,
}
#[derive(Deserialize)]
struct Target {
    point: Reference,
    position: [f64; 2],
}
#[derive(Deserialize)]
#[serde(tag = "op", rename_all = "snake_case")]
enum Command {
    Snapshot,
    AddGeometry { geometry: Geometry },
    RemoveGeometry { id: u32 },
    AddConstraint { relation: Relation },
    UpdateConstraint { id: u32, relation: Relation },
    RemoveConstraint { id: u32 },
    Solve { target: Option<Target> },
}
#[derive(Serialize)]
struct GeometryView {
    id: u32,
    #[serde(flatten)]
    geometry: Geometry,
}
#[derive(Serialize)]
struct ConstraintView {
    id: u32,
    relation: Relation,
    warning: bool,
    suspect: bool,
    ratio: Option<f64>,
}
#[derive(Serialize)]
struct View {
    geometry: Vec<GeometryView>,
    constraints: Vec<ConstraintView>,
    status: String,
    message: String,
    dof: Option<usize>,
    target_distance: Option<f64>,
    iterations: usize,
}

#[wasm_bindgen]
impl SolverPlayground {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        Self {
            system: System2d::default(),
            geometry: vec![],
            constraints: vec![],
            next_id: 1,
            report: None,
            status: "edited".into(),
            message: String::new(),
            suspects: vec![],
        }
    }

    /// Applies an edit or solve, returning JSON geometry and diagnostics from the Rust solver.
    /// Invalid commands throw; numerical failures return a view retaining the editable state.
    pub fn execute(&mut self, request: &str) -> Result<String, JsValue> {
        let command =
            serde_json::from_str(request).map_err(|e| JsValue::from_str(&format!("{e}")))?;
        self.apply(command).map_err(|e| JsValue::from_str(&e))?;
        serde_json::to_string(&self.view().map_err(|e| JsValue::from_str(&e))?)
            .map_err(|e| JsValue::from_str(&e.to_string()))
    }
}
impl Default for SolverPlayground {
    fn default() -> Self {
        Self::new()
    }
}

impl SolverPlayground {
    fn edited(&mut self) {
        self.report = None;
        self.status = "edited".into();
        self.message.clear();
        self.suspects.clear();
    }
    fn shape(&self, reference: Reference) -> Result<Shape, String> {
        self.geometry
            .iter()
            .find(|(id, _)| *id == reference.id)
            .map(|(_, s)| *s)
            .ok_or_else(|| "Unknown geometry".into())
    }
    fn point(&self, r: Reference) -> Result<PointRef, String> {
        match (self.shape(r)?, r.role) {
            (Shape::Point(k), Role::Point) => Ok(k.into()),
            (Shape::Segment(k), Role::Start) => Ok(k.start()),
            (Shape::Segment(k), Role::End) => Ok(k.end()),
            (Shape::Circle(k), Role::Center) => Ok(k.center()),
            (Shape::Arc(k), Role::Center) => Ok(k.center()),
            (Shape::Arc(k), Role::Start) => Ok(k.start()),
            (Shape::Arc(k), Role::End) => Ok(k.end()),
            _ => Err("Select a point or a characteristic point".into()),
        }
    }
    fn line(&self, r: Reference) -> Result<SegmentKey, String> {
        match self.shape(r)? {
            Shape::Segment(k) => Ok(k),
            _ => Err("Select a segment".into()),
        }
    }
    fn circle(&self, r: Reference) -> Result<CircularKey, String> {
        match self.shape(r)? {
            Shape::Circle(k) => Ok(k.into()),
            Shape::Arc(k) => Ok(k.into()),
            _ => Err("Select a circle or arc".into()),
        }
    }
    fn relation(&self, r: &Relation) -> Result<C, String> {
        let b = || r.b.ok_or_else(|| "Select reference B".to_string());
        let c = || r.c.ok_or_else(|| "Select reference C".to_string());
        let value = || r.value.ok_or_else(|| "Enter a dimension".to_string());
        use RelationKind::*;
        Ok(match r.kind {
            Coincident => C::coincident(self.point(r.a)?, self.point(b()?)?),
            Distance => C::distance(self.point(r.a)?, self.point(b()?)?, value()?),
            OffsetX => C::offset_x(self.point(r.a)?, self.point(b()?)?, value()?),
            OffsetY => C::offset_y(self.point(r.a)?, self.point(b()?)?, value()?),
            PointOnLine => C::point_on_line(self.point(r.a)?, self.line(b()?)?),
            PointOnCircle => C::point_on_circle(self.point(r.a)?, self.circle(b()?)?),
            DistanceToLine => C::distance_to_line(self.point(r.a)?, self.line(b()?)?, value()?),
            Horizontal => C::horizontal(self.line(r.a)?),
            Vertical => C::vertical(self.line(r.a)?),
            Length => C::length(self.line(r.a)?, value()?),
            Radius => C::radius(self.circle(r.a)?, value()?),
            EqualLength => C::equal_length(self.line(r.a)?, self.line(b()?)?),
            EqualRadius => C::equal_radius(self.circle(r.a)?, self.circle(b()?)?),
            Parallel => C::parallel(self.line(r.a)?, self.line(b()?)?),
            Perpendicular => C::perpendicular(self.line(r.a)?, self.line(b()?)?),
            Angle => C::angle(self.line(r.a)?, self.line(b()?)?, value()?.to_radians()),
            OrientedAngle => {
                C::oriented_angle(self.line(r.a)?, self.line(b()?)?, value()?.to_radians())
            }
            Concentric => C::concentric(self.circle(r.a)?, self.circle(b()?)?),
            Midpoint => C::midpoint(self.point(r.a)?, self.line(b()?)?),
            SymmetricAboutLine => {
                C::symmetric_about_line(self.point(r.a)?, self.point(b()?)?, self.line(c()?)?)
            }
            SymmetricAboutPoint => {
                C::symmetric_about_point(self.point(r.a)?, self.point(b()?)?, self.point(c()?)?)
            }
            FixedPoint => C::fixed_point(self.point(r.a)?),
            Fixed => C::fixed(self.shape(r.a)?.key()),
            TangentLineCircle => C::tangent_line_circle(self.line(r.a)?, self.circle(b()?)?),
            TangentExternal | TangentInternal | TangentAutomatic => C::tangent_circles(
                self.circle(r.a)?,
                self.circle(b()?)?,
                match r.kind {
                    TangentExternal => TangencyMode::External,
                    TangentInternal => TangencyMode::Internal,
                    _ => TangencyMode::Automatic,
                },
            ),
        })
    }
    fn apply(&mut self, command: Command) -> Result<(), String> {
        match command {
            Command::Snapshot => return Ok(()),
            Command::AddGeometry { geometry } => {
                let shape = match geometry {
                    Geometry::Point { position } => {
                        Shape::Point(self.system.add_point(position.into()).map_err(message)?)
                    }
                    Geometry::Segment { start, end } => Shape::Segment(
                        self.system
                            .add_segment(TrimmedCurve2::segment(start.into(), end.into()))
                            .map_err(message)?,
                    ),
                    Geometry::Circle { center, radius } => Shape::Circle(
                        self.system
                            .add_circle(Circle2::new(center.into(), Vector2::x(), radius))
                            .map_err(message)?,
                    ),
                    Geometry::Arc {
                        center,
                        radius,
                        start_angle,
                        sweep,
                    } => Shape::Arc(
                        self.system
                            .add_arc(TrimmedCurve2::arc(
                                center.into(),
                                Vector2::new(start_angle.cos(), start_angle.sin()),
                                radius,
                                sweep,
                            ))
                            .map_err(message)?,
                    ),
                };
                self.geometry.push((self.next_id, shape));
                self.next_id += 1;
            }
            Command::RemoveGeometry { id } => {
                let shape = self.shape(Reference {
                    id,
                    role: Role::Whole,
                })?;
                let removed = self.system.remove_cascade(shape.key()).map_err(message)?;
                self.constraints
                    .retain(|(_, k, _)| !removed.constraints.contains(k));
                self.geometry.retain(|(key, _)| *key != id);
            }
            Command::AddConstraint { relation } => {
                let definition = self.relation(&relation)?;
                let key = self.system.add_constraint(definition).map_err(message)?;
                self.constraints.push((self.next_id, key, relation));
                self.next_id += 1;
            }
            Command::UpdateConstraint { id, relation } => {
                let definition = self.relation(&relation)?;
                let entry = self
                    .constraints
                    .iter_mut()
                    .find(|(key, _, _)| *key == id)
                    .ok_or("Unknown constraint")?;
                self.system
                    .update_constraint(entry.1, definition)
                    .map_err(message)?;
                entry.2 = relation;
            }
            Command::RemoveConstraint { id } => {
                let index = self
                    .constraints
                    .iter()
                    .position(|(key, _, _)| *key == id)
                    .ok_or("Unknown constraint")?;
                self.system
                    .remove_constraint(self.constraints[index].1)
                    .map_err(message)?;
                self.constraints.remove(index);
            }
            Command::Solve { target } => {
                let targets = target
                    .map(|t| {
                        self.point(t.point)
                            .map(|p| vec![Target2d::point(p, t.position.into())])
                    })
                    .transpose()?
                    .unwrap_or_default();
                self.suspects.clear();
                match self.system.solve_toward(&targets) {
                    Ok(solution) => {
                        self.report = Some(solution.report().clone());
                        self.status = "solved".into();
                        self.message.clear();
                    }
                    Err(error) => {
                        self.report = None;
                        self.status = "error".into();
                        self.message=match &error {SolveError::Conflict {..}=>"Contraintes incompatibles.",_=>"Le calcul local n’a pas convergé. Les contraintes signalées appartiennent au groupe concerné."}.into();
                        if let SolveError::Conflict { constraints }
                        | SolveError::NonConvergence { constraints, .. } = error
                        {
                            self.suspects = self
                                .constraints
                                .iter()
                                .filter(|(_, k, _)| constraints.contains(k))
                                .map(|(id, _, _)| *id)
                                .collect();
                        }
                    }
                }
                return Ok(());
            }
        }
        self.edited();
        Ok(())
    }
    fn view(&self) -> Result<View, String> {
        let geometry = self
            .geometry
            .iter()
            .map(|(id, shape)| {
                let geometry = match shape {
                    Shape::Point(k) => Geometry::Point {
                        position: xy(self.system.point(*k).map_err(message)?),
                    },
                    Shape::Segment(k) => {
                        let curve = self.system.segment(*k).map_err(message)?;
                        Geometry::Segment {
                            start: xy(curve.start()),
                            end: xy(curve.end()),
                        }
                    }
                    Shape::Circle(k) => {
                        let circle = self.system.circle(*k).map_err(message)?;
                        Geometry::Circle {
                            center: xy(circle.center()),
                            radius: circle.radius(),
                        }
                    }
                    Shape::Arc(k) => {
                        let arc = self.system.arc(*k).map_err(message)?;
                        let center = self.system.point(k.center()).map_err(message)?;
                        let radial = arc.start() - center;
                        Geometry::Arc {
                            center: xy(center),
                            radius: radial.norm(),
                            start_angle: radial.y.atan2(radial.x),
                            sweep: arc.interval().delta(),
                        }
                    }
                };
                Ok(GeometryView { id: *id, geometry })
            })
            .collect::<Result<_, String>>()?;
        let constraints = self
            .constraints
            .iter()
            .map(|(id, key, relation)| ConstraintView {
                id: *id,
                relation: relation.clone(),
                suspect: self.suspects.contains(id),
                warning: self
                    .report
                    .as_ref()
                    .is_some_and(|r| r.redundancies.iter().any(|v| v.constraint == *key)),
                ratio: self.report.as_ref().and_then(|r| {
                    r.residuals
                        .iter()
                        .find(|v| v.constraint == *key)
                        .map(|r| r.max_tolerance_ratio)
                }),
            })
            .collect();
        Ok(View {
            geometry,
            constraints,
            status: self.status.clone(),
            message: self.message.clone(),
            dof: self.report.as_ref().map(|r| r.degrees_of_freedom),
            target_distance: self
                .report
                .as_ref()
                .and_then(|r| r.targets.first().map(|t| t.distance)),
            iterations: self
                .report
                .as_ref()
                .map_or(0, |r| r.iterations + r.target_iterations),
        })
    }
}
fn xy(point: Point2) -> [f64; 2] {
    [point.x, point.y]
}
fn message(error: impl std::fmt::Display) -> String {
    error.to_string()
}
