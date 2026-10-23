use super::super::{
    Constraint2d, ConstraintKey, GeometryKey, PointRef, SegmentKey, SolveOptions, TangencyMode,
    Target2d,
};
use super::super::{
    constraints::{StoredConstraint, TargetLocation},
    keys::PointRole,
    primitives::{Kind, Param, Quantity},
};
use super::{
    curve::Curve,
    differentiate::{Jet, JetVector2},
    prepare::Component,
};
use crate::geometry::dim2::utils::Point2;
use Constraint2d::*;
use nalgebra::{DMatrix, DVector};
use std::ops::Range;

pub(super) struct Row {
    pub expression: Jet,
    pub angular: bool,
}
pub(super) struct Evaluation {
    pub values: DVector<f64>,
    pub jacobian: DMatrix<f64>,
    pub angular: Vec<bool>,
    pub groups: Vec<(ConstraintKey, Range<usize>)>,
}
impl Evaluation {
    fn new(rows: Vec<Row>, groups: Vec<(ConstraintKey, Range<usize>)>, columns: usize) -> Self {
        let mut jacobian = DMatrix::zeros(rows.len(), columns);
        for (i, row) in rows.iter().enumerate() {
            for &(j, v) in &row.expression.gradient {
                jacobian[(i, j)] = v;
            }
        }
        Self {
            values: DVector::from_iterator(rows.len(), rows.iter().map(|r| r.expression.value)),
            jacobian,
            angular: rows.iter().map(|r| r.angular).collect(),
            groups,
        }
    }
    pub fn finite(&self) -> bool {
        self.values
            .iter()
            .chain(self.jacobian.iter())
            .all(|v| v.is_finite())
    }
    pub fn feasible(&self, scale: f64, options: &SolveOptions) -> bool {
        self.finite()
            && self.values.iter().zip(&self.angular).all(|(v, a)| {
                v.abs()
                    <= if *a {
                        options.angular_tolerance
                    } else {
                        options.linear_tolerance / scale
                    }
            })
    }
}

pub(super) fn hard(component: &Component, x: &DVector<f64>) -> Evaluation {
    permanent(Context {
        component,
        x,
        world: false,
    })
}
/// Audits published coordinates, including rounding at large world offsets.
pub(super) fn hard_world(component: &Component, x: &DVector<f64>) -> Evaluation {
    permanent(Context {
        component,
        x,
        world: true,
    })
}
fn permanent(ctx: Context<'_>) -> Evaluation {
    let mut rows = vec![];
    let mut groups = vec![];
    for (key, c) in &ctx.component.constraints {
        let start = rows.len();
        rows.extend(ctx.constraint(c));
        groups.push((*key, start..rows.len()));
    }
    Evaluation::new(rows, groups, ctx.x.len())
}
pub(super) fn soft(component: &Component, x: &DVector<f64>) -> Evaluation {
    let ctx = Context {
        component,
        x,
        world: false,
    };
    let mut rows = vec![];
    for (_, target) in &component.targets {
        let actual = ctx.target(target);
        let wanted = ctx.constant_point(target.position);
        let delta = (actual - wanted).scale(Jet::constant(target.weight.sqrt()));
        rows.extend(pair(delta));
    }
    Evaluation::new(rows, vec![], x.len())
}
pub(super) fn target_point_world(
    component: &Component,
    x: &DVector<f64>,
    target: &Target2d,
) -> Point2 {
    let p = Context {
        component,
        x,
        world: true,
    }
    .target(target);
    Point2::new(p.x.value, p.y.value)
}
fn linear(expression: Jet) -> Row {
    Row {
        expression,
        angular: false,
    }
}
fn angular(expression: Jet) -> Row {
    Row {
        expression,
        angular: true,
    }
}
fn pair(v: JetVector2) -> Vec<Row> {
    vec![linear(v.x), linear(v.y)]
}

struct Context<'a> {
    component: &'a Component,
    x: &'a DVector<f64>,
    world: bool,
}
impl Context<'_> {
    /// A parameter as a differentiable expression of the unknowns. In world
    /// mode `x` holds every parameter slot directly, without reduction.
    fn parameter(&self, key: GeometryKey, param: Param) -> Jet {
        let slot = self.component.slot(key, param);
        if self.world {
            Jet::variable(self.x[slot], slot)
        } else if let Some(column) = self.component.columns[slot] {
            Jet::variable(self.x[column], column) * self.component.multipliers[slot]
                + self.component.offsets[slot]
        } else {
            Jet::constant(self.component.offsets[slot])
        }
    }
    fn coordinates(&self, key: GeometryKey, x: Param, y: Param) -> JetVector2 {
        JetVector2 {
            x: self.parameter(key, x),
            y: self.parameter(key, y),
        }
    }
    fn center(&self, key: GeometryKey) -> JetVector2 {
        self.coordinates(key, Param::X, Param::Y)
    }
    fn radius(&self, key: GeometryKey) -> Jet {
        self.parameter(key, Param::Radius)
    }
    /// A curve primitive with its shape read from the unknowns.
    fn curve(&self, key: GeometryKey) -> Curve {
        let primitive = &self.component.entry(key).primitive;
        match primitive.kind {
            Kind::Segment => Curve::Segment {
                start: self.coordinates(key, Param::X, Param::Y),
                end: self.coordinates(key, Param::EndX, Param::EndY),
            },
            Kind::Circle => Curve::Circle {
                center: self.center(key),
                radius: self.radius(key),
                axis: primitive.circle_axis,
                sense: primitive.sense,
            },
            Kind::Arc => Curve::Arc {
                center: self.center(key),
                radius: self.radius(key),
                start_angle: self.parameter(key, Param::StartAngle),
                sweep: self.parameter(key, Param::Sweep),
                sense: primitive.sense,
            },
            Kind::Point => unreachable!("typed keys never use a point as a curve"),
        }
    }
    /// Stored points are read directly; an arc's endpoints lie on its curve.
    fn point(&self, point: PointRef) -> JetVector2 {
        let key = point.geometry;
        let kind = self.component.entry(key).primitive.kind;
        match kind.point_params(point.role) {
            Some((x, y)) => self.coordinates(key, x, y),
            None => {
                let t = if point.role == PointRole::Start {
                    0.
                } else {
                    1.
                };
                self.curve(key).point_at(Jet::constant(t))
            }
        }
    }
    /// A segment's derivative is the same everywhere: its end minus its start.
    fn direction(&self, line: SegmentKey) -> JetVector2 {
        self.curve(line.0).derivative_at(Jet::constant(0.))
    }
    fn constant_point(&self, p: Point2) -> JetVector2 {
        if self.world {
            return JetVector2 {
                x: Jet::constant(p.x),
                y: Jet::constant(p.y),
            };
        }
        JetVector2 {
            x: Jet::constant((p.x - self.component.origin.x) / self.component.scale),
            y: Jet::constant((p.y - self.component.origin.y) / self.component.scale),
        }
    }
    fn line_distance(&self, point: JetVector2, line: SegmentKey) -> Jet {
        self.direction(line)
            .unit()
            .cross(point - self.point(line.start()))
    }
    fn target(&self, target: &Target2d) -> JetVector2 {
        match target.location {
            TargetLocation::Point(p) => self.point(p),
            TargetLocation::Sample(key, t) => self.curve(key).point_at(Jet::constant(t)),
        }
    }
    /// Emits geometric residuals and exact local derivatives with permanent-row provenance.
    fn constraint(&self, c: &StoredConstraint) -> Vec<Row> {
        let scale = if self.world { 1. } else { self.component.scale };
        match &c.definition {
            Coincident(a, b) => pair(self.point(*a) - self.point(*b)),
            PointOnLine(p, l) => vec![linear(self.line_distance(self.point(*p), *l))],
            PointOnCircle(p, r) => vec![linear(
                (self.point(*p) - self.center(r.0)).norm() - self.radius(r.0),
            )],
            Horizontal(l) | Vertical(l) => {
                let direction = self.direction(*l);
                let angle = direction.y.atan2(direction.x);
                let target = if matches!(c.definition, Horizontal(_)) {
                    if c.side < 0. {
                        std::f64::consts::PI
                    } else {
                        0.
                    }
                } else {
                    c.side * std::f64::consts::FRAC_PI_2
                };
                vec![angular((angle - target).wrapped())]
            }
            Distance(a, b, v) if *v == 0. => pair(self.point(*a) - self.point(*b)),
            Distance(a, b, v) => vec![linear(
                (self.point(*b) - self.point(*a)).norm() - *v / scale,
            )],
            DistanceToLine(p, l, v) => vec![linear(
                self.line_distance(self.point(*p), *l) - c.side * *v / scale,
            )],
            OffsetX(a, b, v) => vec![linear(self.point(*b).x - self.point(*a).x - *v / scale)],
            OffsetY(a, b, v) => vec![linear(self.point(*b).y - self.point(*a).y - *v / scale)],
            Length(l, v) => vec![linear(self.direction(*l).norm() - *v / scale)],
            Radius(r, v) => vec![linear(self.radius(r.0) - *v / scale)],
            EqualLength(a, b) => vec![linear(
                self.direction(*a).norm() - self.direction(*b).norm(),
            )],
            EqualRadius(a, b) => vec![linear(self.radius(a.0) - self.radius(b.0))],
            Parallel(a, b) | Perpendicular(a, b) => {
                let first = self.direction(*a).unit();
                let second = self.direction(*b).unit();
                let angle = first.clone().cross(second.clone()).atan2(first.dot(second));
                let target = if matches!(c.definition, Parallel(..)) {
                    if c.side < 0. {
                        std::f64::consts::PI
                    } else {
                        0.
                    }
                } else {
                    c.side * std::f64::consts::FRAC_PI_2
                };
                vec![angular((angle - target).wrapped())]
            }
            Angle {
                first,
                second,
                radians,
                oriented,
            } => {
                let a = self.direction(*first).unit();
                let b = self.direction(*second).unit();
                let angle = a.clone().cross(b.clone()).atan2(a.dot(b));
                vec![angular(
                    (angle
                        - if *oriented {
                            *radians
                        } else {
                            c.side * *radians
                        })
                    .wrapped(),
                )]
            }
            Concentric(a, b) => pair(self.center(a.0) - self.center(b.0)),
            Midpoint { point, start, end } => pair(
                self.point(*point)
                    - (self.point(*start) + self.point(*end)).scale(Jet::constant(0.5)),
            ),
            SymmetricAboutPoint {
                first,
                second,
                center,
            } => pair(
                (self.point(*first) + self.point(*second)).scale(Jet::constant(0.5))
                    - self.point(*center),
            ),
            SymmetricAboutLine {
                first,
                second,
                line,
            } => {
                let a = self.point(*first);
                let b = self.point(*second);
                let middle = (a.clone() + b.clone()).scale(Jet::constant(0.5));
                vec![
                    linear(self.line_distance(middle, *line)),
                    linear((b - a).dot(self.direction(*line).unit())),
                ]
            }
            FixedPoint(p) => pair(
                self.point(*p) - self.constant_point(Point2::new(c.captured[0], c.captured[1])),
            ),
            Fixed(key) => {
                let layout = self.component.entry(*key).primitive.kind.layout();
                layout
                    .iter()
                    .zip(&c.captured)
                    .map(|(&param, &value)| {
                        let constant = if self.world {
                            value
                        } else {
                            self.component.normalized(value, param)
                        };
                        let residual = self.parameter(*key, param) - constant;
                        match param {
                            Param::StartAngle => angular(residual.wrapped()),
                            _ if param.quantity() == Quantity::Angle => angular(residual),
                            _ => linear(residual),
                        }
                    })
                    .collect()
            }
            TangentLineCircle(l, r) => vec![linear(
                self.line_distance(self.center(r.0), *l) - self.radius(r.0) * c.side,
            )],
            TangentCircles { first, second, .. } => {
                let separation = (self.center(second.0) - self.center(first.0)).norm();
                let a = self.radius(first.0);
                let b = self.radius(second.0);
                let radii = match c.tangency {
                    TangencyMode::FirstContainsSecond => a - b,
                    TangencyMode::SecondContainsFirst => b - a,
                    _ => a + b,
                };
                vec![linear(separation - radii)]
            }
        }
    }
}
