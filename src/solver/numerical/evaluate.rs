use super::super::{
    Constraint2d, ConstraintKey, GeometryKey, PointRef, SolveOptions, TangencyMode, Target2d,
};
use super::super::{
    constraints::{StoredConstraint, TargetLocation},
    keys::PointRole,
    primitives::Kind,
};
use super::{
    differentiate::{Jet, V2},
    prepare::Component,
};
use crate::geometry::dim2::utils::Point2;
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
fn pair(v: V2) -> Vec<Row> {
    vec![linear(v.x), linear(v.y)]
}

struct Context<'a> {
    component: &'a Component,
    x: &'a DVector<f64>,
    world: bool,
}
impl Context<'_> {
    fn parameter(&self, key: GeometryKey, index: usize) -> Jet {
        let index = self.component.entry(key).offset + index;
        if self.world {
            Jet::variable(self.x[index], index)
        } else if let Some(column) = self.component.columns[index] {
            Jet::variable(self.x[column], column) * self.component.multipliers[index]
                + self.component.offsets[index]
        } else {
            Jet::constant(self.component.offsets[index])
        }
    }
    fn center(&self, key: GeometryKey) -> V2 {
        V2 {
            x: self.parameter(key, 0),
            y: self.parameter(key, 1),
        }
    }
    fn point(&self, point: PointRef) -> V2 {
        let key = point.geometry;
        let primitive = &self.component.entry(key).primitive;
        match (primitive.kind, point.role) {
            (Kind::Segment, PointRole::End) => V2 {
                x: self.parameter(key, 2),
                y: self.parameter(key, 3),
            },
            (Kind::Arc, PointRole::Start) => self.arc_point(key, 0.),
            (Kind::Arc, PointRole::End) => self.arc_point(key, 1.),
            _ => self.center(key),
        }
    }
    fn arc_point(&self, key: GeometryKey, fraction: f64) -> V2 {
        let primitive = &self.component.entry(key).primitive;
        let angle = self.parameter(key, 3) + self.parameter(key, 4) * (primitive.sense * fraction);
        let radial = V2 {
            x: angle.clone().cos(),
            y: angle.sin(),
        }
        .scale(self.parameter(key, 2));
        self.center(key) + radial
    }
    fn direction(&self, line: super::super::SegmentKey) -> V2 {
        self.point(line.end()) - self.point(line.start())
    }
    fn constant_point(&self, p: Point2) -> V2 {
        if self.world {
            return V2 {
                x: Jet::constant(p.x),
                y: Jet::constant(p.y),
            };
        }
        V2 {
            x: Jet::constant((p.x - self.component.origin.x) / self.component.scale),
            y: Jet::constant((p.y - self.component.origin.y) / self.component.scale),
        }
    }
    fn line_distance(&self, point: V2, line: super::super::SegmentKey) -> Jet {
        self.direction(line)
            .unit()
            .cross(point - self.point(line.start()))
    }
    fn target(&self, target: &Target2d) -> V2 {
        match target.location {
            TargetLocation::Point(p) => self.point(p),
            TargetLocation::Sample(key, t) => {
                let primitive = &self.component.entry(key).primitive;
                match primitive.kind {
                    Kind::Segment => {
                        let start = self.center(key);
                        let end = V2 {
                            x: self.parameter(key, 2),
                            y: self.parameter(key, 3),
                        };
                        start.clone() + (end - start).scale(Jet::constant(t))
                    }
                    Kind::Arc => self.arc_point(key, t),
                    Kind::Circle => {
                        let a = &primitive.circle_axis;
                        let radial = V2 {
                            x: Jet::constant(a.x * t.cos() - primitive.sense * a.y * t.sin()),
                            y: Jet::constant(a.y * t.cos() + primitive.sense * a.x * t.sin()),
                        };
                        self.center(key) + radial.scale(self.parameter(key, 2))
                    }
                    Kind::Point => self.center(key),
                }
            }
        }
    }
    /// Emits geometric residuals and exact local derivatives with permanent-row provenance.
    fn constraint(&self, c: &StoredConstraint) -> Vec<Row> {
        let scale = if self.world { 1. } else { self.component.scale };
        use Constraint2d::*;
        match &c.definition {
            Coincident(a, b) => pair(self.point(*a) - self.point(*b)),
            PointOnLine(p, l) => vec![linear(self.line_distance(self.point(*p), *l))],
            PointOnCircle(p, r) => vec![linear(
                (self.point(*p) - self.center(r.0)).norm() - self.parameter(r.0, 2),
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
            Radius(r, v) => vec![linear(self.parameter(r.0, 2) - *v / scale)],
            EqualLength(a, b) => vec![linear(
                self.direction(*a).norm() - self.direction(*b).norm(),
            )],
            EqualRadius(a, b) => vec![linear(self.parameter(a.0, 2) - self.parameter(b.0, 2))],
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
                let kind = self.component.entry(*key).primitive.kind;
                c.captured
                    .iter()
                    .enumerate()
                    .map(|(i, value)| {
                        let constant = if self.world {
                            *value
                        } else {
                            self.component.normalized(*value, kind, i)
                        };
                        let residual = self.parameter(*key, i) - constant;
                        if kind == Kind::Arc && i >= 3 {
                            angular(if i == 3 { residual.wrapped() } else { residual })
                        } else {
                            linear(residual)
                        }
                    })
                    .collect()
            }
            TangentLineCircle(l, r) => vec![linear(
                self.line_distance(self.center(r.0), *l) - self.parameter(r.0, 2) * c.side,
            )],
            TangentCircles { first, second, .. } => {
                let separation = (self.center(second.0) - self.center(first.0)).norm();
                let a = self.parameter(first.0, 2);
                let b = self.parameter(second.0, 2);
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
